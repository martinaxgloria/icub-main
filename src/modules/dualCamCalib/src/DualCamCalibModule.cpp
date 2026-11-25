// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <iCub/DualCamCalibModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include <algorithm>
#include <cstring>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

CamCalibModule::CamCalibModule()
{
    calibToolLeft = NULL;
    calibToolRight = NULL;
    align=ALIGN_WIDTH;
    requested_fps=0;
    time_lastOut=yarp::os::Time::now();
    dualImage_mode = false;
}

CamCalibModule::~CamCalibModule()
{
    calibToolLeft = NULL;
    calibToolRight = NULL;
}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf)
{
    string str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
    verboseExecTime = rf.check("verboseExecTime");

    if      (rf.check("w_align")) align=ALIGN_WIDTH;
    else if (rf.check("h_align")) align=ALIGN_HEIGHT;

    if (rf.check("fps"))
    {
        requested_fps=rf.find("fps").asFloat64();
        yInfo() << "Module will run at " << requested_fps;
    }
    else
    {
        yInfo() << "Module will run at max fps";
    }

    setName(str.c_str()); // modulePortName

    Bottle botLeftConfig(rf.toString().c_str());
    Bottle botRightConfig(rf.toString().c_str());

    string strLeftGroup = "CAMERA_CALIBRATION_LEFT";
    if (botLeftConfig.check(strLeftGroup.c_str()))
    {            
        Bottle &group=botLeftConfig.findGroup(strLeftGroup.c_str(),string("Loading configuration from group " + strLeftGroup).c_str());
        botLeftConfig.fromString(group.toString());
    }
    else
    {
        yError() << "Group " << strLeftGroup << " not found.";
        return false;
    }
    
    string strRightGroup = "CAMERA_CALIBRATION_RIGHT";
    if (botRightConfig.check(strRightGroup.c_str()))
    {            
        Bottle &group=botRightConfig.findGroup(strRightGroup.c_str(),string("Loading configuration from group " + strRightGroup).c_str());
        botRightConfig.fromString(group.toString());
    }
    else
    {
        yError() << "Group " << strRightGroup << " not found.";
        return false;
    }

    string calibToolLeftName  = botLeftConfig.check("projection", Value("pinhole"), "Projection/mapping applied to calibrated image [projection|spherical|fisheye] (string).").asString().c_str();
    string calibToolRightName = botRightConfig.check("projection", Value("pinhole"), "Projection/mapping applied to calibrated image [projection|spherical|fisheye] (string).").asString().c_str();

    // Stereo fisheye rectification needs both cameras and the stereo transform.
    Property stereoConfig;
    stereoConfig.fromString(rf.toString());

    calibToolLeft = CalibToolFactories::getPool().get(calibToolLeftName.c_str());
    if (calibToolLeft!=NULL) {
        stereoConfig.put("group", strLeftGroup);
        bool ok = calibToolLeftName == "fisheye" ? calibToolLeft->open(stereoConfig)
                                                : calibToolLeft->open(botLeftConfig);
        if (!ok) {
            delete calibToolLeft;
            calibToolLeft = NULL;
            return false;
        }
    }
    calibToolRight = CalibToolFactories::getPool().get(calibToolRightName.c_str());
    if (calibToolRight!=NULL) {
        stereoConfig.put("group", strRightGroup);
        bool ok = calibToolRightName == "fisheye" ? calibToolRight->open(stereoConfig)
                                                 : calibToolRight->open(botRightConfig);
        if (!ok) {
            delete calibToolRight;
            calibToolRight = NULL;
            return false;
        }
    }

    if(rf.check("dual"))
    {
        dualImage_mode = true;
        yInfo() << "Dual mode activate!!";
    }

    if(dualImage_mode)
    {
        leftImage  = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
        rightImage = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;

        // open a single port with name /dual:i
        if (yarp::os::Network::exists(getName("/dual:i")))
        {
            yWarning() << "port " << getName("/dual:i") << " already in use";
        }
        if(! imageInLeft.open(getName("/dual:i")) )
            return false;
        imageInLeft.setStrict(false);
    }
    else
    {
        if (yarp::os::Network::exists(getName("/left:i")))
        {
            yWarning() << "port " << getName("/left:i") << " already in use";
        }
        if (yarp::os::Network::exists(getName("/right:i")))
        {
            yWarning() << "port " << getName("/right:i") << " already in use";
        }
        imageInLeft.open(getName("/left:i"));
        imageInRight.open(getName("/right:i"));
        imageInLeft.setStrict(false);
        imageInRight.setStrict(false);
    }

    if (yarp::os::Network::exists(getName("/out")))
    {
        yWarning() << "port " << getName("/out") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/conf")))
    {
        yWarning() << "port " << getName("/conf") << " already in use";
    }
    imageOut.open(getName("/out:o"));
    configPort.open(getName("/conf"));
    attach(configPort);
    return true;
}

bool CamCalibModule::close()
{
    yTrace();
    imageInLeft.close();
    if(!dualImage_mode)
        imageInRight.close();
    imageOut.close();
    configPort.close();
    if (calibToolLeft != NULL)
    {
        calibToolLeft->close();
        delete calibToolLeft;
        calibToolLeft = NULL;
    }
    if (calibToolRight != NULL)
    {
        calibToolRight->close();
        delete calibToolRight;
        calibToolRight = NULL;
    }
    return true;
}

bool CamCalibModule::interruptModule()
{
    imageInLeft.interrupt();
    if(!dualImage_mode)
        imageInRight.interrupt();
    imageOut.interrupt();
    configPort.interrupt();
    return true;
}

bool CamCalibModule::updateModule()
{
    bool lready=false;
    bool rready=false;

    int dual_rowsize_pixels, dual_height_pixels, single_rowsize_pixels;
    int dual_rowsize_bytes,  single_rowsize_bytes;

    unsigned char *left_raw, *right_raw, *dual_raw;

    if(dualImage_mode)
    {
        // read the dual image and split it up into 2 separated images for calibration
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* dual = imageInLeft.read();
        if(dual == NULL)
        {
            yarp::os::Time::delay(0.001);
            return true;
        }

        dual_rowsize_pixels = dual->width();
        single_rowsize_pixels = dual_rowsize_pixels/2;
        dual_height_pixels = dual->height();

        dual_rowsize_bytes = dual->getRowSize();
        single_rowsize_bytes = dual_rowsize_bytes/2;

        leftImage->resize(dual_rowsize_pixels/2, dual_height_pixels);
        rightImage->resize(dual_rowsize_pixels/2, dual_height_pixels);

        leftImage->setQuantum(dual->getQuantum());
        rightImage->setQuantum(dual->getQuantum());

        left_raw  = leftImage->getRawImage();
        right_raw = rightImage->getRawImage();
        dual_raw  = dual->getRawImage();

        for(int h=0; h<dual_height_pixels; h++)
        {
            memcpy(left_raw+(h*single_rowsize_bytes),  dual_raw+(h*dual_rowsize_bytes),                         single_rowsize_bytes);
            memcpy(right_raw+(h*single_rowsize_bytes), dual_raw+(h*dual_rowsize_bytes) + single_rowsize_bytes,  single_rowsize_bytes);
        }
    }
    else
    {
        leftImage  = imageInLeft.read(false);
        rightImage = imageInRight.read(false);
    }

    if (calibToolLeft!=NULL && leftImage!=NULL && leftImage->width()>0 && leftImage->height()>0)
    {
        calibToolLeft->apply(*leftImage,calibratedImgLeft);
        lready=true;
    }
    if (calibToolRight!=NULL && rightImage!=NULL && rightImage->width()>0 && rightImage->height()>0)
    {
        calibToolRight->apply(*rightImage,calibratedImgRight);
        rready=true;
    }

    // Keep the last calibrated frame from each input: the cameras arrive independently.
    const int frameWidth = std::max(calibratedImgLeft.width(), calibratedImgRight.width());
    const int frameHeight = std::max(calibratedImgLeft.height(), calibratedImgRight.height());
    if (frameWidth<=0 || frameHeight<=0)
        return true;

    const double now = yarp::os::Time::now();
    if (requested_fps==0)
    {
        if (!lready && !rready)
            return true;
    }
    else if (now-time_lastOut<=1/requested_fps)
    {
        return true;
    }

    const int outw = align==ALIGN_WIDTH ? frameWidth*2 : frameWidth;
    const int outh = align==ALIGN_HEIGHT ? frameHeight*2 : frameHeight;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>& calibratedImgOut = imageOut.prepare();
    calibratedImgOut.resize(outw, outh);
    calibratedImgOut.zero();

    auto copyFrame = [&](const yarp::sig::ImageOf<yarp::sig::PixelRgb>& image, int x, int y)
    {
        if (image.width()==0 || image.height()==0)
            return;
        for (int row=0; row<image.height(); ++row)
        {
            std::memcpy(calibratedImgOut.getPixelAddress(x,y+row),
                        image.getPixelAddress(0,row), image.width()*image.getPixelSize());
        }
    };
    copyFrame(calibratedImgLeft, 0, 0);
    copyFrame(calibratedImgRight, align==ALIGN_WIDTH ? frameWidth : 0,
                                 align==ALIGN_HEIGHT ? frameHeight : 0);

    yarp::os::Stamp stamp;
    imageOut.setEnvelope(stamp);

    imageOut.writeStrict();
    if (verboseExecTime) yDebug ("%f", now-time_lastOut);
    time_lastOut=now;

    return true;
}

double CamCalibModule::getPeriod()
{
  return 0.0;
}

bool CamCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else
    {
        yError() << "command not known - type help for more info";
    }
    return true;
}


