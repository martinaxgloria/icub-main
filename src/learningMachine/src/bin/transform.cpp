/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Executable for the Transform Module
 *
 */

#include <iostream>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "iCub/TransformModule.h"
#include "iCub/TransformerCatalogue.h"
#include "iCub/EventListenerCatalogue.h"

using namespace iCub::learningmachine;

int main (int argc, char* argv[]) {
    Network yarp;
    int ret;

    ResourceFinder rf;
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setDefaultContext("learningMachine");

    TransformModule module;
    try {
        // initialize catalogue of transformers
        registerTransformers();

        // initialize catalogue of event listeners
        registerEventListeners();

        ret = module.runModule(rf);
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch(char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }
    return ret;
}

