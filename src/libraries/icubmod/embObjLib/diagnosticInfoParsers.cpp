/*
 * Copyright (C) Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include "diagnosticLowLevelFormatter_hid.h"
#include "diagnosticLowLevelFormatter.h"
#include "EoBoards.h"


using namespace Diagnostic::LowLevel;




/**************************************************************************************************************************/
/******************************************   DefaultParser   ***************************************************/
/**************************************************************************************************************************/
DefaultParser::DefaultParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):m_dnginfo(dnginfo), m_entityNameProvider(entityNameProvider){;}

void DefaultParser::parseInfo()
{
    char str[512] = {0};
    uint8_t *p64 = (uint8_t*)&(m_dnginfo.param64);
    snprintf(str, sizeof(str), " src %s, adr %d,(code 0x%.8x, par16 0x%.4x par64 0x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x) -> %s %s %s",
                                m_dnginfo.baseInfo.sourceCANPortStr.c_str(),
                                m_dnginfo.baseInfo.sourceCANBoardAddr,
                                m_dnginfo.errorCode,
                                m_dnginfo.param16,
                                p64[7], p64[6], p64[5], p64[4], p64[3], p64[2], p64[1], p64[0],
                                eoerror_code2string(m_dnginfo.errorCode),
                                eoerror_code2rulesstring(m_dnginfo.errorCode),
                                m_dnginfo.extraMessage.c_str()
                                );
    m_dnginfo.baseInfo.finalMessage.clear();
    m_dnginfo.baseInfo.finalMessage.append(str);

}

void DefaultParser::printBaseInfo()
{
    char str[512] = {0};
    snprintf(str, sizeof(str), "%s", m_dnginfo.baseMessage.c_str());
    m_dnginfo.baseInfo.finalMessage.append(str);
}


/**************************************************************************************************************************/
/******************************************   ConfigParser   ***************************************************/
/**************************************************************************************************************************/

ConfigParser::ConfigParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void ConfigParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {

        case eoerror_value_CFG_candiscovery_started:
        {
            uint16_t maskcan2 = m_dnginfo.param16;
            eObrd_type_t brdnum =     static_cast<eObrd_type_t>((m_dnginfo.param64 & 0x0000ff0000000000) >> 40);
            const char *canboardname = eoboards_type2string(brdnum);
            uint16_t maskcan1 = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr =      (m_dnginfo.param64 & 0x000000ffff000000) >> 24;
            uint64_t reqfw =      (m_dnginfo.param64 & 0x0000000000ffffff);
            uint8_t num =0;
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 16) & 0xff;
            appl.minor = (reqfw >> 8)  & 0xff;
            appl.build = reqfw & 0xff;
            num = eo_common_hlfword_bitsetcount(maskcan1)+eo_common_hlfword_bitsetcount(maskcan2);

            snprintf(str, sizeof(str), " %s %d %s boards on (can1map, can2map) = (0x%.4x, 0x%.4x) with target can protocol ver %d.%d and application ver %d.%d.%d.",
                                        m_dnginfo.baseMessage.c_str(),
                                        num, canboardname,
                                        maskcan1, maskcan2,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor, appl.build
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_candiscovery_ok:
        {
            uint8_t num = m_dnginfo.param16 & 0x00ff;
            eObool_t fakesearch = (0x0000 == (m_dnginfo.param16 & 0xf000)) ? (eobool_false) : (eobool_true);
            uint64_t brdnum =     (m_dnginfo.param64 & 0x0000ff0000000000) >> 40;
            const char *canboardname = eoboards_type2string(static_cast<eObrd_type_t>(brdnum));
            uint64_t searchtime = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr =      (m_dnginfo.param64 & 0x000000ffff000000) >> 24;
            uint64_t reqfw =      (m_dnginfo.param64 & 0x0000000000ffffff);
            char strOK[80] = "OK";

            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 16) & 0xff;
            appl.minor = (reqfw >> 8)  & 0xff;
            appl.build = reqfw & 0xff;

           
            if(eobool_true == fakesearch)
            {
                snprintf(strOK, sizeof(strOK), "OK but FAKE (without any control on CAN w/ get-fw-version<> message)");
            }

            snprintf(str, sizeof(str), "%s is %s for %d %s boards with target can protocol ver %d.%d and application ver %d.%d.%d. Search time was %d ms",
                                        m_dnginfo.baseMessage.c_str(),
                                        strOK,
                                        num, canboardname,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor, appl.build,
                                        (int)searchtime
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_candiscovery_detectedboard:
        {
            uint64_t brdnum =     (m_dnginfo.param64 & 0x0000ff0000000000) >> 40;
            const char *canboardname = eoboards_type2string(static_cast<eObrd_type_t>(brdnum));
            uint64_t searchtime = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr =      (m_dnginfo.param64 & 0x000000ffff000000) >> 24;
            uint64_t reqfw =      (m_dnginfo.param64 & 0x0000000000ffffff);
            uint8_t address;
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 16) & 0xff;
            appl.minor = (reqfw >> 8)  & 0xff;
            appl.build = reqfw & 0xff;
            address = m_dnginfo.param16 & 0x000f;


            snprintf(str, sizeof(str), "%s %s board in %s addr %d with can protocol ver %d.%d and application ver %d.%d.%d Search time was %d ms",
                                        m_dnginfo.baseMessage.c_str(),
                                        canboardname,
                                        m_dnginfo.baseInfo.sourceCANPortStr.c_str(), address,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor, appl.build,
                                        (int)searchtime
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_candiscovery_boardsmissing:
        {
            uint8_t numofmissing = m_dnginfo.param16 & 0x00ff;
            const char *canboardname = eoboards_type2string(static_cast<eObrd_type_t>((m_dnginfo.param16 >> 8)));
            uint64_t searchtime = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            uint16_t maskofmissing = m_dnginfo.param64 & 0x000000000000ffff;

            uint8_t n = 1;
            uint8_t i = 0;

            snprintf(str, sizeof(str), "%s %d missing %s boards for %d ms in %s:",
                                        m_dnginfo.baseMessage.c_str(),
                                        numofmissing,
                                        canboardname,
                                        (int)searchtime,
                                        m_dnginfo.baseInfo.sourceCANPortStr.c_str()
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
            for(i=1; i<15; i++)
            {
                if(eobool_true == eo_common_hlfword_bitcheck(maskofmissing, i))
                {
                    snprintf(str, sizeof(str), "%d of %d: missing %s BOARD %s:%s:%d",
                                                n, numofmissing, canboardname,
                                                m_dnginfo.baseInfo.sourceBoardIpAddrStr.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), i
                                                );
                    m_dnginfo.baseInfo.finalMessage.append(str);
                    n++;

                }
            }

        } break;

        case eoerror_value_CFG_candiscovery_boardsinvalid:
        {
            uint8_t numofinvalid = m_dnginfo.param16 & 0x00ff;
            const char *canboardname = eoboards_type2string(static_cast<eObrd_type_t>(m_dnginfo.param16 >> 8));
            uint64_t invalidmask = m_dnginfo.param64;
            uint8_t n = 1;
            uint8_t i = 0;
            const char *empty = "";
            const char *wrongtype = "WRONG BOARD TYPE";
            const char *wrongprot = "WRONG PROTOCOL VERSION";
            const char *wrongappl = "WRONG APPLICATION VERSION";

            snprintf(str, sizeof(str), "%s %d invalid %s boards in %s:\n",
                                        m_dnginfo.baseMessage.c_str(),
                                        numofinvalid,
                                        canboardname,
                                        m_dnginfo.baseInfo.sourceCANPortStr.c_str()
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);

            for(int i=1; i<15; i++)
            {
                uint64_t val = (invalidmask >> (4*i)) & 0x0f;
                if(0 != val)
                {
                    snprintf(str, sizeof(str), "\t %d of %d: wrong %s because it has: %s%s%s \n",
                                                n, numofinvalid, canboardname,
                                                ((val & 0x1) == 0x1) ? (wrongtype) : (empty),
                                                ((val & 0x2) == 0x2) ? (wrongappl) : (empty),
                                                ((val & 0x4) == 0x4) ? (wrongprot) : (empty)
                    );

                    m_dnginfo.baseInfo.finalMessage.append(str);
                    n++;

                }
            }

        } break;

        case eoerror_value_CFG_skin_ok:
        {
            uint16_t maskcan1 = (m_dnginfo.param64 & 0x0000ffff00000000) >> 32;
            uint16_t maskcan2 = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint64_t reqfw = (m_dnginfo.param64 & 0x000000000000ffff);
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 8) & 0xff;
            appl.minor = reqfw & 0xff;
            uint16_t numOfpatches = m_dnginfo.param16;

            snprintf(str, sizeof(str), "%s on %d skin patches for boards on (can1map, can2map) = (0x%.4x, 0x%.4x) with target can protocol ver %d.%d and application ver %d.%d",
                m_dnginfo.baseMessage.c_str(),
                numOfpatches,
                maskcan1, maskcan2,
                prot.major, prot.minor,
                appl.major, appl.minor
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_skin_failed_toomanyboards:
        case eoerror_value_CFG_inertials_failed_toomanyboards:
        case eoerror_value_CFG_inertials3_failed_toomanyboards:
        case eoerror_value_CFG_temperatures_failed_toomanyboards:
        {
            uint8_t numOfReqBoards = (m_dnginfo.param16 & 0xff00) >> 8;
            uint8_t numOfMaxBoards = m_dnginfo.param16 & 0x00ff;

            uint16_t maskcan1 = (m_dnginfo.param64 & 0x000000000000ffff);
            uint16_t maskcan2 = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;

            snprintf(str, sizeof(str), " %s for %d boards. Limit of max number of boards is %d. Boards are on (can1map, can2map) = (0x%.4x, 0x%.4x)",
                                        m_dnginfo.baseMessage.c_str(),
                                        numOfReqBoards, numOfMaxBoards,
                                        maskcan1, maskcan2
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_skin_failed_candiscovery:
        case eoerror_value_CFG_inertials_failed_candiscovery:
        case eoerror_value_CFG_inertials3_failed_candiscovery:
        case eoerror_value_CFG_temperatures_failed_candiscovery:
        {
            uint16_t incompMaskcan2 = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            uint16_t incompMaskcan1 = (m_dnginfo.param64 & 0x0000ffff00000000) >> 32;
            uint16_t missMaskcan2 = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint16_t missMaskcan1 = (m_dnginfo.param64 & 0x000000000000ffff);
            uint16_t numOfPatches = m_dnginfo.param16;
            
            if (eoerror_value_CFG_skin_failed_candiscovery == value)
            {
                snprintf(str, sizeof(str), "%s for %d skin patches. ", m_dnginfo.baseMessage.c_str(), numOfPatches);
                m_dnginfo.baseInfo.finalMessage.append(str);
            }
            else
            {
                snprintf(str, sizeof(str), "%s. ", m_dnginfo.baseMessage.c_str());
                m_dnginfo.baseInfo.finalMessage.append(str);
            }

            snprintf(str, sizeof(str), "Missing can boards on (can1map, can2map) = (0x%.4x, 0x%.4x) and found but incompatible can boards on (can1map, can2map) = (0x%.4x, 0x%.4x)",
                missMaskcan1, missMaskcan2,
                incompMaskcan1, incompMaskcan2
            );

            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_strain_ok:
        case eoerror_value_CFG_strain_failed_candiscovery:
        {
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint64_t reqfw = (m_dnginfo.param64 & 0x000000000000ffff);
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 8) & 0xff;
            appl.minor = reqfw & 0xff;
            uint8_t strain = (m_dnginfo.param64 & 0x0000000f00000000) >> 20;
            uint8_t address = m_dnginfo.param16 & 0x00ff;
            uint8_t port = m_dnginfo.param16 >> 8;

            snprintf(str, sizeof(str), "%s for board at addr:%d and port:%d with can protocol ver %d.%d and application ver %d.%d. Strain number is:%d",
                m_dnginfo.baseMessage.c_str(),
                address, port,
                prot.major, prot.minor,
                appl.major, appl.minor,
                strain
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_mais_ok:
        case eoerror_value_CFG_mais_failed_candiscovery:
        case eoerror_value_CFG_psc_ok:
        case eoerror_value_CFG_psc_failed_candiscovery:
        case eoerror_value_CFG_pos_ok:
        case eoerror_value_CFG_pos_failed_candiscovery:
        {
            eObrd_protocolversion_t prot = {0};
            eObrd_firmwareversion_t appl = {0};
            uint64_t reqpr = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint64_t reqfw = (m_dnginfo.param64 & 0x000000000000ffff);
            prot.major = reqpr >> 8;
            prot.minor = reqpr & 0xff;
            appl.major = (reqfw >> 8) & 0xff;
            appl.minor = reqfw & 0xff;
            uint8_t address = m_dnginfo.param16 & 0x00ff;
            uint8_t port = m_dnginfo.param16 >> 8;

            snprintf(str, sizeof(str), "%s on board at addr: %d and port:%d with can protocol ver %d.%d and application ver %d.%d.",
                                        m_dnginfo.baseMessage.c_str(),
                                        address,
                                        port,
                                        prot.major, prot.minor,
                                        appl.major, appl.minor
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_mais_failed_verify_because_active:
        case eoerror_value_CFG_mc_foc_ok:
        case eoerror_value_CFG_mc_foc_failed_candiscovery_of_foc:
        case eoerror_value_CFG_mc_foc_failed_encoders_verify:
        case eoerror_value_CFG_mc_mc4_ok:
        case eoerror_value_CFG_mc_mc4_failed_candiscovery_of_mc4:
        case eoerror_value_CFG_mc_mc4_failed_mais_verify:
        case eoerror_value_CFG_mc_mc4plus_ok:
        case eoerror_value_CFG_mc_mc4plus_failed_encoders_verify:
        case eoerror_value_CFG_inertials_ok:
        case eoerror_value_CFG_comm_cannotloadaregularrop:
        case eoerror_value_CFG_mc_mc4plusmais_ok:
        case eoerror_value_CFG_mc_mc4plusmais_failed_encoders_verify:
        case eoerror_value_CFG_mc_mc4plusmais_failed_candiscovery_of_mais:
        case eoerror_value_CFG_services_not_verified_yet:
        case eoerror_value_CFG_mc_not_verified_yet:
        case eoerror_value_CFG_strain_not_verified_yet:
        case eoerror_value_CFG_mais_not_verified_yet:
        case eoerror_value_CFG_skin_not_verified_yet:
        case eoerror_value_CFG_inertials_not_verified_yet:
        case eoerror_value_CFG_inertials3_not_verified_yet:
        case eoerror_value_CFG_encoders_not_verified_yet:
        case eoerror_value_CFG_mc_using_onboard_config:
        case eoerror_value_CFG_strain_using_onboard_config:
        case eoerror_value_CFG_mais_using_onboard_config:
        case eoerror_value_CFG_inertials_using_onboard_config:
        case eoerror_value_CFG_inertials3_using_onboard_config:
        case eoerror_value_CFG_skin_using_onboard_config:
        case eoerror_value_CFG_inertials3_ok:
        case eoerror_value_CFG_temperatures_not_verified_yet:
        case eoerror_value_CFG_temperatures_ok:
        case eoerror_value_CFG_temperatures_using_onboard_config:
        case eoerror_value_CFG_psc_failed_verify_because_active:
        case eoerror_value_CFG_psc_not_verified_yet:
        case eoerror_value_CFG_psc_using_onboard_config:
        case eoerror_value_CFG_mc_mc2pluspsc_ok:
        case eoerror_value_CFG_mc_mc2pluspsc_failed_encoders_verify:
        case eoerror_value_CFG_mc_mc2pluspsc_failed_candiscovery_of_pscs:
        case eoerror_value_CFG_inertials_failed_notsupported:
        case eoerror_value_CFG_inertials3_failed_notsupported:
        case eoerror_value_CFG_temperatures_failed_notsupported:
        case eoerror_value_CFG_mais_failed_notsupported:
        case eoerror_value_CFG_strain_failed_notsupported:
        case eoerror_value_CFG_skin_failed_notsupported:
        case eoerror_value_CFG_psc_failed_notsupported:
        case eoerror_value_CFG_mc_failed_notsupported:
        case eoerror_value_CFG_encoders_failed_notsupported:
        case eoerror_value_CFG_pos_not_verified_yet:
        case eoerror_value_CFG_pos_using_onboard_config:
        case eoerror_value_CFG_pos_failed_notsupported:
        case eoerror_value_CFG_mc_mc4plusfaps_ok:
        case eoerror_value_CFG_mc_mc4plusfaps_failed_encoders_verify:
        case eoerror_value_CFG_mc_mc4plusfaps_failed_candiscovery:
        case eoerror_value_CFG_mc_mc4pluspmc_ok:
        case eoerror_value_CFG_mc_mc4pluspmc_failed_encoders_verify:
        case eoerror_value_CFG_mc_mc4pluspmc_failed_candiscovery_of_pmc:
        case eoerror_value_CFG_ft_ok:
        case eoerror_value_CFG_ft_failed_candiscovery:
        case eoerror_value_CFG_ft_not_verified_yet:
        case eoerror_value_CFG_ft_using_onboard_config:
        case eoerror_value_CFG_ft_failed_notsupported:
        case eoerror_value_CFG_ft_failed_fullscales:
        case eoerror_value_CFG_bat_ok:
        case eoerror_value_CFG_bat_failed_candiscovery:
        case eoerror_value_CFG_bat_not_verified_yet:
        case eoerror_value_CFG_bat_using_onboard_config:
        case eoerror_value_CFG_bat_failed_notsupported:
        {
            printBaseInfo();
        } break;

        // p16&0xf000: number of joint; primary encs: failure mask in p16&0x000f and errorcodes in p64&0x0000ffff; secondary encs: failure mask in p16&0x00f0 and errorcodes in p64&0xffff0000"},
        case eoerror_value_CFG_encoders_ok:
        case eoerror_value_CFG_encoders_failed_verify:
        {
            uint8_t numOfJoints = (m_dnginfo.param16 & 0xf000) >> 12;
            uint8_t failmaskenc1 = m_dnginfo.param16 & 0x000f;
            int16_t errorenc1 = m_dnginfo.param64 & 0x0000ffff;
            uint8_t failmaskenc2 = (m_dnginfo.param16 & 0x00f0) >> 4;
            int16_t errorenc2 = (m_dnginfo.param64 & 0xffff0000) >> 16;

            int16_t rawerror1 = errorenc1 & failmaskenc1;
            int16_t rawerror2 = errorenc2 & failmaskenc2;
            snprintf(str, sizeof(str), "%s", m_dnginfo.baseMessage.c_str());
            m_dnginfo.baseInfo.finalMessage.append(str);

            for(auto i=0; i < numOfJoints; i++)
            {
                // 1. check if joint ith has encoder with errors
                auto primary_enc_with_error = (failmaskenc1 & (1<<i));
                auto secondary_enc_with_error = (failmaskenc2 & (1<<i));
                auto primary_error_code = 0;
                auto secondary_error_code = 0;
                if(primary_enc_with_error) 
                {
                    primary_error_code = ( (errorenc1 & (0xf <<i)) >> 4*i);
                    m_entityNameProvider.getAxisName(i, m_dnginfo.baseInfo.axisName);
                    snprintf(str, sizeof(str), " joint %d (%s) has error on primary encoder (code=%d). ", 
                            i, m_dnginfo.baseInfo.axisName.c_str(), primary_error_code); //TODO: get a string instead of a code
                    m_dnginfo.baseInfo.finalMessage.append(str);
                }

                if(secondary_enc_with_error) 
                {
                    secondary_error_code = ( (errorenc2 & (0xf <<i)) >> 4*i);
                    m_entityNameProvider.getAxisName(i, m_dnginfo.baseInfo.axisName);
                    snprintf(str, sizeof(str), " joint %d (%s) has error on secodary encoder (code=%d)", 
                            i, m_dnginfo.baseInfo.axisName.c_str(), secondary_error_code); //TODO: get a string instead of a code
                    m_dnginfo.baseInfo.finalMessage.append(str);
                }


            }

        } break;

        case eoerror_value_CFG_inertials_failed_unsupportedsensor:
        case eoerror_value_CFG_inertials3_failed_unsupportedsensor:
        {
            int16_t unsuppsens = m_dnginfo.param16;

            snprintf(str, sizeof(str), "%s. Number of unsupported sens is %d", 
                m_dnginfo.baseMessage.c_str(),
                unsuppsens
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_inertials_changed_requestedrate:
        case eoerror_value_CFG_inertials3_changed_requestedrate:
        case eoerror_value_CFG_temperatures_changed_requestedrate:
        case eoerror_value_CFG_psc_changed_requestedrate:
        case eoerror_value_CFG_pos_changed_requestedrate:
        {
            uint8_t reqrate = (m_dnginfo.param16 & 0xff00) >> 8;
            uint8_t assrate = m_dnginfo.param16 & 0x00ff;

            snprintf(str, sizeof(str), "%s. Requested rate %u and Assigned rate %u", 
                m_dnginfo.baseMessage.c_str(),
                reqrate,
                assrate
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_CFG_inertials3_failed_generic:
        case eoerror_value_CFG_temperatures_failed_generic:
        {
            uint8_t numOfSens = m_dnginfo.param64;

            snprintf(str, sizeof(str), "%s for %d sensors", 
                m_dnginfo.baseMessage.c_str(),
                numOfSens
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;
        
        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_Config error value.");


        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch

    
    
}




/**************************************************************************************************************************/
/******************************************   MotionControlParser   ***************************************************/
/**************************************************************************************************************************/

MotionControlParser::MotionControlParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void MotionControlParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {
        
        case eoerror_value_MC_motor_external_fault:
        case eoerror_value_MC_motor_qencoder_phase_disappeared:
        {
            snprintf(str, sizeof(str), " %s", m_dnginfo.baseMessage.c_str());
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_MC_motor_overcurrent:
        case eoerror_value_MC_motor_i2t_limit:
        case eoerror_value_MC_motor_hallsensors:
        case eoerror_value_MC_motor_can_invalid_prot:
        case eoerror_value_MC_motor_can_generic:
        case eoerror_value_MC_motor_can_no_answer:
        case eoerror_value_MC_axis_torque_sens:
        case eoerror_value_MC_joint_hard_limit:
        {
            uint8_t joint_num = m_dnginfo.param16 & 0x00ff;
            m_entityNameProvider.getAxisName(joint_num, m_dnginfo.baseInfo.axisName);

            snprintf(str, sizeof(str), " %s (Joint=%s (NIB=%d))", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.axisName.c_str(), joint_num);
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_MC_aea_abs_enc_invalid:
        case eoerror_value_MC_aea_abs_enc_spikes:
        case eoerror_value_MC_aea_abs_enc_timeout:
        {
            uint8_t joint_num = m_dnginfo.param16 & 0x00ff;
            uint8_t enc_port = (m_dnginfo.param16 & 0xff00)>>8;
            m_entityNameProvider.getAxisName(joint_num, m_dnginfo.baseInfo.axisName);

            snprintf(str, sizeof(str), " %s (Joint=%s (NIB=%d), encoderPort=%d)",
                                        m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.axisName.c_str(), joint_num, enc_port
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;


        case eoerror_value_MC_motor_qencoder_dirty:
        case eoerror_value_MC_motor_qencoder_phase: //TBD: check encoder raw value
        {
            uint16_t joint_num = m_dnginfo.param16;
            uint16_t enc_raw_value = m_dnginfo.param64 & 0xffff;
            m_entityNameProvider.getAxisName(joint_num, m_dnginfo.baseInfo.axisName);

            snprintf(str, sizeof(str), " %s (Joint=%s (NIB=%d), Raw_quad_encoder_value=%d)",
                                        m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.axisName.c_str(), joint_num, enc_raw_value
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_MC_generic_error: //TBD Check print
        {
            snprintf(str, sizeof(str), " %s (Error is %lx)", m_dnginfo.baseMessage.c_str(), m_dnginfo.param64);
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;
        
        case eoerror_value_MC_motor_wrong_state: //TBD: check states
        {
            uint16_t joint_num = m_dnginfo.param16;
            uint16_t req_state = (m_dnginfo.param64 & 0xf0)>>4;
            uint16_t cur_state = m_dnginfo.param64 & 0x0f;

            m_entityNameProvider.getAxisName(joint_num, m_dnginfo.baseInfo.axisName);


            snprintf(str, sizeof(str), " %s Joint=%s (NIB=%d). The requested state is %d, but the current is %d)",
                                        m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.axisName.c_str(), joint_num, req_state, cur_state
                                        );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_MotionControl error value.");


        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch

    
    
}




/**************************************************************************************************************************/
/******************************************   SkinParser   ***************************************************/
/**************************************************************************************************************************/



SkinParser::SkinParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void SkinParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch (value)
    {
        case eoerror_value_SK_unspecified:
        case eoerror_value_SK_obsoletecommand:
        {
            printBaseInfo();
        } break;

        case eoerror_value_SK_arrayofcandataoverflow:
        {
            uint8_t frame_id = m_dnginfo.param16 & 0x00ff;
            uint8_t frame_size = (m_dnginfo.param16 & 0xf000) >> 12;
            uint64_t frame_data = m_dnginfo.param64;

            snprintf(str, sizeof(str), " %s. Frame.ID=%d, Frame.Size=%d Frame.Data=0x%lx",
                m_dnginfo.baseMessage.c_str(), frame_id, frame_size, frame_data
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;
        
        case eoerror_value_SK_onoroff:
        {
            std::string emsboardstate = "unknown";
            switch (m_dnginfo.param16)
            {
                case 0: emsboardstate = "OFF"; break;
                case 1: emsboardstate = "ON"; break;
            };
            
            snprintf(str, sizeof(str), " %s %s", m_dnginfo.baseMessage.c_str(), emsboardstate.c_str());
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_SK_unexpecteddata:
        {
            std::string emsboardstate = "unknown";
            switch (m_dnginfo.param16)
            {
                case 0: emsboardstate = "CFG"; break;
                case 1: emsboardstate = "RUN"; break;
            }

            snprintf(str, sizeof(str), " %s %s", m_dnginfo.baseMessage.c_str(), emsboardstate.c_str());
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognized eoerror_category_Skin error value");
        } break;
        
        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();
        
        } break;
    }
}

/**************************************************************************************************************************/
/******************************************   HwErrorParser   ***************************************************/
/**************************************************************************************************************************/



HwErrorParser::HwErrorParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void HwErrorParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {
        
        case eoerror_value_HW_strain_saturation:
        {
            uint16_t channel = m_dnginfo.param16;
            uint32_t lower_saturation_counts = m_dnginfo.param64 & 0xffffffff;
            uint32_t upper_saturation_counts = (m_dnginfo.param64 & 0xffffffff00000000)>>32;
            snprintf(str, sizeof(str), " %s %d is the channel involved. In the last second, the lower saturation counts is %d and the upper one is %d", 
                                        m_dnginfo.baseMessage.c_str(),
                                        channel,
                                        lower_saturation_counts,
                                        upper_saturation_counts);
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_HW_encoder_invalid_value:
        case eoerror_value_HW_encoder_close_to_limits:
        case eoerror_value_HW_encoder_crc: 
        case eoerror_value_HW_encoder_not_connected:
        {
            printBaseInfo();
        } break;

    
        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_HardWare error value.");


        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch

    
    
}



/**************************************************************************************************************************/
/******************************************   SysErrorParser   ***************************************************/
/**************************************************************************************************************************/



SysParser::SysParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void SysParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {
        
        case eoerror_value_SYS_runninghappily:
        {
            std::string appstate = "unknown";
            switch(m_dnginfo.param16&0x000f)
            {
                case 0: appstate="just restarted"; break;
                case 1: appstate="idle"; break;
                case 2: appstate="running"; break;
            };
            snprintf(str, sizeof(str), " %s Application state is %s.", m_dnginfo.baseMessage.c_str(), appstate.c_str());
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_ctrloop_execoverflowRX:
        {
            // TODO: check if time to show is TX. Shouldn't be RX in this case?
            snprintf(str, sizeof(str), " %s RX execution time %d[usec]. Latest previous execution times of TX, RX, DO, TX %ld[usec]", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16, m_dnginfo.param64);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_ctrloop_execoverflowDO:
        {
            snprintf(str, sizeof(str), " %s DO execution time %d[usec]. Latest previous execution times of RX, DO, TX, RX %ld[usec]", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16, m_dnginfo.param64);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_ctrloop_execoverflowTX:
        {
            // TODO: ask suggested the usec time (last parsed value is just one value, not 4) --> do I need to mask it or printing just par64 as int does the job?
            snprintf(str, sizeof(str), " %s TX execution time %d[usec]. Latest previous execution times of TX, RX, DO %ld[usec]",
                    m_dnginfo.baseMessage.c_str(), m_dnginfo.param16, m_dnginfo.param64
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_ropparsingerror:
        {
            snprintf(str, sizeof(str), " %s Error code is  %d (eOparserResult_t).", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_halerror:
        {
            snprintf(str, sizeof(str), " %s HAL error code is  %d.", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_osalerror:
        {
            snprintf(str, sizeof(str), " %s OSAL error code is  %d.", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_ipalerror:
        {
            snprintf(str, sizeof(str), " %s IPAL error code is  %d.", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_dispatcherfifooverflow:
        {
            snprintf(str, sizeof(str), " %s Number of lost items is  %d.", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_txfifooverflow:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s Frame.ID=%d, Frame.Size=%d Frame.Data=0x.%lx", 
            m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), (m_dnginfo.param16&& 0x0fff), ((m_dnginfo.param16&& 0xf000)>>12), m_dnginfo.param64 );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_txbusfailure:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s. Size of fifo is %d", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), ((m_dnginfo.param16&& 0xff00) >>8));
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_formingfailure:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s. Message class is %d. Message cmd is %d", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), ((m_dnginfo.param16&& 0xff00) >>8), (m_dnginfo.param16&& 0x00ff));
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_parsingfailure:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s. Frame.size=%d. Frame.Id=%d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), ((m_dnginfo.param16&& 0xf000) >>12), (m_dnginfo.param16&& 0x0fff));
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case  eoerror_value_SYS_canservices_genericerror:
        {
            snprintf(str, sizeof(str), " %s error code is %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_rxphaseaverage:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;
       
        case eoerror_value_SYS_ctrloop_dophaseaverage:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_txphaseaverage:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_rxphasemax:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_dophasemax:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;
        case eoerror_value_SYS_ctrloop_txphasemax:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_rxphasemin:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_dophasemin:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_ctrloop_txphasemin:
        {
            snprintf(str, sizeof(str), " %s %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_proxy_forward_fails:
        {
            snprintf(str, sizeof(str), " %s. ROP.sign=%d, ROP.id=%d. Proxy list capacity is %d, size is %d ", 
                     m_dnginfo.baseMessage.c_str(), 
                     (int32_t)((m_dnginfo.param64&0xffffffff00000000)>>32),
                     (int32_t)(m_dnginfo.param64&0x00000000ffffffff), 
                     ((m_dnginfo.param16&0xff00)>>8), (m_dnginfo.param16&0x00ff));
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_proxy_ropdes_notfound:
        {
            snprintf(str, sizeof(str), " %s ROP.id=%d ", m_dnginfo.baseMessage.c_str(), (int32_t)(m_dnginfo.param64&0x00000000ffffffff));
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_canservices_canprint:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s Frame.Size=%d Frame.Data=0x.%lx", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), m_dnginfo.param16, m_dnginfo.param64 );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_rxmaisbug:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s Frame.Size=%d Frame.Data=0x.%lx", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), m_dnginfo.param16, m_dnginfo.param64 );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_rxfromwrongboard:
        {
            snprintf(str, sizeof(str), " %s CanPort=%s Frame.Size=%d Frame.Data=0x.%lx", m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), m_dnginfo.param16, m_dnginfo.param64 );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_transceiver_rxseqnumber_error:
        {
            int16_t receivedNum = m_dnginfo.param64+ m_dnginfo.param16;
            snprintf(str, sizeof(str), " %s Expected number is %ld, received number is %d ", m_dnginfo.baseMessage.c_str(), m_dnginfo.param64, receivedNum);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_transceiver_rxseqnumber_restarted:
        {
            snprintf(str, sizeof(str), " %s Expected number is %ld", m_dnginfo.baseMessage.c_str(), m_dnginfo.param64);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_board_detected:
        {
            //in param64 the fw copies the struct eObrd_typeandversions_t defined in EoBoards.h in icub=firmware repo
            /**
            typedef struct {
                eOenum08_t                  boardtype;
                uint8_t                     firmwarebuildnumber;
                eObrd_version_t             firmwareversion;
                eObrd_version_t             protocolversion;   
            } eObrd_typeandversions_t;      EO_VERIFYsizeof(eObrd_typeandversions_t, 6);


            typedef struct                  // size is: 1+1+0 = 2
            {
                uint8_t                     major;
                uint8_t                     minor;    
            } eObrd_version_t;

             */
            //TODO:
            //checking the fw it seems this error code is no longer used.
            //So I cannot retrieve the board type. 
            //For now I leave the code. When I'm sure that it is old, I'll remove it
            int fw_build =    (m_dnginfo.param64 & 0x00000000000000ff);
            int fw_major =    (m_dnginfo.param64 & 0x000000000000ff00) >> 8;
            int fw_minor =    (m_dnginfo.param64 & 0x0000000000ff0000) >> 16;
            int proto_major = (m_dnginfo.param64 & 0x00000000ff000000) >> 24;
            int proto_minor = (m_dnginfo.param64 & 0x000000ff00000000) >> 32;

            //used in comm-v1 protocol
            // eObrd_typeandversions_t *brd_info_ptr = (eObrd_typeandversions_t *)&m_dnginfo.param64;
            // int fw_build =   brd_info_ptr->firmwarebuildnumber;
            // int fw_major =   brd_info_ptr->firmwareversion.major;
            // int fw_minor =   brd_info_ptr->firmwareversion.minor;
            // int proto_major =brd_info_ptr->protocolversion.major;
            // int proto_minor =brd_info_ptr->protocolversion.minor;

            // eObrd_type_t  general_brd_type = eoboards_cantype2type((eObrd_cantype_t )brd_info_ptr->boardtype);

            // std::string board_type_str = eoboards_type2string(general_brd_type);
            
            snprintf(str, sizeof(str), " %s on CAN port=%s with address %d.  Fw ver is %d.%d.%d. Proto ver is %d.%d", 
                                         m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), m_dnginfo.baseInfo.sourceCANBoardAddr,
                                         fw_build, fw_major, fw_minor, proto_major, proto_minor );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_board_wrongprotversion:
        {
            //in param64 the fw copies the struct eObrd_typeandversions_t defined in EoBoards.h in icub=firmware repo
            /**
            typedef struct {
                eOenum08_t                  boardtype;
                uint8_t                     firmwarebuildnumber;
                eObrd_version_t             firmwareversion;
                eObrd_version_t             protocolversion;   
            } eObrd_typeandversions_t;      EO_VERIFYsizeof(eObrd_typeandversions_t, 6);


            typedef struct                  // size is: 1+1+0 = 2
            {
                uint8_t                     major;
                uint8_t                     minor;    
            } eObrd_version_t;

             */
            //as above 
            int fw_build =    (m_dnginfo.param64 & 0x00000000000000ff);
            int fw_major =    (m_dnginfo.param64 & 0x000000000000ff00) >> 8;
            int fw_minor =    (m_dnginfo.param64 & 0x0000000000ff0000) >> 16;
            int proto_major = (m_dnginfo.param64 & 0x00000000ff000000) >> 24;
            int proto_minor = (m_dnginfo.param64 & 0x000000ff00000000) >> 32;
            int req_proto_major = (m_dnginfo.param16 & 0xff00) >>8;
            int req_proto_minor = (m_dnginfo.param16 & 0x00ff);


            snprintf(str, sizeof(str), " %s on CAN port=%s with address %d. Fw ver is %d.%d.%d. Proto ver is %d.%d. Required Version is %d.%d", 
                                         m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), m_dnginfo.baseInfo.sourceCANBoardAddr, 
                                         fw_build, fw_major, fw_minor, proto_major, proto_minor, req_proto_major, req_proto_minor );
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_SYS_canservices_board_notfound:
        {
            eObrd_type_t  general_brd_type = eoboards_cantype2type((eObrd_cantype_t)m_dnginfo.param16);
            snprintf(str, sizeof(str), " %s The board is on CAN port=%s with address %d. Board type is %s.", 
                                         m_dnginfo.baseMessage.c_str(), m_dnginfo.baseInfo.sourceCANPortStr.c_str(), 
                                         m_dnginfo.baseInfo.sourceCANBoardAddr, eoboards_type2string(general_brd_type));
            m_dnginfo.baseInfo.finalMessage.append(str);

        }break;

        case eoerror_value_SYS_canservices_boards_lostcontact: //TODO: DONE (see eomn_servicecategory2string) make a specific message.need some translation from enum to string
        case eoerror_value_SYS_canservices_boards_retrievedcontact://TODO: DONE  (see eomn_servicecategory2string) make a specific message.need some translation from enum to string
        {
            eOmn_serv_category_t serv_category = (eOmn_serv_category_t)m_dnginfo.param16;
            uint16_t lostMaskcan2 = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint16_t lostMaskcan1 = (m_dnginfo.param64 & 0x000000000000ffff);
            char lostCanBoards1[64] = {0};
            char lostCanBoards2[64] = {0};

            for(int i=1; i<15; i++)
            {
                if ( (lostMaskcan1 & (1<<i)) == (1<<i))
                {
                    strcat(lostCanBoards1,  std::to_string(i).c_str());
                    strcat(lostCanBoards1, " ");
                }

                if ( (lostMaskcan2 & (1<<i)) == (1<<i))
                {
                    strcat(lostCanBoards2,  std::to_string(i).c_str());
                    strcat(lostCanBoards2, " ");
                }
            }


            snprintf(str, sizeof(str), "%s Type of service category is %s. Lost can boards on (can1map, can2map) = ([ %s ], [ %s ] )",
                m_dnginfo.baseMessage.c_str(),
                eomn_servicecategory2string(serv_category),
                lostCanBoards1, lostCanBoards2
            );

            m_dnginfo.baseInfo.finalMessage.append(str);

        } break;

/**
 * {eoerror_value_SYS_canservices_monitor_regularcontact, "SYS: a service has verified that the TX of its CAN boards is regular. In sourceaddress the eOmn_serv_category_t, in par64 LS 32 bits the bit mask of boards (CAN1 in MS 16 bits and CAN2 in LS 16 bits)"},
    {eoerror_value_SYS_canservices_monitor_lostcontact,  "SYS: a service has detected that some CAN boards have stopped transmission. In sourceaddress the eOmn_serv_category_t, in par64 LS 32 bits the bit mask of lost board (CAN1 in MS 16 bits and CAN2 in LS 16 bits), in in par64 MS 32 bits the time in ms since last contact"},
    {eoerror_value_SYS_canservices_monitor_stillnocontact,  "SYS: a service has detected that some CAN boards are still not transmitting. In sourceaddress the eOmn_serv_category_t, in par64 LS 32 bits the bit mask of lost board (CAN1 in MS 16 bits and CAN2 in LS 16 bits), in in par64 MS 32 bits the total disappearence time in ms"},
    {eoerror_value_SYS_canservices_monitor_retrievedcontact, "SYS: a service has recovered all CAN boards that were not transmitting. In sourceaddress the eOmn_serv_category_t)"}

*/
        case eoerror_value_SYS_canservices_monitor_retrievedcontact: //TODO: make a specific message.need some translation from enum to string
        {
            eOmn_serv_category_t serv_category = (eOmn_serv_category_t)m_dnginfo.baseInfo.sourceCANBoardAddr;
            snprintf(str, sizeof(str), "%s Type of service category is %s.",
                m_dnginfo.baseMessage.c_str(),
                eomn_servicecategory2string(serv_category)
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_SYS_canservices_monitor_regularcontact: //TODO: make a specific message.need some translation from enum to string
        {
            eOmn_serv_category_t serv_category = (eOmn_serv_category_t)m_dnginfo.baseInfo.sourceCANBoardAddr;
            uint16_t foundMaskcan2 = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint16_t foundMaskcan1 = (m_dnginfo.param64 & 0x000000000000ffff);
            char foundCanBoards1[64] = {0};
            char foundCanBoards2[64] = {0};

            for(int i=1; i<15; i++)
            {
                if ( (foundMaskcan1 & (1<<i)) == (1<<i))
                {
                    strcat(foundCanBoards1,  std::to_string(i).c_str());
                    strcat(foundCanBoards1, " ");
                }

                if ( (foundMaskcan2 & (1<<i)) == (1<<i))
                {
                    strcat(foundCanBoards1,  std::to_string(i).c_str());
                    strcat(foundCanBoards2, " ");
                }
            }

            snprintf(str, sizeof(str), "%s Type of service category is %s. CAN boards are on (can1map, can2map) = ([ %s ], [ %s ])",
                m_dnginfo.baseMessage.c_str(),
                eomn_servicecategory2string(serv_category),
                foundCanBoards1,
                foundCanBoards2
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_SYS_canservices_monitor_lostcontact: //TODO: make a specific message.need some translation from enum to string
        {
            eOmn_serv_category_t serv_category = (eOmn_serv_category_t)m_dnginfo.baseInfo.sourceCANBoardAddr;
            uint16_t lostMaskcan2 = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint16_t lostMaskcan1 = (m_dnginfo.param64 & 0x000000000000ffff);
            uint64_t timeLastContact = (m_dnginfo.param64 & 0xffff000000000000) >> 48;
            char lostCanBoards1[64] = {0};
            char lostCanBoards2[64] = {0};

            for(int i=1; i<15; i++)
            {
                if ( (lostMaskcan1 & (1<<i)) == (1<<i))
                {
                    strcat(lostCanBoards1, std::to_string(i).c_str());
                    strcat(lostCanBoards1, " ");
                }

                if ( (lostMaskcan2 & (1<<i)) == (1<<i))
                {
                    strcat(lostCanBoards2, std::to_string(i).c_str());
                    strcat(lostCanBoards2, " ");
                }
            }

            snprintf(str, sizeof(str), "%s Type of service category is %s. Lost CAN boards are on (can1map, can2map) = ([ %s ], [ %s ]). Time since last contact: %ld [ms]",
                m_dnginfo.baseMessage.c_str(),
                eomn_servicecategory2string(serv_category),
                lostCanBoards1,
                lostCanBoards2,
                timeLastContact
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_SYS_canservices_monitor_stillnocontact://TODO: make a specific message.need some translation from enum to string
        {
            eOmn_serv_category_t serv_category = (eOmn_serv_category_t)m_dnginfo.baseInfo.sourceCANBoardAddr;
            uint16_t lostMaskcan2 = (m_dnginfo.param64 & 0x00000000ffff0000) >> 16;
            uint16_t lostMaskcan1 = (m_dnginfo.param64 & 0x000000000000ffff);
            uint64_t totDisappTime = (m_dnginfo.param64 & 0xffff000000000000) >> 48;

            char lostCanBoards1[64] = {0};
            char lostCanBoards2[64] = {0};

            for(int i=1; i<15; i++)
            {
                if ( (lostMaskcan1 & (1<<i)) == (1<<i))
                {
                    strcat(lostCanBoards1, std::to_string(i).c_str());
                    strcat(lostCanBoards1, " ");
                }

                if ( (lostMaskcan2 & (1<<i)) == (1<<i))
                {
                    strcat(lostCanBoards2, std::to_string(i).c_str());
                    strcat(lostCanBoards2, " ");
                }
            }

            snprintf(str, sizeof(str), "%s Type of service category is %s. Lost CAN boards are on (can1map, can2map) = ([ %s ] , [ %s ]). Total disappearance time: %ld [ms]",
                m_dnginfo.baseMessage.c_str(),
                eomn_servicecategory2string(serv_category),
                lostCanBoards1,
                lostCanBoards2,
                totDisappTime
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;


        case eoerror_value_SYS_unspecified:            
        case eoerror_value_SYS_tobedecided:            
        case eoerror_value_SYS_memory_zerorequested:   
        case eoerror_value_SYS_memory_notinitialised:  
        case eoerror_value_SYS_memory_missing:         
        case eoerror_value_SYS_mutex_timeout:          
        case eoerror_value_SYS_wrongparam:             
        case eoerror_value_SYS_wrongusage:             
        case eoerror_value_SYS_runtimeerror:
        case eoerror_value_SYS_runninginfatalerrorstate:
        case eoerror_value_SYS_udptxfailure:
        case eoerror_value_SYS_configurator_udptxfailure:
        case eoerror_value_SYS_runner_udptxfailure:
        case eoerror_value_SYS_runner_transceivererror:
        case eoerror_value_SYS_canservices_rxfifooverflow:
        case eoerror_value_SYS_proxy_forward_ok:
        case eoerror_value_SYS_proxy_forward_callback_fails:
        case eoerror_value_SYS_proxy_reply_ok:
        case eoerror_value_SYS_proxy_reply_fails:
        case eoerror_value_SYS_canservices_boards_missing:
        case eoerror_value_SYS_canservices_boards_searched:
        case eoerror_value_SYS_canservices_boards_found:
        case eoerror_value_SYS_transceiver_rxinvalidframe_error:
        {
            printBaseInfo();
        } break;

    
        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_HardWare error value.");


        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch

    
    
}






/**************************************************************************************************************************/
/******************************************   EthMonitorParser   *****************************************/
/**************************************************************************************************************************/



EthMonitorParser::EthMonitorParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void EthMonitorParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {
        
        case eoerror_value_ETHMON_link_goes_up:
        case eoerror_value_ETHMON_link_goes_down:
        case eoerror_value_ETHMON_error_rxcrc:
        {
            std::string appstate = "unknown";
            switch(m_dnginfo.param64&0xff00000000000000)
            {
                case 0: appstate="N/A"; break;
                case 1: appstate="idle"; break;
                case 3: appstate="running"; break;
            };

            std::string ethport =  "unknown";
            switch(m_dnginfo.param16)
            {
                case 0: ethport="ETH input (P2/P13/J4)"; break;
                case 1: ethport="ETH output (P3/P12/J5)"; break;
                case 2: ethport="internal"; break;
            };
            if(eoerror_value_ETHMON_error_rxcrc == value)
                snprintf(str, sizeof(str), " %s in port %s. Application state is %s. Number of erros is %ld", 
                        m_dnginfo.baseMessage.c_str(), ethport.c_str(), appstate.c_str(), (m_dnginfo.param64&0xffffffff));
            else
                snprintf(str, sizeof(str), " %s in port %s. Application state is %s.", m_dnginfo.baseMessage.c_str(), ethport.c_str(), appstate.c_str());
            
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;

        case eoerror_value_ETHMON_txseqnumbermissing:
        {
            snprintf(str, sizeof(str), " %s w/ expected sequence %ld and number of detected %d", m_dnginfo.baseMessage.c_str(), m_dnginfo.param64, m_dnginfo.param16);
            m_dnginfo.baseInfo.finalMessage.append(str);
        }break;
        

        case eoerror_value_ETHMON_juststarted:
        case eoerror_value_ETHMON_justverified:
        {
            printBaseInfo();
        } break;

        
        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_HardWare error value.");
            

        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch
}


/**************************************************************************************************************************/
/******************************************   InertialSensorParser   ********************************************/
/**************************************************************************************************************************/

InertialSensorParser::InertialSensorParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}

void InertialSensorParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {
        
        case eoerror_value_IS_arrayofinertialdataoverflow:
        {
            
            uint8_t frame_id = m_dnginfo.param16 & 0x00ff;
            uint8_t frame_size = (m_dnginfo.param16 & 0xf000) >> 12;
            uint64_t frame_data = m_dnginfo.param64;

            snprintf(str, sizeof(str), " %s. Frame.ID=%d, Frame.Size=%d Frame.Data=0x%lx",
                m_dnginfo.baseMessage.c_str(), frame_id, frame_size, frame_data
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_IS_unknownsensor:
        {
            printBaseInfo();
        } break;

    
        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_HardWare error value.");


        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch
}

/**************************************************************************************************************************/
/******************************************   AnalogSensorParser   **********************************************/
/**************************************************************************************************************************/

AnalogSensorParser::AnalogSensorParser(AuxEmbeddedInfo &dnginfo, EntityNameProvider &entityNameProvider):DefaultParser(dnginfo, entityNameProvider){;}
void AnalogSensorParser::parseInfo()
{
    char str[512] = {0};
    eOerror_value_t value = eoerror_code2value(m_dnginfo.errorCode);
    m_dnginfo.baseInfo.finalMessage.clear();

    switch(value)
    {
        
        case eoerror_value_AS_arrayoftemperaturedataoverflow:
        {
            
            uint8_t frame_id = m_dnginfo.param16 & 0x00ff;
            uint8_t frame_size = (m_dnginfo.param16 & 0xf000) >> 12;
            uint64_t frame_data = m_dnginfo.param64;

            snprintf(str, sizeof(str), " %s. Frame.ID=%d, Frame.Size=%d Frame.Data=0x%lx",
                m_dnginfo.baseMessage.c_str(), frame_id, frame_size, frame_data
            );
            m_dnginfo.baseInfo.finalMessage.append(str);
        } break;

        case eoerror_value_AS_unknownsensor:
        {
            printBaseInfo();
        } break;

    
        case EOERROR_VALUE_DUMMY:
        {
            m_dnginfo.baseInfo.finalMessage.append(": unrecognised eoerror_category_HardWare error value.");


        } break;

        default:
        {
            Diagnostic::LowLevel::DefaultParser::parseInfo();

        }

    }//end switch
}

