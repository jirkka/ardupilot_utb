//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_SITLAllocation.h
///  @brief      Read-only rekonstrukce přiděleného normalized AP povelu Bᵀ(T/G), finite/linearity/sequence validace.
///  @author     Jiri Kovar
///  @version    2026-10-16
///  @date       2026-10-16
//--------------------------------------------------------------------------------------------------------------------
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
 */
#pragma once
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include <AP_Math/AP_Math.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Reconstruction of actual normalized allocation, never a physical torque estimate.
/// @brief UTB component; Read-only rekonstrukce přiděleného normalized AP povelu Bᵀ(T/G),
///        finite/linearity/sequence validace.
class AP_UTB_SITLAllocation
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    struct Capture {
        Vector3f factors[4];
        float collective[4] {};
        float thrust[4] {};
        float post_thrust[4] {};
        float gain = 0;
        Vector3f raw;
        Vector3f command;
        uint32_t output_seq = 0;
        uint32_t command_seq = 0;
        uint32_t sampled_us = 0;
        uint8_t limits = 0;
        bool factors_available = false;
        bool boost = false;
        bool full_spool = false;
        bool armed = false;
        bool pid_valid = false;
    };
    struct Result {
        Capture capture;
        Vector3f allocated;
        Vector3f residual;
        bool valid = false;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Reconstruct normalized AP allocation and reject unsupported or inconsistent captures.
    ///
    ///  @param[in]  capture Capture.
    ///  @return     Result.
    ///
    ///  @details    Public class member. Declaration/definition: static Result reconstruct(const Capture &capture).
    //--------------------------------------------------------------------------------------------------------------------
    static Result reconstruct(const Capture &capture);
};
#endif // AP_UTB_RATE_SITL_AVAILABLE

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private function prototypes
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
