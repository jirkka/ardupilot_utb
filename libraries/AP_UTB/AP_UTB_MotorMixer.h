//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_MotorMixer.h
///  @brief      Diagnostická alokace class 1 BF_X=12/BF_X_REV=18, společné škálování R/P/Y a posun collective.
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
#include "AP_UTB_RateController.h"
#if AP_UTB_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Diagnostic allocation only. No actuator, HAL or logger dependencies.
/// @brief UTB component; Diagnostická alokace class 1 BF_X=12/BF_X_REV=18, společné škálování R/P/Y a posun
///        collective.
class AP_UTB_MotorMixer
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    struct Result {
        float thrust[4] {};
        Vector3f achieved;
        float achieved_thrust = 0;
        float scale = 0;
        float collective_shift = 0;
        uint8_t positive_limits = 0;
        uint8_t negative_limits = 0;
        uint8_t lower_mask = 0;
        uint8_t upper_mask = 0;
        bool moments_scaled = false;
        bool collective_shifted = false;
        bool valid = false;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Accept only the explicitly supported class/type combinations.
    ///
    ///  @param[in]  frame_class Configured AP frame class.
    ///  @param[in]  frame_type Configured AP frame type.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: static bool supports_frame(uint8_t frame_class, uint8_t
    ///              frame_type).
    //--------------------------------------------------------------------------------------------------------------------
    static bool supports_frame(uint8_t frame_class, uint8_t frame_type)
    {
        return frame_class == 1 && (frame_type == 12 || frame_type == 18);
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Allocate diagnostic motor values while preserving the common R/P/Y direction.
    ///
    ///  @param[in]  control Control.
    ///  @param[in]  thrust Normalized diagnostic collective request.
    ///  @param[in]  frame_class Configured AP frame class.
    ///  @param[in]  frame_type Configured AP frame type.
    ///  @return     Result.
    ///
    ///  @details    Diagnostic allocation only. These four values are never physical motor commands.
    //--------------------------------------------------------------------------------------------------------------------
    Result mix(const AP_UTB_RateController::Result &control, float thrust,
               uint8_t frame_class, uint8_t frame_type) const;
};
#endif // AP_UTB_ENABLED

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
