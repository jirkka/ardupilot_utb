//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_Reference.h
///  @brief      AP-derived UTB shadow reference: circular roll/pitch limit a statická ACRO rate/expo mapa.
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
#if AP_UTB_ENABLED
#include <AP_Math/AP_Math.h>
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// AP-derived UTB shadow reference: static map only, no trainer/RATE_TC.
/// @brief UTB component; AP-derived UTB shadow reference: circular roll/pitch limit a statická ACRO rate/expo
///        mapa.
class AP_UTB_Reference
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Map calibrated RC sticks through the static AP-derived ACRO rate/expo curve.
    ///
    ///  @param[in]  sticks Sticks.
    ///  @param[in]  rp_rate_deg Rp rate deg.
    ///  @param[in]  y_rate_deg Y rate deg.
    ///  @param[in]  rp_expo Rp expo.
    ///  @param[in]  y_expo Y expo.
    ///  @param[in]  target Target.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: static bool rates(const Vector3f &sticks, float
    ///              rp_rate_deg, float y_rate_deg, float rp_expo, float y_expo, Vector3f &target).
    //--------------------------------------------------------------------------------------------------------------------
    static bool rates(const Vector3f &sticks, float rp_rate_deg, float y_rate_deg,
                      float rp_expo, float y_expo, Vector3f &target);
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
