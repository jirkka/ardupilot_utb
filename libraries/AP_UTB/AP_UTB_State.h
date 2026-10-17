//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_State.h
///  @brief      Read-only stav FRD→NED, primární gyro, health a stáří obou monotónních časů.
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
class AP_AHRS_View;

// Read-only diagnostic snapshot. Units are radians and radians/second, body FRD.
struct AP_UTB_State {
    Quaternion attitude_body_to_ned;
    Vector3f rates_rads;
    uint32_t sampled_us = 0;
    uint32_t imu_updated_us = 0;
    uint8_t primary_gyro = 255;
    bool attitude_valid = false;
    bool rates_valid = false;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Validate the component contract, including finite values and the applicable bounds.
    ///
    ///  @param[in]  now_us Monotonic validation timestamp in microseconds.
    ///  @param[in]  max_age_us Maximum accepted sample age in microseconds.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool valid(uint32_t now_us, uint32_t max_age_us).
    //--------------------------------------------------------------------------------------------------------------------
    bool valid(uint32_t now_us, uint32_t max_age_us) const;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
/// @brief UTB component; Read-only stav FRD→NED, primární gyro, health a stáří obou monotónních časů.
class AP_UTB_StateProvider
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      ~ap utb state provider.
    ///
    ///  @param[in]  None
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: virtual ~AP_UTB_StateProvider().
    //--------------------------------------------------------------------------------------------------------------------
    virtual ~AP_UTB_StateProvider() = default;
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Read AP attitude, gyro health and timestamps into the supplied state snapshot.
    ///
    ///  @param[out] state Sampled attitude, gyro and validity metadata.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: virtual bool sample(AP_UTB_State &state).
    //--------------------------------------------------------------------------------------------------------------------
    virtual bool sample(AP_UTB_State &state) const = 0;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class AP_UTB_APStateProvider : public AP_UTB_StateProvider
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Init.
    ///
    ///  @param[in]  view View.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void init(const AP_AHRS_View &view).
    //--------------------------------------------------------------------------------------------------------------------
    void init(const AP_AHRS_View &view)
    {
        _view = &view;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Read AP attitude, gyro health and timestamps into the supplied state snapshot.
    ///
    ///  @param[out] state Sampled attitude, gyro and validity metadata.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool sample(AP_UTB_State &state).
    //--------------------------------------------------------------------------------------------------------------------
    bool sample(AP_UTB_State &state) const override;

private:
    const AP_AHRS_View *_view = nullptr;
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
