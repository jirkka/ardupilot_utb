//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_State.cpp
///  @brief      Read-only stav FRD→NED, primární gyro, health a stáří obou monotónních časů.
///  @author     Jiri Kovar
///  @version    2026-10-17
///  @date       2026-10-17
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
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "AP_UTB_State.h"

#if AP_UTB_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
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
///  @details    Public class member. Declaration/definition: bool AP_UTB_State::valid(uint32_t now_us, uint32_t max_age_us).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_State::valid(uint32_t now_us, uint32_t max_age_us) const
{
    // Orientace tělesa vůči NED, ověřovaná společně s IMU daty.
    const Quaternion &q = attitude_body_to_ned;

    if (!attitude_valid || !rates_valid ||
        uint32_t(now_us - sampled_us) > max_age_us ||
        uint32_t(now_us - imu_updated_us) > max_age_us) {
        return false;
    }
    return isfinite(q.q1) && isfinite(q.q2) && isfinite(q.q3) && isfinite(q.q4) &&
           fabsf(q.length_squared() - 1.0f) < 0.01f &&
           !rates_rads.is_nan() && !rates_rads.is_inf();
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Read AP attitude, gyro health and timestamps into the supplied state snapshot.
///
///  @param[out] state Sampled attitude, gyro and validity metadata.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB_APStateProvider::sample(AP_UTB_State &state).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_APStateProvider::sample(AP_UTB_State &state) const
{
    state = {};
    if (_view == nullptr) {
        return false;
    }
    state.sampled_us = AP_HAL::micros();
    state.imu_updated_us = AP::ins().get_last_update_usec();
    _view->get_quat_body_to_ned(state.attitude_body_to_ned);
    state.primary_gyro = AP::ahrs().get_primary_gyro_index();
    state.rates_rads = _view->get_gyro_latest();
    state.attitude_valid = AP::ahrs().initialised() && AP::ahrs().healthy();
    state.rates_valid = AP::ins().get_gyro_health(state.primary_gyro);
    // This is diagnostic validity, not permission to control a vehicle.
    return state.valid(state.sampled_us, 100000);
}
#endif // AP_UTB_ENABLED

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
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
