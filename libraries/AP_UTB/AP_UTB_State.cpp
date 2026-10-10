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
#include "AP_UTB_State.h"

#if AP_UTB_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

bool AP_UTB_State::valid(uint32_t now_us, uint32_t max_age_us) const
{
    if (!attitude_valid || !rates_valid ||
        uint32_t(now_us - sampled_us) > max_age_us ||
        uint32_t(now_us - imu_updated_us) > max_age_us) {
        return false;
    }
    const Quaternion &q = attitude_body_to_ned;
    return isfinite(q.q1) && isfinite(q.q2) && isfinite(q.q3) && isfinite(q.q4) &&
           fabsf(q.length_squared() - 1.0f) < 0.01f &&
           !rates_rads.is_nan() && !rates_rads.is_inf();
}

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
