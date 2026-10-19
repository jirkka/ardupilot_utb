/// @file UTB_State.cpp
/// @brief AP-derived flight control; independent UTB computation, source: AP_AHRS_View.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
/*
   Derived from ArduPilot Copter 4.7.1, reference b25e828b86f945996489b26fb0357ce05f72afcb.
   Source files and hashes are recorded in docs/UTB_PHASE_3A_CS.md.
   Copyright remains with the original ArduPilot contributors.
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version. See COPYING.txt.
   This program is distributed WITHOUT ANY WARRANTY; without even the
   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "AP_UTB_config.h"
#if AP_UTB_MATH_AVAILABLE
#include "UTB_State.h"
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
bool UTB_State::finite() const
{
    // No position/altitude envelope: check only data consumed by the flight controller.
    const Vector3f gyro = get_gyro_latest();
    Quaternion attitude;
    get_quat_body_to_ned(attitude);
    return AP::ahrs().initialised() && AP::ins().get_gyro_health(AP::ahrs().get_primary_gyro_index()) &&
           !gyro.is_nan() && !gyro.is_inf() && isfinite(attitude.length_squared()) && is_positive(attitude.length_squared());
}

#endif
