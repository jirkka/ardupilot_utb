#pragma once
/// @file UTB_State.h
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
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Math/AP_Math.h>

// Public functions: the native AHRS view owns all sensor filtering and estimation.
class UTB_State
{
public:
    explicit UTB_State(const AP_AHRS_View &view) : _view(view) {}
    Vector3f get_gyro_latest() const
    {
        return _view.get_gyro_latest();
    }
    void get_quat_body_to_ned(Quaternion &attitude) const
    {
        _view.get_quat_body_to_ned(attitude);
    }
    float cos_roll() const
    {
        return _view.cos_roll();
    }
    float cos_pitch() const
    {
        return _view.cos_pitch();
    }
    float roll() const
    {
        return _view.roll;
    }
    float pitch() const
    {
        return _view.pitch;
    }
    const Vector3f &get_accel_ef() const
    {
        return _view.get_accel_ef();
    }
    float yaw() const
    {
        return _view.yaw;
    }
    bool finite() const;
private:
    // AP owns the referenced view for the vehicle lifetime.
    const AP_AHRS_View &_view;
};
#endif
