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
    explicit UTB_State(const AP_AHRS_View &view) : _view(&view) {}
#if AP_UTB_ATTITUDE_SHADOW_AVAILABLE
    // Snapshot-backed state has no AP object dependency.
    UTB_State() = default;
    void shadow_sample(const Quaternion &attitude, const Vector3f &gyro, float yaw)
    {
        _attitude = attitude; _gyro = gyro; _yaw = yaw;
    }
#endif
    Vector3f get_gyro_latest() const
    {
        return _view != nullptr ? _view->get_gyro_latest() : _gyro;
    }
    void get_quat_body_to_ned(Quaternion &attitude) const
    {
        if (_view != nullptr) {
            _view->get_quat_body_to_ned(attitude);
        } else {
            attitude = _attitude;
        }
    }
    float cos_roll() const
    {
        return _view != nullptr ? _view->cos_roll() : cosf(_attitude.get_euler_roll());
    }
    float cos_pitch() const
    {
        return _view != nullptr ? _view->cos_pitch() : cosf(_attitude.get_euler_pitch());
    }
    float roll() const
    {
        return _view != nullptr ? _view->roll : _attitude.get_euler_roll();
    }
    float pitch() const
    {
        return _view != nullptr ? _view->pitch : _attitude.get_euler_pitch();
    }
    const Vector3f &get_accel_ef() const
    {
        return _view != nullptr ? _view->get_accel_ef() : _accel;
    }
    float yaw() const
    {
        return _view != nullptr ? _view->yaw : _yaw;
    }
    bool finite() const;
private:
    // AP owns the referenced view for the vehicle lifetime.
    const AP_AHRS_View *_view = nullptr;
    Quaternion _attitude;
    Vector3f _gyro, _accel;
    float _yaw = 0;
};
#endif
