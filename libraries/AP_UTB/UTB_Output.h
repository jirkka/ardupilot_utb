#pragma once
/// @file UTB_Output.h
/// @brief AP-derived flight control; independent UTB computation, source: AP_Motors public interface.
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
#include <AP_Motors/AP_Motors.h>
// Public functions: setters only; output() remains owned by the original Copter motor slot.
class UTB_Output
{
public:
    explicit UTB_Output(AP_Motors &motors) : _motors(motors) {}
    void begin()
    {
        _valid = true;
#if AP_UTB_RATE_SITL_AVAILABLE
        _writes = 0;
#endif
    }
    bool valid() const
    {
        return _valid;
    }
#if AP_UTB_RATE_SITL_AVAILABLE
    // Observe actual setter calls, independently of the PID telemetry snapshot.
    uint8_t writes() const
    {
        return _writes;
    }
    const Vector3f &control() const
    {
        return _control;
    }
    const Vector3f &feedforward() const
    {
        return _feedforward;
    }
#endif
    void roll(float value);
    void roll_ff(float value);
    void pitch(float value);
    void pitch_ff(float value);
    void yaw(float value);
    void yaw_ff(float value);
    void throttle_filter(float hz);
    void throttle(float value);
    void throttle_average(float value);
private:
    // The only physical-output interface; the original motor slot still owns output().
    AP_Motors &_motors;
    bool _valid = true;
#if AP_UTB_RATE_SITL_AVAILABLE
    Vector3f _control, _feedforward; // Values actually passed to the AP_Motors setters.
    uint8_t _writes = 0; // Exactly six R/P/Y plus FF calls per own rate iteration.
#endif
};
#endif
