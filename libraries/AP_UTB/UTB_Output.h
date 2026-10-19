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
#if AP_UTB_RATE_SITL_AVAILABLE
#include <AP_Motors/AP_Motors.h>
// Public functions: setters only; output() remains owned by the original Copter motor slot.
class UTB_Output
{
public:
    explicit UTB_Output(AP_Motors &motors) : _motors(motors) {}
    void begin()
    {
        _valid = true;
    }
    bool valid() const
    {
        return _valid;
    }
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
};
#endif
