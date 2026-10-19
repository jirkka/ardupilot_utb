/// @file UTB_Output.cpp
/// @brief AP-derived flight control; independent UTB computation, source: AC_AttitudeControl_Multi motor setter order.
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
#include "UTB_Output.h"
#include <AP_InternalError/AP_InternalError.h>
void UTB_Output::roll(float value)
{
    if (!isfinite(value)) {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        value = 0;
    }
    _motors.set_roll(value);
}
void UTB_Output::roll_ff(float value)
{
    if (!isfinite(value)) {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        value = 0;
    }
    _motors.set_roll_ff(value);
}
void UTB_Output::pitch(float value)
{
    if (!isfinite(value)) {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        value = 0;
    }
    _motors.set_pitch(value);
}
void UTB_Output::pitch_ff(float value)
{
    if (!isfinite(value)) {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        value = 0;
    }
    _motors.set_pitch_ff(value);
}
void UTB_Output::yaw(float value)
{
    if (!isfinite(value)) {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        value = 0;
    }
    _motors.set_yaw(value);
}
void UTB_Output::yaw_ff(float value)
{
    if (!isfinite(value)) {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        value = 0;
    }
    _motors.set_yaw_ff(value);
}
void UTB_Output::throttle_filter(float value)
{
    if (isfinite(value)) {
        _motors.set_throttle_filter_cutoff(value);
    } else {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
    }
}
void UTB_Output::throttle(float value)
{
    if (isfinite(value)) {
        _motors.set_throttle(value);
    } else {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
    }
}
void UTB_Output::throttle_average(float value)
{
    if (isfinite(value)) {
        _motors.set_throttle_avg_max(value);
    } else {
        _valid = false;
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
    }
}

#endif
