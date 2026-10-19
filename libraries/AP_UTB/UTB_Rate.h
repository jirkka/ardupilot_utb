#pragma once
/// @file UTB_Rate.h
/// @brief AP-derived flight control; independent UTB computation, source: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h.
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

/// @file    UTB_Rate.h
/// @brief   ArduCopter attitude control library

#include "UTB_Attitude.h"
#include "UTB_Output.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
# define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
# define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
# define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
# define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RPY_FILT_HZ
# define AC_ATC_MULTI_RATE_RPY_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
# define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
# define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
# define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
# define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
# define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif


class UTB_Rate : public UTB_Attitude
{
public:
    UTB_Rate(UTB_State &ahrs, AP_MotorsMulticopter& motors);

    // empty destructor to suppress compiler warning
    virtual ~UTB_Rate() {}

    bool output_valid() const
    {
        return _output.valid();
    }

    // pid accessors
    UTB_PID& get_rate_roll_pid() override
    {
        return _pid_rate_roll;
    }
    UTB_PID& get_rate_pitch_pid() override
    {
        return _pid_rate_pitch;
    }
    UTB_PID& get_rate_yaw_pid() override
    {
        return _pid_rate_yaw;
    }
    const UTB_PID& get_rate_roll_pid() const override
    {
        return _pid_rate_roll;
    }
    const UTB_PID& get_rate_pitch_pid() const override
    {
        return _pid_rate_pitch;
    }
    const UTB_PID& get_rate_yaw_pid() const override
    {
        return _pid_rate_yaw;
    }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // Calculate body-frame throttle required to produce the given earth-frame throttle input (accounts for vehicle tilt)
    float get_throttle_boosted(float throttle_in);

    // Set desired throttle vs attitude mixing (actual mix is slewed toward this value over 1~2 seconds)
    // Low values favor pilot/autopilot throttle over attitude control; high values prioritize attitude control
    // Has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override
    {
        _throttle_rpy_mix_desired = _thr_mix_min;
    }
    void set_throttle_mix_man() override
    {
        _throttle_rpy_mix_desired = _thr_mix_man;
    }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override
    {
        _throttle_rpy_mix_desired = _throttle_rpy_mix = value;
    }
    float get_throttle_mix(void) const override
    {
        return _throttle_rpy_mix;
    }

    // Returns true if throttle mix is near minimum (i.e., attitude control is deprioritised)
    bool is_throttle_mix_min() const override
    {
        return (_throttle_rpy_mix < 1.25f * _thr_mix_min);
    }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run_dt(const Vector3f& gyro_rads, float dt) override;
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Boosts angle controller gains during rapid throttle changes to improve responsiveness
    // boost angle_p/pd each cycle on high throttle slew
    void update_throttle_gain_boost();

    // Slews the current throttle-to-attitude mix ratio toward the target (_throttle_rpy_mix_desired)
    void update_throttle_rpy_mix();

    // Get throttle limit based on priority of attitude vs throttle control (used for blending during low thrust)
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;

    // Roll rate PID controller (used for body-frame angular rate control)
    UTB_PID                _pid_rate_roll {
        UTB_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    // Pitch rate PID controller (used for body-frame angular rate control)
    UTB_PID                _pid_rate_pitch{
        UTB_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    // Yaw rate PID controller (used for body-frame angular rate control)
    UTB_PID                _pid_rate_yaw{
        UTB_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_YAW_P,
            .i         = AC_ATC_MULTI_RATE_YAW_I,
            .d         = AC_ATC_MULTI_RATE_YAW_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = AC_ATC_MULTI_RATE_YAW_FILT_HZ,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    // Output interface keeps NaN/Inf away from the unchanged native motor library.
    UTB_Output _output;

    // angle_p/pd boost multiplier
    AP_Float              _throttle_gain_boost;
};

#endif // AP_UTB_RATE_SITL_AVAILABLE
