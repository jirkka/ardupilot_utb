/// @file UTB_Rate.cpp
/// @brief AP-derived flight control; independent UTB computation, source: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp.
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
#include "UTB_Rate.h"
#include "UTB_Output.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "UTB_PID.h"
#include <AP_Scheduler/AP_Scheduler.h>

// table of user settable parameters
const AP_Param::GroupInfo UTB_Rate::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(UTB_Attitude, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_RLL_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_RLL_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_RLL_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_RLL_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, UTB_Rate, UTB_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_PIT_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_PIT_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_PIT_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_PIT_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, UTB_Rate, UTB_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Corrects in proportion to the difference between the desired yaw rate vs actual yaw rate
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_YAW_PDMX
    // @DisplayName: Yaw axis rate controller PD sum maximum
    // @Description: Yaw axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_YAW_D_FF
    // @DisplayName: Yaw Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_YAW_NTF
    // @DisplayName: Yaw Target notch filter index
    // @Description: Yaw Target notch filter index
    // @Range: 1 8
    // @Units: Hz
    // @User: Advanced

    // @Param: RAT_YAW_NEF
    // @DisplayName: Yaw Error notch filter index
    // @Description: Yaw Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, UTB_Rate, UTB_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, UTB_Rate, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, UTB_Rate, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, UTB_Rate, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: THR_G_BOOST
    // @DisplayName: Throttle-gain boost
    // @Description: Throttle-gain boost ratio. A value of 0 means no boosting is applied, a value of 1 means full boosting is applied. Describes the ratio increase that is applied to angle P and PD on pitch and roll.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("THR_G_BOOST", 7, UTB_Rate, _throttle_gain_boost, 0.0f),

    AP_GROUPEND
};

UTB_Rate::UTB_Rate(UTB_State &ahrs, AP_MotorsMulticopter& motors) :
    UTB_Attitude(ahrs, motors),
    _motors_multi(motors),
    _output(motors)
{
    AP_Param::setup_object_defaults(this, var_info);

#if AP_FILTER_ENABLED
    set_notch_sample_rate(AP::scheduler().get_loop_rate_hz());
#endif
}

// Update Alt_Hold angle maximum
void UTB_Rate::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max_rad = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max_rad = _althold_lean_angle_max_rad + (_dt_s / (_dt_s + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max_rad);
}

void UTB_Rate::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _output.throttle_filter(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _output.throttle(throttle_in);
    _output.throttle_average(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void UTB_Rate::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float UTB_Rate::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle_rad);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float UTB_Rate::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors->get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_gain_boost - boost angle_p/pd each cycle on high throttle slew
void UTB_Rate::update_throttle_gain_boost()
{
    // Boost PD and Angle P on very rapid throttle changes
    if (_motors->get_throttle_slew_rate() > AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH) {
        const float pd_boost = constrain_float(_throttle_gain_boost + 1.0f, 1.0, 2.0);
        set_PD_scale_mult(Vector3f{pd_boost, pd_boost, 1.0f});

        const float angle_p_boost = constrain_float((_throttle_gain_boost + 1.0f) * (_throttle_gain_boost + 1.0f), 1.0, 4.0);
        set_angle_P_scale_mult(Vector3f{angle_p_boost, angle_p_boost, 1.0f});
    }
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void UTB_Rate::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt_s, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt_s, _throttle_rpy_mix - _throttle_rpy_mix_desired);

        // if the mix is still higher than that being used, reset immediately
        const float throttle_hover = _motors->get_throttle_hover();
        const float throttle_in = _motors->get_throttle();
        const float throttle_out = MAX(_motors->get_throttle_out(), throttle_in);
        float mix_used;
        // since throttle_out >= throttle_in at this point we don't need to check throttle_in < throttle_hover
        if (throttle_out < throttle_hover) {
            mix_used = (throttle_out - throttle_in) / (throttle_hover - throttle_in);
        } else {
            mix_used = throttle_out / throttle_hover;
        }

        _throttle_rpy_mix = MIN(_throttle_rpy_mix, MAX(mix_used, _throttle_rpy_mix_desired));
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void UTB_Rate::rate_controller_run_dt(const Vector3f& gyro_rads, float dt)
{
    _output.begin();
    // take a copy of the target so that it can't be changed from under us.
    Vector3f ang_vel_body = _ang_vel_body_rads;

    // boost angle_p/pd each cycle on high throttle slew
    update_throttle_gain_boost();

    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    ang_vel_body += _sysid_ang_vel_body_rads;

    _rate_gyro_rads = gyro_rads;
    _rate_gyro_time_us = AP_HAL::micros64();

    _output.roll(get_rate_roll_pid().update_all(ang_vel_body.x, gyro_rads.x,  dt, _motors->limit.roll, _pd_scale.x, _i_scale.x) + _actuator_sysid.x);
    _output.roll_ff(get_rate_roll_pid().get_ff());

    _output.pitch(get_rate_pitch_pid().update_all(ang_vel_body.y, gyro_rads.y,  dt, _motors->limit.pitch, _pd_scale.y, _i_scale.y) + _actuator_sysid.y);
    _output.pitch_ff(get_rate_pitch_pid().get_ff());

    _output.yaw(get_rate_yaw_pid().update_all(ang_vel_body.z, gyro_rads.z,  dt, _motors->limit.yaw, _pd_scale.z, _i_scale.z) + _actuator_sysid.z);
    _output.yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    _pd_scale_used = _pd_scale;
    _i_scale_used = _i_scale;
    _angle_P_scale_used = _angle_P_scale;
}

// run the rate controller using the configured _dt and latest gyro_rads
void UTB_Rate::rate_controller_run()
{
    Vector3f gyro_latest_rads = _ahrs.get_gyro_latest();
    rate_controller_run_dt(gyro_latest_rads, _dt_s);
}

// sanity check parameters.  should be called once before takeoff
void UTB_Rate::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

void UTB_Rate::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
    _pid_rate_yaw.set_notch_sample_rate(sample_rate);
#endif
}

#if AP_UTB_NATIVE_SHADOW_ENABLED
UTB_Rate::ShadowOutput UTB_Rate::shadow_rate_step(UTB_PID (&pid)[3], const ShadowInput &input)
{
    // Same PID, actuator and yaw FF ordering as rate_controller_run_dt above.
    // Effective scales and saturation are captured after native AP gain boost/mix.
    ShadowOutput result;
    for (uint8_t axis = 0; axis < 3; axis++) {
        result.control[axis] = pid[axis].update_all(input.target[axis], input.gyro[axis], input.dt,
                              input.limited[axis], input.pd_scale[axis], input.i_scale[axis]) + input.actuator[axis];
        result.feedforward[axis] = pid[axis].get_ff();
    }
    result.feedforward[2] *= input.yaw_ff_scale;
    return result;
}
#endif
#endif // AP_UTB_RATE_SITL_AVAILABLE
