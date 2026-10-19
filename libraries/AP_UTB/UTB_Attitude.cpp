/// @file UTB_Attitude.cpp
/// @brief AP-derived flight control; independent UTB computation, source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp.
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
#include "UTB_Attitude.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

// AP_Math/control.cpp: local control-law/input-shaping helpers, no hidden AP controller calls.
static float utb3_inv_sqrt_controller(float output, float p, float D_max);
static void utb3_shape_accel(float accel_desired, float& accel,
                             float jerk_max, float dt);
static void utb3_shape_angle_vel_accel(float angle_desired, float angle_vel_desired, float angle_accel_desired,
                                       float angle, float angle_vel, float& angle_accel,
                                       float angle_vel_min, float angle_vel_max, float angle_accel_max,
                                       float angle_jerk_max, float dt, bool limit_total);
static void utb3_shape_pos_vel_accel(postype_t pos_desired, float vel_desired, float accel_desired,
                                     postype_t pos, float vel, float& accel,
                                     float vel_min, float vel_max,
                                     float accel_min, float accel_max,
                                     float jerk_max, float dt, bool limit_total);
static float utb3_sqrt_controller(float error, float p, float second_ord_lim, float dt);
static Vector2f utb3_sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);
static float utb3_sqrt_controller_accel(float error, float rate_cmd, float rate_state, float p, float second_ord_lim);

static float utb3_inv_sqrt_controller(float output, float p, float D_max)
{
    // Degenerate case: second-order limit (D_max) is positive, but P gain is zero
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0 * D_max);
    }

    // Degenerate case: no D_max, but P gain is non-zero → use linear model
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }

    // Degenerate case: both gains are zero — no useful model
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0;
    }

    // Compute transition threshold between linear and sqrt regions
    const float linear_velocity = D_max / p;

    if (fabsf(output) < linear_velocity) {
        // Linear region: below transition threshold
        return output / p;
    }

    // Square-root region: above transition threshold
    const float linear_dist = D_max / sq(p);
    const float stopping_dist = (linear_dist * 0.5f) + sq(output) / (2.0 * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

static void utb3_shape_accel(float accel_desired, float& accel,
                             float jerk_max, float dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        float accel_delta = accel_desired - accel;
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
        accel += accel_delta;
    }
}

static void utb3_shape_angle_vel_accel(float angle_desired, float angle_vel_desired, float angle_accel_desired,
                                       float angle, float angle_vel, float& angle_accel,
                                       float angle_vel_min, float angle_vel_max, float angle_accel_max,
                                       float angle_jerk_max, float dt, bool limit_total)
{
    // Wrap desired angle to the nearest equivalent setpoint relative to the current angle.
    const float angle_desired_wrapped = angle + wrap_PI(angle_desired - angle);
    utb3_shape_pos_vel_accel( angle_desired_wrapped, angle_vel_desired, angle_accel_desired, angle, angle_vel, angle_accel, angle_vel_min, angle_vel_max, -angle_accel_max, angle_accel_max, angle_jerk_max, dt, limit_total);
}

static void utb3_shape_pos_vel_accel(postype_t pos_desired, float vel_desired, float accel_desired,
                                     postype_t pos, float vel, float& accel,
                                     float vel_min, float vel_max,
                                     float accel_min, float accel_max,
                                     float jerk_max, float dt, bool limit_total)
{
    // Sanity check limits and jerk_max.
    if (is_positive(vel_min) || is_negative(vel_max) || !is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Position error to be corrected.
    const float pos_error = pos_desired - pos;

    // Select utb3_sqrt_controller parameters based on error sign so the second-order limit
    // (acceleration allowance) matches the direction of motion.
    float accel_lim;
    float k_v;
    if (is_positive(pos_error)) {
        accel_lim = -accel_min;         // acceleration limit magnitude (positive)
        k_v = jerk_max / accel_lim;     // inner velocity-loop gain derived from jerk/accel limits (1/s)
    } else {
        accel_lim = accel_max;          // acceleration limit magnitude (positive)
        k_v = jerk_max / accel_lim;     // inner velocity-loop gain derived from jerk/accel limits (1/s)
    }

    // Work in the correction frame by removing the feedforward velocity.
    // vel_corr is the correction-frame velocity, so pos_error_dot = vel_desired - vel = -vel_corr.
    const float vel_corr = vel - vel_desired;

    // Velocity correction command derived from position error (second-order limited).
    float vel_corr_cmd = utb3_sqrt_controller(pos_error, k_v, accel_lim, dt);

    // Rate-of-change implied by the shaped velocity correction, using correction-frame closing rate.
    const float accel_corr_cmd = utb3_sqrt_controller_accel(pos_error, vel_corr_cmd, vel_corr, k_v, accel_lim);

    // Convert the implied rate-of-change term into an equivalent velocity correction bias.
    vel_corr_cmd += accel_corr_cmd / k_v;

    // Limit correction velocity magnitude if velocity limiting is enabled (non-zero limits).
    if (is_negative(vel_min)) {
        vel_corr_cmd = MAX(vel_corr_cmd, vel_min);
    }
    if (is_positive(vel_max)) {
        vel_corr_cmd = MIN(vel_corr_cmd, vel_max);
    }

    // Total velocity target = feedforward + correction.
    float vel_target = vel_desired + vel_corr_cmd;

    // Constrain total velocity if limiting is enabled and velocity limits are enabled (non-zero).
    if (limit_total) {
        if (is_negative(vel_min)) {
            vel_target = MAX(vel_target, vel_min);
        }
        if (is_positive(vel_max)) {
            vel_target = MIN(vel_target, vel_max);
        }
    }

    // Acceleration demand from velocity error.
    float accel_target = (vel_target - vel) * k_v;

    // Bound acceleration command (before external feedforward is added).
    accel_target = constrain_float(accel_target, accel_min, accel_max);

    // Add external acceleration feedforward.
    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled.
    if (limit_total) {
        accel_target = constrain_float(accel_target, accel_min, accel_max);
    }

    // Jerk-limit acceleration toward accel_target.
    utb3_shape_accel(accel_target, accel, jerk_max, dt);
}

static float utb3_sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // No second-order limit: use pure linear controller
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // No P gain, but with acceleration limit — use sqrt-shaped response only
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Both P and second-order limits defined — use hybrid model
        const float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            // Positive error beyond linear region — use sqrt branch
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
        } else if (error < -linear_dist) {
            // Negative error beyond linear region — use sqrt branch
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)));
        } else {
            // Inside linear region
            correction_rate = error * p;
        }
    }
    if (is_positive(dt)) {
        // Clamp to ensure we do not overshoot the error in the last time step
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

static Vector2f utb3_sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt)
{
    const float error_length = error.length();
    if (!is_positive(error_length)) {
        return Vector2f{};
    }

    const float correction_length = utb3_sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

static float utb3_sqrt_controller_accel(float error, float rate_cmd, float rate_state, float p, float second_ord_lim)
{
    // If we are moving away from the target return zero.
    if (!is_positive(rate_cmd * rate_state)) {
        return 0.0;
    }

    // If no second-order limit, controller is linear everywhere (rate_cmd ~ p*error).
    if (!is_positive(second_ord_lim)) {
        return -p * rate_state;
    }

    // If no P gain but second-order limit exists, controller is pure sqrt everywhere.
    if (!is_positive(p)) {
        if (is_zero(rate_cmd)) {
            return 0.0;
        }
        return -(second_ord_lim / fabsf(rate_cmd)) * rate_state;
    }

    // Both P and second-order limit defined — match utb3_sqrt_controller() region selection.
    const float linear_dist = second_ord_lim / sq(p);

    if (fabsf(error) <= linear_dist) {
        // Inside linear region.
        return -p * rate_state;
    }

    // Outside linear region (sqrt branch). Guard divide-by-zero on rate_cmd.
    if (is_zero(rate_cmd)) {
        return 0.0f;
    }
    return -(second_ord_lim / fabsf(rate_cmd)) * rate_state;
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// default gains for Plane
# define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.15f    // Medium
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN     5.0     // Min lean angle so that vehicle can maintain limited control
#define AC_ATTITUDE_CONTROL_AFTER_RATE_CONTROL 0
#else
// default gains for Copter and Sub
# define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.10f   // Crisp
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN     10.0   // Min lean angle so that vehicle can maintain limited control
#define AC_ATTITUDE_CONTROL_AFTER_RATE_CONTROL 1
#endif

// lean angle max (in degrees) for all vehicles
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MAX 80.0        // lean angle max in degrees

// default angle max for all vehicles
#ifndef AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT
# define AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT 30.0f   // default max lean angle in degrees
#endif

UTB_Attitude *UTB_Attitude::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo UTB_Attitude::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // 2 was SLEW_YAW (in cdeg/s) - moved to RATE_WPY_MAX in deg/s

    // 3 was for ACCEL_RP_MAX

    // 4 was ACCEL_Y_MAX (in cdeg/s/s) - moved to ACC_Y_MAX in deg/s/s

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedforward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, UTB_Attitude, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // 6 was ACCEL_R_MAX (in cdeg/s/s) - moved to ACC_R_MAX in deg/s/s

    // 7 was ACCEL_P_MAX (in cdeg/s/s) - moved to ACC_P_MAX in deg/s/s

    // IDs 8,9,10,11 RESERVED (in use on Solo)

    // @Param: ANGLE_BOOST
    // @DisplayName: Angle Boost
    // @Description: Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ANGLE_BOOST", 12, UTB_Attitude, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, UTB_Attitude, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, UTB_Attitude, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, UTB_Attitude, AC_P),

    // @Param: ANG_LIM_TC
    // @DisplayName: Angle Limit (to maintain altitude) Time Constant
    // @Description: Angle Limit (to maintain altitude) Time Constant
    // @Range: 0.5 10.0
    // @User: Advanced
    AP_GROUPINFO("ANG_LIM_TC", 16, UTB_Attitude, _angle_limit_tc, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT),

    // @Param: RATE_R_MAX
    // @DisplayName: Angular Velocity Max for Roll
    // @Description: Maximum angular velocity in roll axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_R_MAX", 17, UTB_Attitude, _ang_vel_roll_max_degs, 0.0f),

    // @Param: RATE_P_MAX
    // @DisplayName: Angular Velocity Max for Pitch
    // @Description: Maximum angular velocity in pitch axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_P_MAX", 18, UTB_Attitude, _ang_vel_pitch_max_degs, 0.0f),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angular Velocity Max for Yaw
    // @Description: Maximum angular velocity in yaw axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX", 19, UTB_Attitude, _ang_vel_yaw_max_degs, 0.0f),

    // @Param: INPUT_TC
    // @DisplayName: Attitude control input time constant
    // @Description: Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_GROUPINFO("INPUT_TC", 20, UTB_Attitude, _input_tc, AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT),

    // @Param: LAND_R_MULT
    // @DisplayName: Landed roll gain multiplier
    // @Description: Roll gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the roll axis.
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_R_MULT", 21, UTB_Attitude, _land_roll_mult, 1.0),

    // @Param: LAND_P_MULT
    // @DisplayName: Landed pitch gain multiplier
    // @Description: Pitch gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the pitch axis.
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_P_MULT", 22, UTB_Attitude, _land_pitch_mult, 1.0),

    // @Param: LAND_Y_MULT
    // @DisplayName: Landed yaw gain multiplier
    // @Description: Yaw gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the yaw axis.
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_Y_MULT", 23, UTB_Attitude, _land_yaw_mult, 1.0),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: deg
    // @Increment: 0.1
    // @Range: 10.0 80.0
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 24, UTB_Attitude, _angle_max_deg, AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT),

    // @Param: RATE_WPY_MAX
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Auto, Guided, Circle, Follow, RTL, SmartRTL, Throw and ZigZag flight modes
    // @Units: deg/s
    // @Range: 5 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RATE_WPY_MAX", 25, UTB_Attitude, _rate_wp_yaw_max_degs, AC_ATTITUDE_CONTROL_RATE_WPY_MAX_DEFAULT),

    // @Param: ACC_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: deg/s/s
    // @Range: 0 720
    // @Values: 0:Disabled, 90:VerySlow, 180:Slow, 360:Medium, 540:Fast
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("ACC_Y_MAX", 26, UTB_Attitude, _accel_yaw_max_degss, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_DEGSS),

    // @Param: ACC_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: deg/s/s
    // @Range: 0 1800
    // @Increment: 10
    // @Values: 0:Disabled, 300:VerySlow, 720:Slow, 1080:Medium, 1620:Fast
    // @User: Advanced
    AP_GROUPINFO("ACC_R_MAX", 27, UTB_Attitude, _accel_roll_max_degss, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_DEGSS),

    // @Param: ACC_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: deg/s/s
    // @Range: 0 1800
    // @Increment: 10
    // @Values: 0:Disabled, 300:VerySlow, 720:Slow, 1080:Medium, 1620:Fast
    // @User: Advanced
    AP_GROUPINFO("ACC_P_MAX", 28, UTB_Attitude, _accel_pitch_max_degss, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_DEGSS),

    AP_GROUPEND
};

constexpr Vector3f UTB_Attitude::VECTORF_111;

// get the latest gyro for the purposes of attitude control
// Counter-inuitively the lowest latency for rate control is achieved by running rate control
// *before* attitude control. This is because you want rate control to run as close as possible
// to the time that a gyro sample was read to minimise jitter and control errors. Running rate
// control after attitude control might makes sense logically, but the overhead of attitude
// control calculations (and other updates) compromises the actual rate control.
//
// In the case of running rate control in a separate thread, the ordering between rate and attitude
// updates is less important, except that gyro sample used should be the latest
//
// Currently quadplane runs rate control after attitude control, necessitating the following code
// to minimise latency.
// However this code can be removed once quadplane updates it's structure to run the rate loops before
// the Attitude controller.
const Vector3f UTB_Attitude::get_latest_gyro() const
{
#if AC_ATTITUDE_CONTROL_AFTER_RATE_CONTROL
    // rate updates happen before attitude updates so the last gyro value is the last rate gyro value
    // this also allows a separate rate thread to be the source of gyro data
    return _rate_gyro_rads;
#else
    // rate updates happen after attitude updates so the AHRS must be consulted for the last gyro value
    return _ahrs.get_gyro_latest();
#endif
}

void UTB_Attitude::reset_rate_controller_I_terms()
{
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();
}

// reset rate controller I terms smoothly to zero in 0.5 seconds
void UTB_Attitude::reset_rate_controller_I_terms_smoothly()
{
    get_rate_roll_pid().relax_integrator(0.0, _dt_s, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_pitch_pid().relax_integrator(0.0, _dt_s, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_yaw_pid().relax_integrator(0.0, _dt_s, AC_ATTITUDE_RATE_RELAX_TC);
}

// reset the rate controller target loop updates
void UTB_Attitude::rate_controller_target_reset()
{
    _sysid_ang_vel_body_rads.zero();
    _actuator_sysid.zero();
    _pd_scale = VECTORF_111;
    _i_scale = VECTORF_111;
    _angle_P_scale = VECTORF_111;
}

// Reduce attitude control gains while landed to stop ground resonance
void UTB_Attitude::landed_gain_reduction(bool landed)
{
    if (is_positive(_input_tc)) {
        // use 2.0 x tc to match the response time to 86% commanded
        const float spool_step = _dt_s / (2.0 * _input_tc);
        if (landed) {
            _landed_gain_ratio = MIN(1.0, _landed_gain_ratio + spool_step);
        } else {
            _landed_gain_ratio = MAX(0.0, _landed_gain_ratio - spool_step);
        }
    } else {
        _landed_gain_ratio = landed ? 1.0 : 0.0;
    }
    Vector3f scale_mult = VECTORF_111 * (1.0 - _landed_gain_ratio) + Vector3f{_land_roll_mult, _land_pitch_mult, _land_yaw_mult} * _landed_gain_ratio;
    set_PD_scale_mult(scale_mult);
    set_angle_P_scale_mult(scale_mult);
}

// Sets the desired roll and pitch angle inputs (radians) and a yaw rate input (radians/s).
// Used when roll/pitch stabilization is required while yaw is controlled as a rate.
// If body-frame rate feedforward shaping is enabled, shapes Euler rate/acceleration targets
// with configured limits and time constants and converts them back to body-frame targets.
// Otherwise, updates the attitude target directly and zeros rate/accel feedforward targets.
void UTB_Attitude::input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads)
{
    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // Convert body-frame angular acceleration limits (roll, pitch, yaw) into equivalent
        // Euler-angle acceleration limits for the current attitude target.
        const Vector3f euler_accel_radss = body_to_euler_limit(_attitude_target, Vector3f{get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()});
        const Vector3f euler_rate_max_rads = body_to_euler_limit(_attitude_target, Vector3f{radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), radians(_ang_vel_yaw_max_degs)});

        // Convert the body-frame angular acceleration target into an equivalent Euler-angle
        // acceleration target for the current attitude target.
        Vector3f euler_accel_target_rads;
        body_to_euler_derivative(_attitude_target, _ang_accel_target_rads, euler_accel_target_rads);

        // Shape Euler roll/pitch angle error into Euler rate/acceleration targets.
        // The shaper applies Euler rate/acceleration limits and the configured input time constant.
        attitude_command_model(wrap_PI(euler_roll_angle_rad - _euler_angle_target_rad.x), 0.0, _euler_rate_target_rads.x, euler_accel_target_rads.x, fabsf(euler_rate_max_rads.x), euler_accel_radss.x, _input_tc, _dt_s);
        attitude_command_model(wrap_PI(euler_pitch_angle_rad - _euler_angle_target_rad.y), 0.0, _euler_rate_target_rads.y, euler_accel_target_rads.y, fabsf(euler_rate_max_rads.y), euler_accel_radss.y, _input_tc, _dt_s);

        // Shape yaw rate input into Euler yaw rate/acceleration targets, applying the configured yaw rate time constant
        // and limiting Euler acceleration about the yaw axis.
        attitude_command_model(0.0, euler_yaw_rate_rads, _euler_rate_target_rads.z, euler_accel_target_rads.z, fabsf(euler_rate_max_rads.z), euler_accel_radss.z, _rate_y_tc, _dt_s);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_derivative_to_body(_attitude_target, _euler_rate_target_rads, _ang_vel_target_rads);

        // Convert euler angle derivative of desired attitude into a body-frame angular acceleration vector for feedforward
        euler_derivative_to_body(_attitude_target, euler_accel_target_rads, _ang_accel_target_rads);
    } else {
        // No shaping/feedforward: directly update roll/pitch attitude targets and integrate yaw target from yaw rate.
        _euler_angle_target_rad.x = euler_roll_angle_rad;
        _euler_angle_target_rad.y = euler_pitch_angle_rad;
        _euler_angle_target_rad.z += euler_yaw_rate_rads * _dt_s;

        // Compute quaternion target attitude
        _attitude_target.from_euler(_euler_angle_target_rad.x, _euler_angle_target_rad.y, _euler_angle_target_rad.z);

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Sets the desired roll, pitch, and yaw body-frame angular rate inputs (radians/s).
// Used by Copter's rate-only acro mode.
// Shapes the body-frame rate inputs using acceleration limits and time constants to produce
// body-frame angular velocity/acceleration targets for the rate controller.
// Attitude targets are updated from the current AHRS attitude to keep target state coherent for mode transitions.
void UTB_Attitude::input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads)
{
    // Shape body-frame rate inputs into body-frame angular velocity/acceleration targets,
    // applying acceleration limits and configured time constants.
    attitude_command_model(0.0, roll_rate_bf_rads, _ang_vel_target_rads.x, _ang_accel_target_rads.x, 0.0, get_accel_roll_max_radss(), _rate_rp_tc, _dt_s);
    attitude_command_model(0.0, pitch_rate_bf_rads, _ang_vel_target_rads.y, _ang_accel_target_rads.y, 0.0, get_accel_pitch_max_radss(), _rate_rp_tc, _dt_s);
    attitude_command_model(0.0, yaw_rate_bf_rads, _ang_vel_target_rads.z, _ang_accel_target_rads.z, 0.0, get_accel_yaw_max_radss(), _rate_y_tc, _dt_s);

    // Update attitude and Euler targets from the current vehicle attitude (used for conditioning mode transitions).
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Update body-frame angular velocity target used by the rate controller.
    _ang_vel_body_rads = _ang_vel_target_rads;
}

// Calculates the body frame angular velocities to follow the target attitude
void UTB_Attitude::update_attitude_target()
{
    // rotate target and normalize
    Quaternion attitude_target_update;
    attitude_target_update.from_axis_angle(_ang_vel_target_rads * _dt_s);
    _attitude_target *= attitude_target_update;
    _attitude_target.normalize();
}

// Calculates the body frame angular velocities to follow the target attitude
void UTB_Attitude::attitude_controller_run_quat()
{
    // This represents a quaternion rotation in NED frame to the body
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    thrust_heading_rotation_angles(_attitude_target, attitude_body, attitude_error, _thrust_angle_rad, _thrust_error_angle_rad);

    // Compute the angular velocity corrections in the body frame from the attitude error
    Vector3f ang_vel_body_rads = update_ang_vel_target_from_att_error(attitude_error);

    // ensure angular velocity does not go over configured limits
    ang_vel_limit(ang_vel_body_rads, radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), radians(_ang_vel_yaw_max_degs));

    // rotation from the target frame to the body frame
    Quaternion rotation_target_to_body = attitude_body.inverse() * _attitude_target;

    // target angle velocity vector in the body frame
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target_rads;
    Vector3f gyro = get_latest_gyro();
    // Correct the thrust vector and smoothly add feedforward and yaw input
    _feedforward_scalar = 1.0f;
    if (_thrust_error_angle_rad > AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD * 2.0f) {
        ang_vel_body_rads.z = gyro.z;
    } else if (_thrust_error_angle_rad > AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD) {
        _feedforward_scalar = (1.0f - (_thrust_error_angle_rad - AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD) / AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD);
        ang_vel_body_rads.x += ang_vel_body_feedforward.x * _feedforward_scalar;
        ang_vel_body_rads.y += ang_vel_body_feedforward.y * _feedforward_scalar;
        ang_vel_body_rads.z += ang_vel_body_feedforward.z;
        ang_vel_body_rads.z = gyro.z * (1.0 - _feedforward_scalar) + ang_vel_body_rads.z * _feedforward_scalar;
    } else {
        ang_vel_body_rads += ang_vel_body_feedforward;
    }

    // Record error to handle EKF resets
    _attitude_ang_error = attitude_body.inverse() * _attitude_target;
    // finally update the attitude target
    _ang_vel_body_rads = ang_vel_body_rads;
}

// thrust_heading_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
// The maximum error in the yaw axis is limited based on static output saturation.
void UTB_Attitude::thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const
{
    Quaternion thrust_vector_correction;
    thrust_vector_rotation_angles(attitude_target, attitude_body, thrust_vector_correction, attitude_error_rad, thrust_angle_rad, thrust_error_angle_rad);

    // Todo: Limit roll an pitch error based on output saturation and maximum error.

    // Limit Yaw Error based to the maximum that would saturate the output when yaw rate is zero.
    Quaternion heading_vec_correction_quat;

    float heading_accel_max = constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS);
    if (!is_zero(get_rate_yaw_pid().kP())) {
        float heading_error_max = MIN(utb3_inv_sqrt_controller(1.0 / get_rate_yaw_pid().kP(), _p_angle_yaw.kP(), heading_accel_max), AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD);
        if (!is_zero(_p_angle_yaw.kP()) && fabsf(attitude_error_rad.z) > heading_error_max) {
            attitude_error_rad.z = constrain_float(wrap_PI(attitude_error_rad.z), -heading_error_max, heading_error_max);
            heading_vec_correction_quat.from_axis_angle(Vector3f{0.0f, 0.0f, attitude_error_rad.z});
            attitude_target = attitude_body * thrust_vector_correction * heading_vec_correction_quat;
        }
    }
}

// thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
void UTB_Attitude::thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const
{
    // The direction of thrust is [0,0,-1] is any body-fixed frame, inc. body frame and target frame.
    const Vector3f thrust_vector_up{0.0f, 0.0f, -1.0f};

    // attitude_target and attitude_body are passive rotations from target / body frames to the NED frame

    // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the target thrust vector in the inertial frame
    const Vector3f att_target_thrust_vec = attitude_target * thrust_vector_up; // target thrust vector

    // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the current thrust vector in the inertial frame
    const Vector3f att_body_thrust_vec = attitude_body * thrust_vector_up; // current thrust vector

    // the dot product is used to calculate the current lean angle for use of external functions
    thrust_angle_rad = acosf(constrain_float(thrust_vector_up * att_body_thrust_vec,-1.0f,1.0f));

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_vec_cross = att_body_thrust_vec % att_target_thrust_vec;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    thrust_error_angle_rad = acosf(constrain_float(att_body_thrust_vec * att_target_thrust_vec, -1.0f, 1.0f));

    // Normalize the thrust rotation vector
    float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_error_angle_rad)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // thrust_vector_correction is defined relative to the body frame but its axis `thrust_vec_cross` was computed in
    // the inertial frame. First rotate it by the inverse of attitude_body to express it back in the body frame
    thrust_vec_cross = attitude_body.inverse() * thrust_vec_cross;
    thrust_vector_correction.from_axis_angle(thrust_vec_cross, thrust_error_angle_rad);

    // calculate the angle error in x and y.
    Vector3f rotation_rad;
    thrust_vector_correction.to_axis_angle(rotation_rad);
    attitude_error_rad.x = rotation_rad.x;
    attitude_error_rad.y = rotation_rad.y;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the body frame
    // heading_vector_correction
    Quaternion heading_vec_correction_quat = thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;

    // calculate the angle error in z (x and y should be zero here).
    heading_vec_correction_quat.to_axis_angle(rotation_rad);
    attitude_error_rad.z = rotation_rad.z;
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using _input_tc
void UTB_Attitude::attitude_command_model(float error_angle, float desired_ang_vel, float& target_ang_vel, float& target_ang_accel, float max_ang_vel, float accel_max, float input_tc, float dt) const
{
    if (!is_positive(dt)) {
        return;
    }

    // protect against divide by zero
    if (!is_positive(accel_max)) {
        // no acceleration set so default to 1800 degrees/s²
        accel_max = radians(1800);
    }

    if (!is_positive(input_tc)) {
        // no acceleration set so default to achieve maximum acceleration in 10 clock cycles
        input_tc = dt * 10.0;
    }

    utb3_shape_angle_vel_accel( error_angle, desired_ang_vel, 0.0,
                                0.0, target_ang_vel, target_ang_accel,
                                -max_ang_vel, max_ang_vel, accel_max,
                                accel_max / input_tc, dt, true);
    target_ang_vel += target_ang_accel * dt;
}

// scale I to represent the current angle P
void UTB_Attitude::scale_I_to_angle_P()
{
    Vector3f i_scale{
        _p_angle_roll.kP() * _angle_P_scale.x,
                     _p_angle_pitch.kP() * _angle_P_scale.y,
                     _p_angle_yaw.kP() * _angle_P_scale.z
    };
    set_I_scale_mult(i_scale);
}

// Limits angular velocity in roll, pitch, and yaw.
// Roll and pitch are coupled: when both limits are set, they are scaled
// proportionally as a 2D vector, preserving their ratio. Yaw is clamped independently
void UTB_Attitude::ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max_rads, float ang_vel_pitch_max_rads, float ang_vel_yaw_max_rads) const
{
    if (is_zero(ang_vel_roll_max_rads) || is_zero(ang_vel_pitch_max_rads)) {
        if (!is_zero(ang_vel_roll_max_rads)) {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max_rads, ang_vel_roll_max_rads);
        }
        if (!is_zero(ang_vel_pitch_max_rads)) {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max_rads, ang_vel_pitch_max_rads);
        }
    } else {
        const Vector2f thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max_rads, euler_rad.y / ang_vel_pitch_max_rads);
        float thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max_rads / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max_rads / thrust_vector_length;
        }
    }
    if (!is_zero(ang_vel_yaw_max_rads)) {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max_rads, ang_vel_yaw_max_rads);
    }
}

// translates body frame rotation rates and acceleration limits to the euler axis
Vector3f UTB_Attitude::body_to_euler_limit(const Quaternion &att, const Vector3f &body_limit)
{
    if (!is_positive(body_limit.x) || !is_positive(body_limit.y) || !is_positive(body_limit.z)) {
        return Vector3f { body_limit };
    }

    const float phi = att.get_euler_roll();
    const float theta = att.get_euler_pitch();

    const float sin_phi = constrain_float(fabsf(sinf(phi)), 0.1f, 1.0f);
    const float cos_phi = constrain_float(fabsf(cosf(phi)), 0.1f, 1.0f);
    const float sin_theta = constrain_float(fabsf(sinf(theta)), 0.1f, 1.0f);
    const float cos_theta = constrain_float(fabsf(cosf(theta)), 0.1f, 1.0f);

    return Vector3f {
        body_limit.x,
        MIN(body_limit.y / cos_phi, body_limit.z / sin_phi),
        MIN(MIN(body_limit.x / sin_theta, body_limit.y / (sin_phi * cos_theta)), body_limit.z / (cos_phi * cos_theta))
    };
}

// Sets attitude target to vehicle attitude and sets all rates to zero
// If reset_rate is false rates are not reset to allow the rate controllers to run
void UTB_Attitude::reset_target_and_rate(bool reset_rate)
{
    // move attitude target to current attitude
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);

    if (reset_rate) {
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
        _euler_rate_target_rads.zero();
    }
}

// Sets yaw target to vehicle heading and sets yaw rate to zero
// If reset_rate is false rates are not reset to allow the rate controllers to run
void UTB_Attitude::reset_yaw_target_and_rate(bool reset_rate)
{
    // move attitude target to current heading
    float yaw_shift = _ahrs.yaw() - _euler_angle_target_rad.z;
    Quaternion _attitude_target_update;
    _attitude_target_update.from_axis_angle(Vector3f{0.0f, 0.0f, yaw_shift});
    _attitude_target = _attitude_target_update * _attitude_target;

    if (reset_rate) {
        // set yaw rate to zero
        _euler_rate_target_rads.z = 0.0f;
        _ang_accel_target_rads.z = 0.0;

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_derivative_to_body(_attitude_target, _euler_rate_target_rads, _ang_vel_target_rads);
    }
}

// Shifts the target attitude to maintain the current error in the event of an EKF reset
void UTB_Attitude::inertial_frame_reset()
{
    // Retrieve quaternion body attitude
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // Recalculate the target quaternion
    _attitude_target = attitude_body * _attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);
}

// euler_derivative_to_body - transform euler angle derivative to body-frame
// Converts euler derivatives (rate, acceleration, etc.) to body-frame equivalents.
// The same transformation applies regardless of derivative order.
// Uses the kinematic relationship for 321 (yaw-pitch-roll) euler sequence.
void UTB_Attitude::euler_derivative_to_body(const Quaternion& att, const Vector3f& euler_derivative_rads, Vector3f& body_derivative_rads)
{
    const float theta = att.get_euler_pitch();
    const float phi = att.get_euler_roll();

    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    const float sin_phi = sinf(phi);
    const float cos_phi = cosf(phi);

    body_derivative_rads.x = euler_derivative_rads.x - sin_theta * euler_derivative_rads.z;
    body_derivative_rads.y = cos_phi * euler_derivative_rads.y + sin_phi * cos_theta * euler_derivative_rads.z;
    body_derivative_rads.z = -sin_phi * euler_derivative_rads.y + cos_theta * cos_phi * euler_derivative_rads.z;
}

// body_to_euler_derivative - transform body-frame derivative to euler angle derivative
// Converts body-frame derivatives (rate, acceleration, etc.) to euler equivalents.
// The same transformation applies regardless of derivative order.
// Uses the kinematic relationship for 321 (yaw-pitch-roll) euler sequence.
// Returns false if the vehicle is pitched 90 degrees up or down (gimbal lock)
bool UTB_Attitude::body_to_euler_derivative(const Quaternion& att, const Vector3f& body_derivative_rads, Vector3f& euler_derivative_rads)
{
    const float theta = att.get_euler_pitch();
    const float phi = att.get_euler_roll();

    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    const float sin_phi = sinf(phi);
    const float cos_phi = cosf(phi);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_derivative_rads.x = body_derivative_rads.x + sin_phi * (sin_theta / cos_theta) * body_derivative_rads.y + cos_phi * (sin_theta / cos_theta) * body_derivative_rads.z;
    euler_derivative_rads.y = cos_phi * body_derivative_rads.y - sin_phi * body_derivative_rads.z;
    euler_derivative_rads.z = (sin_phi / cos_theta) * body_derivative_rads.y + (cos_phi / cos_theta) * body_derivative_rads.z;
    return true;
}

// Update rate_target_ang_vel using attitude_error_rot_vec_rad
Vector3f UTB_Attitude::update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad)
{
    Vector3f rate_target_ang_vel;

    // Compute the roll angular velocity demand from the roll angle error
    const float angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
    if (_use_sqrt_controller && !is_zero(get_accel_roll_max_radss())) {
        rate_target_ang_vel.x = utb3_sqrt_controller(attitude_error_rot_vec_rad.x, angleP_roll, constrain_float(get_accel_roll_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt_s);
    } else {
        rate_target_ang_vel.x = angleP_roll * attitude_error_rot_vec_rad.x;
    }

    // Compute the pitch angular velocity demand from the pitch angle error
    const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
    if (_use_sqrt_controller && !is_zero(get_accel_pitch_max_radss())) {
        rate_target_ang_vel.y = utb3_sqrt_controller(attitude_error_rot_vec_rad.y, angleP_pitch, constrain_float(get_accel_pitch_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt_s);
    } else {
        rate_target_ang_vel.y = angleP_pitch * attitude_error_rot_vec_rad.y;
    }

    // Compute the yaw angular velocity demand from the yaw angle error
    const float angleP_yaw = _p_angle_yaw.kP() * _angle_P_scale.z;
    if (_use_sqrt_controller && !is_zero(get_accel_yaw_max_radss())) {
        rate_target_ang_vel.z = utb3_sqrt_controller(attitude_error_rot_vec_rad.z, angleP_yaw, constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS), _dt_s);
    } else {
        rate_target_ang_vel.z = angleP_yaw * attitude_error_rot_vec_rad.z;
    }

    return rate_target_ang_vel;
}

// Return configured tilt angle limit in radians
float UTB_Attitude::lean_angle_max_rad() const
{
    return radians(constrain_float(_angle_max_deg.get(), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MAX));
}

bool UTB_Attitude::pre_arm_checks(const char *param_prefix,
                                  char *failure_msg,
                                  const uint8_t failure_msg_len)
{
    // validate AC_P members:
    const struct {
        const char *pid_name;
        AC_P &p;
    } ps[] = {
        { "ANG_PIT", get_angle_pitch_p() },
        { "ANG_RLL", get_angle_roll_p() },
        { "ANG_YAW", get_angle_yaw_p() }
    };
    for (uint8_t i=0; i<ARRAY_SIZE(ps); i++) {
        // all AC_P's must have a positive P value:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // validate UTB_PID members:
    const struct {
        const char *pid_name;
        UTB_PID &pid;
    } pids[] = {
        { "RAT_RLL", get_rate_roll_pid() },
        { "RAT_PIT", get_rate_pitch_pid() },
        { "RAT_YAW", get_rate_yaw_pid() },
    };
    for (uint8_t i=0; i<ARRAY_SIZE(pids); i++) {
        // if the PID has a positive FF then we just ensure kP and
        // kI aren't negative
        UTB_PID &pid = pids[i].pid;
        const char *pid_name = pids[i].pid_name;
        if (is_positive(pid.ff())) {
            // kP and kI must be non-negative:
            if (is_negative(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be >= 0", param_prefix, pid_name);
                return false;
            }
            if (is_negative(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be >= 0", param_prefix, pid_name);
                return false;
            }
        } else {
            // kP and kI must be positive:
            if (!is_positive(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, pid_name);
                return false;
            }
            if (!is_positive(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be > 0", param_prefix, pid_name);
                return false;
            }
        }
        // never allow a negative D term (but zero is allowed)
        if (is_negative(pid.kD())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_D must be >= 0", param_prefix, pid_name);
            return false;
        }
    }

    // validate ANGLE_MAX
    if (_angle_max_deg.get() < 10 || _angle_max_deg.get() > 80) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ANGLE_MAX must be >= 10 and <= 80", param_prefix);
        return false;
    }

    return true;
}

#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "UTB_Attitude.h"
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AC_AttitudeControl/LogStructure.h>

// Write an ANG packet
void UTB_Attitude::Write_ANG() const
{
    Vector3f targets = get_att_target_euler_rad() * RAD_TO_DEG;

    const struct log_ANG pkt {
        LOG_PACKET_HEADER_INIT(LOG_ANG_MSG),
                           time_us         : AP::scheduler().get_loop_start_time_us(),
                           control_roll    : targets.x,
                           roll            : degrees(_ahrs.roll()),
                           control_pitch   : targets.y,
                           pitch           : degrees(_ahrs.pitch()),
                           control_yaw     : wrap_360(targets.z),
                           yaw             : wrap_360(degrees(_ahrs.yaw())),
                           sensor_dt       : AP::scheduler().get_last_loop_time_s()
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void UTB_Attitude::Write_Rate(const AC_PosControl &pos_control) const
{
    const Vector3f rate_targets_degs = rate_bf_targets() * RAD_TO_DEG;
    const Vector3f &accel_target_ned_mss = pos_control.get_accel_target_NED_mss();
    const Vector3f gyro_rate = _rate_gyro_rads * RAD_TO_DEG;
    const struct log_Rate pkt_rate {
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
                           time_us         : _rate_gyro_time_us,
                           control_roll    : rate_targets_degs.x,
                           roll            : gyro_rate.x,
                           roll_out        : _motors.get_roll()+_motors.get_roll_ff(),
                           control_pitch   : rate_targets_degs.y,
                           pitch           : gyro_rate.y,
                           pitch_out       : _motors.get_pitch()+_motors.get_pitch_ff(),
                           control_yaw     : rate_targets_degs.z,
                           yaw             : gyro_rate.z,
                           yaw_out         : _motors.get_yaw()+_motors.get_yaw_ff(),
                           control_accel   : (float)(-accel_target_ned_mss.z),
                           accel           : (float)(-(_ahrs.get_accel_ef().z + GRAVITY_MSS)),
                           accel_out       : _motors.get_throttle(),
                           throttle_slew   : _motors.get_throttle_slew_rate()
    };
    AP::logger().WriteBlock(&pkt_rate, sizeof(pkt_rate));

    /*
      log P/PD gain scale if not == 1.0
     */
    const Vector3f &scale = get_last_angle_P_scale();
    const Vector3f &pd_scale = _pd_scale_used;
    const Vector3f &i_scale = _i_scale_used;
    if (scale != UTB_Attitude::VECTORF_111
        || pd_scale != UTB_Attitude::VECTORF_111
        || i_scale != UTB_Attitude::VECTORF_111) {
        const struct log_ATSC pkt_ATSC {
            LOG_PACKET_HEADER_INIT(LOG_ATSC_MSG),
                               time_us  : _rate_gyro_time_us,
                               scaleP_x : scale.x,
                               scaleP_y : scale.y,
                               scaleP_z : scale.z,
                               scalePD_x : pd_scale.x,
                               scalePD_y : pd_scale.y,
                               scalePD_z : pd_scale.z,
                               scaleI_x : i_scale.x,
                               scaleI_y : i_scale.y,
                               scaleI_z : i_scale.z,
        };
        AP::logger().WriteBlock(&pkt_ATSC, sizeof(pkt_ATSC));
    }
}

#endif // HAL_LOGGING_ENABLED

#endif // AP_UTB_RATE_SITL_AVAILABLE
