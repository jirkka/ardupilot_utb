/*
 * AP-derived UTB PID differential tests against the native AC_PID.
 * GNU GPL v3 or later. Original algorithms: ArduPilot contributors.
 */
/// @file test_utb_native.cpp
/// @brief Independent native AP implementation is the numerical oracle.
/// @author Jiri Kovar
/// @version 2026-10-19
/// @date 2026-10-19
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#include <AC_PID/AC_PID.h>
#include <AP_UTB/UTB_PID.h>
#include <AP_UTB/UTB_Input.h>
#include <AP_UTB/UTB_Rate.h>
#include <AP_UTB/UTB_Core.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#if AP_UTB_RATE_SITL_AVAILABLE

// Private functions: static objects reproduce native zero-initialised controller storage.
template <unsigned scenario> void compare_pid()
{
    static AC_PID native(.135f, .135f, .0036f, .07f, .5f, 15, 7, 20, 50, 1, .002f);
    static UTB_PID utb(.135f, .135f, .0036f, .07f, .5f, 15, 7, 20, 50, 1, .002f);
    float target;
    float measured;
    float dt;
    float expected;
    float actual;
    bool limited;
    unsigned sample;
    native.reset_I(); utb.reset_I();
    native.reset_filter(); utb.reset_filter();
    native.set_pdmax(scenario == 1 ? .2f : 0);
    utb.set_pdmax(scenario == 1 ? .2f : 0);
    for (sample = 0; sample < 10000; sample++) {
        dt = .0025f + (sample % 3) * .00001f;
        target = sinf(sample * .01f) * (scenario == 1 ? 10 : 2);
        measured = cosf(sample * .017f);
        limited = scenario != 0 && (sample % 90) < 40;
        if (scenario == 2 && (sample % 113) == 0) {
            native.reset_filter(); utb.reset_filter();
        }
        expected = native.update_all(target, measured, dt, limited, .8f, .7f);
        actual = utb.update_all(target, measured, dt, limited, .8f, .7f);
        ASSERT_FLOAT_EQ(expected, actual) << sample;
        ASSERT_FLOAT_EQ(native.get_ff(), utb.get_ff());
        ASSERT_FLOAT_EQ(native.get_pid_info().P, utb.get_pid_info().P);
        ASSERT_FLOAT_EQ(native.get_pid_info().I, utb.get_pid_info().I);
        ASSERT_FLOAT_EQ(native.get_pid_info().D, utb.get_pid_info().D);
        ASSERT_FLOAT_EQ(native.get_pid_info().Dmod, utb.get_pid_info().Dmod);
    }
    ASSERT_FLOAT_EQ(native.update_all(NAN, 0, .0025f), utb.update_all(NAN, 0, .0025f));
    ASSERT_FLOAT_EQ(native.update_all(0, INFINITY, .0025f), utb.update_all(0, INFINITY, .0025f));
}
// End Private functions
// Public functions: no physical motor or external endpoint is opened.
TEST(UTBNativePID, FilterAndFeedforwardParity)
{
    compare_pid<0>();
}
TEST(UTBNativePID, SaturationAndPDLimitParity)
{
    compare_pid<1>();
}
TEST(UTBNativePID, FilterResetParity)
{
    compare_pid<2>();
}
TEST(UTBNativeInput, ActualAPExpoAndAngleParity)
{
    float input;
    float expo;
    float roll;
    float pitch;
    float expected_roll;
    float expected_pitch;
    Vector2f actual;
    for (int i = -100; i <= 100; i++) {
        input = i * .01f;
        for (int j = 0; j <= 10; j++) {
            expo = j * .1f;
            ASSERT_FLOAT_EQ(input_expo(input, expo), UTB_Input::expo(input, expo));
        }
        for (int j = -100; j <= 100; j++) {
            roll = input; pitch = j * .01f;
            rc_input_to_roll_pitch_rad(roll, pitch, radians(60.0f), radians(60.0f), expected_roll, expected_pitch);
            actual = UTB_Input::angle(roll, pitch, radians(60.0f));
            ASSERT_FLOAT_EQ(expected_roll, actual.x);
            ASSERT_FLOAT_EQ(expected_pitch, actual.y);
        }
    }
}
// The same read-only AHRS and in-memory motor object are supplied to both algorithms.
// No INS drivers, arming, motor output(), scheduler or physical device is started.
TEST(UTBNativeAttitude, QuaternionAndShapingParity)
{
    static AP_Scheduler scheduler;
    static AP_InertialSensor ins;
    static AP_AHRS ahrs;
    static AP_AHRS_View view(ahrs, ROTATION_NONE);
    static AP_MotorsMatrix motors;
    static UTB_State state(view);
    static AC_AttitudeControl_Multi native(view, motors);
    static UTB_Rate utb(state, motors);
    Vector3f expected;
    Vector3f actual;
    Quaternion command;
    Quaternion body;
    Vector3f expected_error;
    Vector3f actual_error;
    float expected_angle;
    float actual_angle;
    float expected_thrust_error;
    float actual_thrust_error;
    float t;
    native.set_dt_s(.0025f);
    utb.set_dt_s(.0025f);
    native.set_roll_pitch_rate_tc(.1f);
    utb.set_roll_pitch_rate_tc(.1f);
    native.set_yaw_rate_tc(.1f);
    utb.set_yaw_rate_tc(.1f);
    for (unsigned sample = 0; sample < 4000; sample++) {
        t = sample * .01f;
        command.from_euler(sinf(t) * 3, cosf(t) * 1.5f, sinf(t * .7f) * 3);
        body.from_euler(cosf(t) * 3, sinf(t) * 1.5f, cosf(t * .7f) * 3);
        Quaternion native_target = command;
        Quaternion utb_target = command;
        native.thrust_heading_rotation_angles(native_target, body, expected_error, expected_angle, expected_thrust_error);
        utb.thrust_heading_rotation_angles(utb_target, body, actual_error, actual_angle, actual_thrust_error);
        ASSERT_FLOAT_EQ(expected_angle, actual_angle);
        ASSERT_FLOAT_EQ(expected_thrust_error, actual_thrust_error);
        for (uint8_t axis = 0; axis < 3; axis++) {
            ASSERT_FLOAT_EQ(expected_error[axis], actual_error[axis]);
        }
        native.input_rate_bf_roll_pitch_yaw_2_rads(sinf(t) * 5, cosf(t) * 5, sinf(t) * 2);
        utb.input_rate_bf_roll_pitch_yaw_2_rads(sinf(t) * 5, cosf(t) * 5, sinf(t) * 2);
        expected = native.rate_bf_targets();
        actual = utb.rate_bf_targets();
        for (uint8_t axis = 0; axis < 3; axis++) {
            ASSERT_FLOAT_EQ(expected[axis], actual[axis]);
        }
        native.inertial_frame_reset();
        utb.inertial_frame_reset();
        native.input_euler_angle_roll_pitch_euler_rate_yaw_rad(sinf(t) * .5f, cosf(t) * .5f, sinf(t));
        utb.input_euler_angle_roll_pitch_euler_rate_yaw_rad(sinf(t) * .5f, cosf(t) * .5f, sinf(t));
        expected = native.rate_bf_targets();
        actual = utb.rate_bf_targets();
        for (uint8_t axis = 0; axis < 3; axis++) {
            ASSERT_FLOAT_EQ(expected[axis], actual[axis]);
        }
    }
    // PID telemetry can lag a later reset_I(). Handoff must transfer live I, not that snapshot.
    static UTB_Core core(view, motors);
    core.rate.set_throttle_mix_value(.17f);
    core.rate.get_rate_roll_pid().set_integrator(.2f);
    core.prepare_native(native);
    ASSERT_FLOAT_EQ(native.get_rate_roll_pid().get_i(), .2f);
    ASSERT_FLOAT_EQ(native.get_throttle_mix(), .17f);
    core.rate.get_rate_roll_pid().update_all(0, 0, .0025f);
    ASSERT_FLOAT_EQ(core.rate.get_rate_roll_pid().get_pid_info().I, .2f);
    core.rate.get_rate_roll_pid().reset_I();
    core.prepare_native(native);
    ASSERT_FLOAT_EQ(native.get_rate_roll_pid().get_i(), 0);
}
#endif
AP_GTEST_MAIN()
