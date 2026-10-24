// GNU GPL v3 or later. Original algorithms: ArduPilot contributors.
/// @file test_utb_active.cpp
/// @brief In-memory motor authority tests; no arming or motor output() call.
/// @author Jiri Kovar
/// @version 2026-10-24
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#include <AP_UTB/UTB_Core.h>
#if AP_UTB_RATE_SITL_AVAILABLE
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>

struct ActiveFixture {
    AP_Scheduler scheduler;
    AP_InertialSensor ins;
    AP_AHRS ahrs;
    AP_AHRS_View view{ahrs, ROTATION_NONE};
    AP_MotorsMatrix motors;
    UTB_State state{view};
    UTB_Rate rate{state, motors};
    ActiveFixture()
    {
        rate.set_dt_s(.0025f);
        motors.set_dt_s(.0025f);
        rate.set_accel_roll_max_radss(0);
        rate.set_accel_pitch_max_radss(0);
        rate.set_accel_yaw_max_radss(0);
        rate.set_roll_pitch_rate_tc(0);
        rate.set_yaw_rate_tc(0);
    }
    void reset()
    {
        UTB_PID *axes[] = {&rate.get_rate_roll_pid(), &rate.get_rate_pitch_pid(), &rate.get_rate_yaw_pid()};
        for (UTB_PID *pid : axes) {
            pid->set_kP(.1f); pid->set_kI(0); pid->set_kD(0); pid->set_ff(0); pid->set_kDff(0);
            pid->set_filt_T_hz(0); pid->set_filt_E_hz(0); pid->set_filt_D_hz(0);
            pid->set_slew_limit(0); pid->set_pdmax(0); pid->set_imax(.5f);
            pid->reset_I(); pid->reset_filter();
        }
        motors.limit.roll = motors.limit.pitch = motors.limit.yaw = false;
        rate.reset_target_and_rate(true);
    }
    void step(const Vector3f &target)
    {
        rate.input_rate_bf_roll_pitch_yaw_2_rads(target.x, target.y, target.z);
        rate.rate_controller_run_dt({}, .0025f);
    }
};
static ActiveFixture &fixture()
{
    static ActiveFixture instance;
    instance.reset();
    return instance;
}

TEST(UTBActiveAuthority, OutputIsAnActualMotorInput)
{
    ActiveFixture &f = fixture();
    UTB_Output output(f.motors);
    output.begin();
    output.roll(.125f); output.pitch(-.25f); output.yaw(.375f);
    output.roll_ff(.01f); output.pitch_ff(.02f); output.yaw_ff(-.03f);
    EXPECT_FLOAT_EQ(f.motors.get_roll(), .125f);
    EXPECT_FLOAT_EQ(f.motors.get_pitch(), -.25f);
    EXPECT_FLOAT_EQ(f.motors.get_yaw(), .375f);
    EXPECT_FLOAT_EQ(f.motors.get_roll_ff(), .01f);
    EXPECT_FLOAT_EQ(f.motors.get_pitch_ff(), .02f);
    EXPECT_FLOAT_EQ(f.motors.get_yaw_ff(), -.03f);
    EXPECT_EQ(output.writes(), 6);
    // Test-only injection: same context, change only the supplied own output.
    output.roll(-.0625f);
    EXPECT_FLOAT_EQ(f.motors.get_roll(), -.0625f);
    EXPECT_LT(f.motors.get_roll(), 0);
    EXPECT_FLOAT_EQ(f.motors.get_pitch(), -.25f);
}
TEST(UTBActiveAuthority, OwnRateAndOwnGainDriveTheMotor)
{
    ActiveFixture &f = fixture();
    float first;
    // Settle the real input shaper before comparing identical rate references.
    for (unsigned sample = 0; sample < 500; sample++) {
        f.step({.2f, -.3f, .1f});
    }
    first = f.motors.get_roll();
    EXPECT_GT(first, 0);
    EXPECT_LT(f.motors.get_pitch(), 0);
    EXPECT_FLOAT_EQ(first, f.rate.output_control().x);
    EXPECT_FLOAT_EQ(first, f.rate.get_rate_roll_pid().get_pid_info().P);
    EXPECT_EQ(f.rate.output_writes(), 6);
    f.rate.get_rate_roll_pid().set_kP(.2f);
    f.step({.2f, -.3f, .1f});
    EXPECT_FLOAT_EQ(f.motors.get_roll(), first * 2);
    EXPECT_EQ(f.rate.output_writes(), 6);
}
TEST(UTBActiveAuthority, RealMotorLimitControlsDirectionalIntegration)
{
    ActiveFixture &f = fixture();
    f.rate.get_rate_roll_pid().set_kI(1);
    f.rate.get_rate_roll_pid().set_integrator(.1f);
    f.motors.limit.roll = true;
    f.step({.2f, 0, 0});
    EXPECT_FLOAT_EQ(f.rate.get_rate_roll_pid().get_i(), .1f);
    // A reference sign change is shaped, so wait for the actual PID error to reverse.
    for (unsigned sample = 0; sample < 500; sample++) {
        f.step({-.2f, 0, 0});
    }
    EXPECT_LT(f.rate.get_rate_roll_pid().get_i(), .1f);
    f.rate.reset_rate_controller_I_terms();
    EXPECT_FLOAT_EQ(f.rate.get_rate_roll_pid().get_i(), 0);
    EXPECT_TRUE(f.motors.limit.roll);
}
TEST(UTBActiveAuthority, FeedforwardUsesItsOwnOutputSetter)
{
    ActiveFixture &f = fixture();
    f.rate.get_rate_roll_pid().set_ff(.1f);
    f.step({.2f, 0, 0});
    EXPECT_GT(f.motors.get_roll_ff(), 0);
    EXPECT_FLOAT_EQ(f.motors.get_roll_ff(), f.rate.get_rate_roll_pid().get_ff());
    EXPECT_FLOAT_EQ(f.motors.get_roll_ff(), f.rate.output_feedforward().x);
}
#endif
AP_GTEST_MAIN()
