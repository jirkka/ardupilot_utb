// GNU GPL v3 or later.
/// @file test_utb_native_bench.cpp
/// @author Jiri Kovar
/// @version 2026-10-19
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_UTB/AP_UTB_NativeBench.h>
#if AP_UTB_NATIVE_BENCH_AVAILABLE
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
class UTB_NativeBenchTestAccess
{
public:
    static bool step(AP_UTB_NativeBench &bench, uint32_t seq)
    {
        bench._seq = seq;
        return bench.compute();
    }
    static float difference(const AP_UTB_NativeBench &bench)
    {
        return bench.max_difference();
    }
    static bool ready(const AP_UTB_NativeBench &bench)
    {
        return bench._session != nullptr;
    }
    static void request(AP_UTB_NativeBench &bench)
    {
        bench._run.set(1);
    }
    static int run(const AP_UTB_NativeBench &bench)
    {
        return bench._run.get();
    }
};
TEST(UTBNativeBench, IndependentSingletonsAndAllProfiles)
{
    static AP_Scheduler scheduler;
    static AP_InertialSensor ins;
    static AP_AHRS ahrs;
    static AP_AHRS_View view(ahrs, ROTATION_NONE);
    static AP_MotorsMatrix live_motors;
    static AC_AttitudeControl_Multi live_controller(view, live_motors);
    static AP_UTB_NativeBench bench;
    live_controller.get_rate_roll_pid().set_integrator(.11f);
    live_motors.set_roll(.123f);
    const float before = live_controller.get_rate_roll_pid().get_i();
    bench.init(view);
    ASSERT_TRUE(UTB_NativeBenchTestAccess::ready(bench));
    ASSERT_EQ(AP_Motors::get_singleton(), &live_motors);
    ASSERT_EQ(AC_AttitudeControl::get_singleton(), &live_controller);
    for (uint32_t seq = 0; seq < 2000; seq++) {
        ASSERT_TRUE(UTB_NativeBenchTestAccess::step(bench, seq));
        ASSERT_LE(UTB_NativeBenchTestAccess::difference(bench), 1.0e-5f) << seq;
    }
    EXPECT_EQ(AP_Motors::get_singleton(), &live_motors);
    EXPECT_EQ(AC_AttitudeControl::get_singleton(), &live_controller);
    EXPECT_FLOAT_EQ(live_motors.get_roll(), .123f);
    EXPECT_FLOAT_EQ(live_motors.get_pitch(), 0);
    EXPECT_FLOAT_EQ(live_motors.get_yaw(), 0);
    EXPECT_FLOAT_EQ(live_controller.get_rate_roll_pid().get_i(), before);
    UTB_NativeBenchTestAccess::request(bench);
    bench.update(true);
    EXPECT_EQ(UTB_NativeBenchTestAccess::run(bench), 0);
}
#else
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
TEST(UTBNativeBench, CompiledOut)
{
    SUCCEED();
}
#endif
AP_GTEST_MAIN()
