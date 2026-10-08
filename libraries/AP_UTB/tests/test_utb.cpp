#include <AP_gtest.h>
#include <AP_UTB/AP_UTB.h>
#include <AP_UTB/AP_UTB_MotorMixer.h>
#include <limits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_UTB_ENABLED
static AP_UTB_State valid_state(uint32_t time_us)
{
    AP_UTB_State state;
    state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
    state.rates_rads = Vector3f(0.1f, -0.2f, 0.3f);
    state.sampled_us = time_us;
    state.imu_updated_us = time_us;
    state.attitude_valid = true;
    state.rates_valid = true;
    return state;
}

TEST(UTBState, RejectsMissingAndStaleInputs)
{
    AP_UTB_State state;
    EXPECT_FALSE(state.valid(1000, 100));
    state = valid_state(1000);
    EXPECT_TRUE(state.valid(1100, 100));
    EXPECT_FALSE(state.valid(1101, 100));
    state.sampled_us = 1100;
    EXPECT_FALSE(state.valid(1101, 100)); // fresh read cannot hide stale IMU
    state = valid_state(1000);
    state.attitude_valid = false;
    EXPECT_FALSE(state.valid(1000, 100));
    state = valid_state(1000);
    state.rates_valid = false;
    EXPECT_FALSE(state.valid(1000, 100));
}

TEST(UTBState, HandlesClockWrap)
{
    const AP_UTB_State state = valid_state(UINT32_MAX - 10);
    EXPECT_TRUE(state.valid(9, 20));
    EXPECT_FALSE(state.valid(10, 20));
}

TEST(UTBState, RejectsInvalidNumbersAndQuaternion)
{
    AP_UTB_State state = valid_state(1000);
    state.rates_rads.x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(state.valid(1000, 100));
    state = valid_state(1000);
    state.rates_rads.z = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(state.valid(1000, 100));
    state = valid_state(1000);
    state.attitude_body_to_ned.q1 = 0;
    EXPECT_FALSE(state.valid(1000, 100));
    state.attitude_body_to_ned.q1 = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(state.valid(1000, 100));
}

TEST(UTBState, UninitialisedProviderClearsPreviousSnapshot)
{
    AP_UTB_APStateProvider provider;
    AP_UTB_State state = valid_state(1000);
    EXPECT_FALSE(provider.sample(state));
    EXPECT_FALSE(state.valid(1000, 100));
}

TEST(UTBSkeleton, DisabledAndUnhealthyByDefault)
{
    AP_UTB manager;
    EXPECT_FALSE(manager.enabled());
    EXPECT_FALSE(manager.healthy());
    manager.update_state();
    EXPECT_FALSE(manager.state().valid(0, 100));
}

TEST(UTBSkeleton, NeverProducesValidControlOrMotorOutputs)
{
    AP_UTB_RateController controller;
    AP_UTB_MotorMixer mixer;
    auto control = controller.update(valid_state(1000), Vector3f(1, 2, 3), 0.0025f);
    EXPECT_FALSE(control.valid);
    // Even a caller supplying a valid-looking command cannot activate the skeleton.
    control.valid = true;
    control.command = Vector3f(1, 1, 1);
    const auto motors = mixer.mix(control, 1.0f);
    EXPECT_FALSE(motors.valid);
    for (const float thrust : motors.thrust) {
        EXPECT_FLOAT_EQ(thrust, 0.0f);
    }
    EXPECT_FALSE(controller.update({}, {}, 0).valid);
}
#endif // AP_UTB_ENABLED

AP_GTEST_MAIN()
