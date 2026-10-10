#include <AP_gtest.h>
#include <AP_UTB/AP_UTB.h>
#include <AP_UTB/AP_UTB_MotorMixer.h>
#include <AP_UTB/AP_UTB_Reference.h>
#include <limits>
#include <utility>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <thread>
#endif

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

static AP_UTB_RateController::Gains proportional(float gain)
{
    return {gain, 0, 0, 0, 0};
}

static AP_UTB_State zero_state()
{
    auto state = valid_state(1000);
    state.rates_rads.zero();
    return state;
}

TEST(UTBRate, ProportionalSignsAndIndependentAxes)
{
    for (float gain : {
             0.0f, 0.2f
         }) {
        AP_UTB_RateController c;
        AP_UTB_RateController::Gains gains[3] = {proportional(gain), proportional(gain), proportional(gain)};
        ASSERT_TRUE(c.configure(gains, 0.0025f));
        auto state = zero_state();
        EXPECT_FALSE(c.update(state, {}, 0.0025f).valid);
        const auto result = c.update(state, {1, -2, 3}, 0.0025f);
        ASSERT_TRUE(result.valid);
        EXPECT_FLOAT_EQ(result.command.x, gain);
        EXPECT_FLOAT_EQ(result.command.y, -2 * gain);
        EXPECT_FLOAT_EQ(result.command.z, 3 * gain);
        EXPECT_FLOAT_EQ(result.axis[0].d, 0);
    }
}

TEST(UTBRate, IntegralAccumulationAndBothClamps)
{
    for (float sign : {
             -1.0f, 1.0f
         }) {
        AP_UTB_RateController c;
        AP_UTB_RateController::Gains gains[3] = {{0, 1, 0, 0.03f, 0}, {}, {}};
        ASSERT_TRUE(c.configure(gains, 0.01f));
        auto state = zero_state();
        c.update(state, {sign, 0, 0}, 0.01f);
        EXPECT_NEAR(c.update(state, {sign, 0, 0}, 0.01f).axis[0].i, sign * 0.01f, 1.0e-6f);
        EXPECT_NEAR(c.update(state, {sign, 0, 0}, 0.02f).axis[0].i, sign * 0.03f, 1.0e-6f);
        EXPECT_NEAR(c.update(state, {sign, 0, 0}, 0.01f).axis[0].i, sign * 0.03f, 1.0e-6f);
        c.reset();
        EXPECT_FALSE(c.update(state, {}, 0.01f).valid);
        EXPECT_FLOAT_EQ(c.update(state, {}, 0.01f).axis[0].i, 0);
    }
}

TEST(UTBRate, MeasurementDerivativeBackwardEulerAndVariableDt)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{0, 0, 0.1f, 0, 10}, {}, {}};
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    auto state = zero_state();
    state.rates_rads.x = 3;
    EXPECT_TRUE(c.update(state, {}, 0.0025f).priming);
    const auto reference_step = c.update(state, {20, 0, 0}, 0.0025f);
    EXPECT_FLOAT_EQ(reference_step.axis[0].d, 0);
    state.rates_rads.x = 3.01f;
    const float tau = 1 / (2 * M_PI * 10);
    const float beta1 = 0.0025f / (tau + 0.0025f);
    const auto first = c.update(state, {}, 0.0025f);
    const float expected1 = beta1 * (3.01f - 3) / 0.0025f;
    EXPECT_NEAR(first.axis[0].d, -0.1f * expected1, 1.0e-6f);
    const float beta2 = 0.005f / (tau + 0.005f);
    const auto second = c.update(state, {}, 0.005f);
    EXPECT_NEAR(second.axis[0].d, -0.1f * (1 - beta2) * expected1, 1.0e-6f);
}

TEST(UTBRate, InvalidDtStateAndNumbersResetHistory)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{1, 1, 0, 1, 0}, {}, {}};
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (float dt : {
             0.0f, -0.001f, 0.0001f, 0.02f, nan, inf
         }) {
        auto state = zero_state();
        c.update(state, {}, 0.0025f);
        EXPECT_TRUE(c.update(state, {1, 0, 0}, 0.0025f).valid);
        EXPECT_FALSE(c.update(state, {}, dt).valid);
        EXPECT_TRUE(c.update(state, {}, 0.0025f).priming);
        EXPECT_FLOAT_EQ(c.update(state, {}, 0.0025f).axis[0].i, 0);
    }
    for (float number : {
             nan, inf
         }) {
        auto state = zero_state();
        state.rates_rads.x = number;
        EXPECT_FALSE(c.update(state, {}, 0.0025f).valid);
        EXPECT_FALSE(c.update(zero_state(), {number, 0, 0}, 0.0025f).valid);
    }
    auto state = zero_state();
    state.attitude_valid = false;
    EXPECT_FALSE(c.update(state, {}, 0.0025f).valid);
    c.update(zero_state(), {}, 0.0025f);
    auto result = c.update(zero_state(), {}, 0.0025f);
    ASSERT_TRUE(result.valid);
    result.axis[0].raw = nan;
    c.feedback(result, {});
    EXPECT_TRUE(c.update(zero_state(), {}, 0.0025f).priming);
}

TEST(UTBRate, ConfigGuardsAndChangePriming)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{0.1f, 0, 0, 0, 0}, {}, {}};
    auto state = zero_state();
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    c.update(state, {}, 0.0025f);
    EXPECT_TRUE(c.update(state, {}, 0.0025f).valid);
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    EXPECT_TRUE(c.update(state, {}, 0.0025f).valid);
    gains[0].p = 0.2f;
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    EXPECT_TRUE(c.update(state, {}, 0.0025f).priming);
    gains[0].d = 0.1f;
    EXPECT_FALSE(c.configure(gains, 0.0025f));
    gains[0].d_hz = 41;
    EXPECT_FALSE(c.configure(gains, 0.0025f));
    gains[0].d_hz = 40;
    EXPECT_TRUE(c.configure(gains, 0.0025f));
    gains[0].p = -0.1f;
    EXPECT_FALSE(c.configure(gains, 0.0025f));
    gains[0].p = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(c.configure(gains, 0.0025f));
}

TEST(UTBRate, DirectionalAntiWindupBothDirectionsAndOneStepDelay)
{
    for (float sign : {
             -1.0f, 1.0f
         }) {
        AP_UTB_RateController c;
        AP_UTB_MotorMixer mixer;
        AP_UTB_RateController::Gains gains[3] = {{1, 1, 0, 1, 0}, {}, {}};
        ASSERT_TRUE(c.configure(gains, 0.01f));
        auto state = zero_state();
        c.update(state, {}, 0.01f);
        const auto saturating = c.update(state, {2 * sign, 0, 0}, 0.01f);
        EXPECT_FALSE(saturating.axis[0].integration_blocked);
        EXPECT_NEAR(saturating.axis[0].i, 0.02f * sign, 1.0e-6f);
        const auto allocated = mixer.mix(saturating, 0.5f, 1, 12);
        ASSERT_TRUE(allocated.valid);
        c.feedback(saturating, allocated.achieved);
        const auto blocked = c.update(state, {2 * sign, 0, 0}, 0.01f);
        EXPECT_TRUE(blocked.axis[0].integration_blocked);
        EXPECT_FLOAT_EQ(blocked.axis[0].i, saturating.axis[0].i);
        const auto unwinding = c.update(state, {-sign, 0, 0}, 0.01f);
        EXPECT_FALSE(unwinding.axis[0].integration_blocked);
        EXPECT_NEAR(unwinding.axis[0].i, 0.01f * sign, 1.0e-6f);
        c.feedback(unwinding, unwinding.command);
        const auto released = c.update(state, {sign, 0, 0}, 0.01f);
        EXPECT_FALSE(released.axis[0].integration_blocked);
        c.feedback({}, {});
        EXPECT_TRUE(c.update(state, {}, 0.01f).priming);
    }
}

TEST(UTBRate, InclusiveDtGuardsAndFiniteOverflow)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{5, 0, 0, 0, 0}, {}, {}};
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    c.update(zero_state(), {}, 0.0025f);
    EXPECT_TRUE(c.update(zero_state(), {}, 0.000625f).valid);
    EXPECT_TRUE(c.update(zero_state(), {}, 0.01f).valid);
    const auto overflow = c.update(zero_state(), {std::numeric_limits<float>::max(), 0, 0}, 0.0025f);
    EXPECT_FALSE(overflow.valid);
    EXPECT_FALSE(overflow.priming);
    EXPECT_FLOAT_EQ(overflow.command.x, 0);
    EXPECT_TRUE(c.update(zero_state(), {}, 0.0025f).priming);
}

TEST(UTBRate, TinyPositiveIntegralIsNotTreatedAsZero)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{0, 1.0e-7f, 0, 1, 0}, {}, {}};
    ASSERT_TRUE(c.configure(gains, 0.01f));
    c.update(zero_state(), {}, 0.01f);
    EXPECT_GT(c.update(zero_state(), {1, 0, 0}, 0.01f).axis[0].i, 0);
}

TEST(UTBRate, ResetDerivativeHistoryAndRejectFilterOverflow)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{0, 0, 0.1f, 0, 10}, {}, {}};
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    auto state = zero_state();
    c.update(state, {}, 0.0025f);
    state.rates_rads.x = 0.1f;
    EXPECT_LT(c.update(state, {}, 0.0025f).axis[0].d, 0);
    c.reset();
    EXPECT_TRUE(c.update(state, {}, 0.0025f).priming);
    EXPECT_FLOAT_EQ(c.update(state, {}, 0.0025f).axis[0].d, 0);
    gains[0].d_hz = 1.0e-40f;
    ASSERT_TRUE(c.configure(gains, 0.0025f));
    c.update(state, {}, 0.0025f);
    EXPECT_FALSE(c.update(state, {}, 0.0025f).valid);
}

TEST(UTBRate, MixerSaturationFeedbackAndIndependentIntegrators)
{
    AP_UTB_RateController c;
    AP_UTB_RateController::Gains gains[3] = {{0.9f, 1, 0, 1, 0}, {0.9f, 2, 0, 1, 0}, {0.9f, 3, 0, 1, 0}};
    ASSERT_TRUE(c.configure(gains, 0.01f));
    c.update(zero_state(), {}, 0.01f);
    const auto first = c.update(zero_state(), {1, 1, 1}, 0.01f);
    EXPECT_NEAR(first.axis[0].i, 0.01f, 1.0e-6f);
    EXPECT_NEAR(first.axis[1].i, 0.02f, 1.0e-6f);
    EXPECT_NEAR(first.axis[2].i, 0.03f, 1.0e-6f);
    const auto allocated = AP_UTB_MotorMixer().mix(first, 0.5f, 1, 12);
    ASSERT_TRUE(allocated.valid);
    EXPECT_TRUE(allocated.moments_scaled);
    c.feedback(first, allocated.achieved);
    const auto next = c.update(zero_state(), {1, 1, 1}, 0.01f);
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_TRUE(next.axis[i].integration_blocked);
        EXPECT_FLOAT_EQ(next.axis[i].i, first.axis[i].i);
    }
}

static AP_UTB_MotorMixer::Result allocate(float thrust, const Vector3f &command, uint8_t frame_type = 12)
{
    AP_UTB_RateController::Result control;
    control.valid = true;
    control.command = command;
    return AP_UTB_MotorMixer().mix(control, thrust, 1, frame_type);
}

TEST(UTBMixer, ZeroEqualAndSignedAxes)
{
    for (float thrust : {
             0.0f, 0.5f, 1.0f
         }) {
        const auto result = allocate(thrust, {});
        ASSERT_TRUE(result.valid);
        for (float motor : result.thrust) {
            EXPECT_FLOAT_EQ(motor, thrust);
        }
        EXPECT_FLOAT_EQ(result.achieved_thrust, thrust);
    }
    const float factors[3][4] = {{-0.5f, -0.5f, 0.5f, 0.5f},
        {-0.5f, 0.5f, -0.5f, 0.5f},
        {-0.5f, 0.5f, 0.5f, -0.5f}
    };
    for (uint8_t axis = 0; axis < 3; axis++) {
        for (float sign : {
                 -1.0f, 1.0f
             }) {
            Vector3f command;
            command[axis] = 0.2f * sign;
            const auto result = allocate(0.5f, command);
            ASSERT_TRUE(result.valid);
            for (uint8_t motor = 0; motor < 4; motor++) {
                EXPECT_NEAR(result.thrust[motor], 0.5f + factors[axis][motor] * command[axis], 1.0e-6f);
            }
            EXPECT_NEAR(result.achieved[axis], command[axis], 1.0e-6f);
        }
    }
}

TEST(UTBMixer, AllocationPriorityAndMasks)
{
    auto result = allocate(0.95f, {0.4f, 0, 0});
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.achieved_thrust, 0.8f, 1.0e-6f);
    EXPECT_NEAR(result.achieved.x, 0.4f, 1.0e-6f);
    EXPECT_EQ(result.positive_limits, 8);
    EXPECT_EQ(result.upper_mask, 12);
    result = allocate(0.05f, {0.4f, 0, 0});
    EXPECT_NEAR(result.achieved_thrust, 0.2f, 1.0e-6f);
    EXPECT_EQ(result.negative_limits, 8);
    EXPECT_EQ(result.lower_mask, 3);
    result = allocate(0.5f, {1, 1, 1});
    ASSERT_TRUE(result.valid);
    EXPECT_FLOAT_EQ(result.scale, 0.5f);
    EXPECT_FLOAT_EQ(result.achieved_thrust, 0.75f);
    EXPECT_FLOAT_EQ(result.thrust[0], 0);
    for (uint8_t i = 1; i < 4; i++) {
        EXPECT_FLOAT_EQ(result.thrust[i], 1);
    }
    EXPECT_TRUE(result.moments_scaled);
    EXPECT_TRUE(result.collective_shifted);
    EXPECT_EQ(result.positive_limits, 7);
    EXPECT_EQ(result.negative_limits, 8);
}

TEST(UTBMixer, RoundtripBoundsAndEqualYawPriority)
{
    for (int roll = -10; roll <= 10; roll++) {
        for (int pitch = -10; pitch <= 10; pitch++) {
            for (int yaw = -10; yaw <= 10; yaw++) {
                const Vector3f command(roll * 0.1f, pitch * 0.1f, yaw * 0.1f);
                const float thrust = (roll + 10) * 0.05f;
                const auto result = allocate(thrust, command);
                ASSERT_TRUE(result.valid);
                EXPECT_GT(result.scale, 0);
                EXPECT_LE(result.scale, 1);
                for (float motor : result.thrust) {
                    EXPECT_GE(motor, 0);
                    EXPECT_LE(motor, 1);
                }
                for (uint8_t j = 0; j < 3; j++) {
                    EXPECT_NEAR(result.achieved[j], result.scale * command[j], 1.0e-5f);
                }
            }
        }
    }
}

TEST(UTBMixer, InvalidFrameDomainAndNumbersAreFiniteZeros)
{
    AP_UTB_RateController::Result control;
    control.valid = true;
    control.command = {0.1f, 0.2f, 0.3f};
    AP_UTB_MotorMixer mixer;
    for (auto frame : {
             std::pair<uint8_t, uint8_t>(0, 12), {0, 18}, {1, 0}, {1, 1}, {2, 12}, {2, 18}
         }) {
        const auto result = mixer.mix(control, 0.5f, frame.first, frame.second);
        EXPECT_FALSE(result.valid);
        for (float motor : result.thrust) {
            EXPECT_FLOAT_EQ(motor, 0);
        }
    }
    for (float number : {
             -0.1f, 1.1f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()
         }) {
        EXPECT_FALSE(mixer.mix(control, number, 1, 12).valid);
        control.command.x = number;
        if (!isfinite(number) || number > 1) {
            EXPECT_FALSE(mixer.mix(control, 0.5f, 1, 12).valid);
        }
    }
    control.valid = false;
    EXPECT_FALSE(mixer.mix(control, 0.5f, 1, 12).valid);
}
TEST(UTBReference, CircularMapAndAPExpoBoundaries)
{
    Vector3f ref;
    ASSERT_TRUE(AP_UTB_Reference::rates({1, 1, -1}, 180, 90, 0, 0, ref));
    EXPECT_NEAR(ref.x, M_PI / sqrtf(2), 1.0e-6f);
    EXPECT_NEAR(ref.y, M_PI / sqrtf(2), 1.0e-6f);
    EXPECT_NEAR(ref.z, -M_PI * 0.5f, 1.0e-6f);
    for (float expo : {
             -0.5f, 0.0f, 0.5f, nextafterf(0.95f, 0), 0.95f
         }) {
        ASSERT_TRUE(AP_UTB_Reference::rates({0.5f, -0.5f, 0.5f}, 180, 90, expo, expo, ref));
        EXPECT_NEAR(ref.x, M_PI * input_expo(0.5f, expo), 1.0e-6f);
        EXPECT_NEAR(ref.y, -ref.x, 1.0e-6f);
        EXPECT_NEAR(ref.z, ref.x * 0.5f, 1.0e-6f);
    }
    EXPECT_FALSE(AP_UTB_Reference::rates({}, 0, 90, 0, 0, ref));
    EXPECT_FALSE(AP_UTB_Reference::rates({1.1f, 0, 0}, 180, 90, 0, 0, ref));
    EXPECT_FALSE(AP_UTB_Reference::rates({}, 180, 90, 1, 0, ref));
    EXPECT_FALSE(AP_UTB_Reference::rates({}, 180, 90, 0, std::numeric_limits<float>::quiet_NaN(), ref));
    EXPECT_FLOAT_EQ(ref.length(), 0);
}

static AP_UTB::Inputs engine_input()
{
    AP_UTB::Inputs in;
    in.state = zero_state();
    in.time_us = 1000;
    in.dt = in.nominal_dt = 0.0025f;
    in.thrust = 0.5f;
    in.frame_class = 1;
    in.frame_type = 12;
    in.reference_valid = true;
    in.logging_available = true;
    return in;
}

static void advance(AP_UTB::Inputs &in)
{
    in.time_us += 2500;
    in.state.sampled_us = in.time_us;
    in.state.imu_updated_us = in.time_us;
}

TEST(UTBHealth, LoggingStartPolicyAndRuntimeLossAreSeparateFromMathematics)
{
    AP_UTB utb;
    AP_UTB::Snapshot snapshot;
    auto in = engine_input();
    AP_UTB_RateController::Gains gains[3] = {{0, 1, 0, 1, 0}, {}, {}};
    in.reference.x = 1;
    in.logging_available = false;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.policy, AP_UTB::PolicyReason::EXPERIMENT_SUPPRESSED);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::NONE);
    EXPECT_FALSE(snapshot.flags & AP_UTB::CONTROLLER_EVALUATED);
    EXPECT_FALSE(snapshot.flags & AP_UTB::MIXER_EVALUATED);
    advance(in);
    in.logging_available = true;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::PRIMING);
    advance(in);
    utb.evaluate(in, gains, snapshot);
    ASSERT_TRUE(snapshot.flags & AP_UTB::SHADOW_HEALTHY);
    const float previous_i = snapshot.control.axis[0].i;
    advance(in);
    in.logging_available = false;
    utb.evaluate(in, gains, snapshot);
    EXPECT_TRUE(snapshot.flags & AP_UTB::SHADOW_HEALTHY);
    EXPECT_FALSE(snapshot.flags & AP_UTB::SHADOW_OBSERVABLE);
    EXPECT_EQ(snapshot.observation, AP_UTB::ObservationReason::LOGGING_UNAVAILABLE);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::NONE);
    EXPECT_GT(snapshot.control.axis[0].i, previous_i);
}

TEST(UTBHealth, FrameFastRateStaleAndModeResetRecovery)
{
    AP_UTB utb;
    AP_UTB::Snapshot snapshot;
    auto in = engine_input();
    AP_UTB_RateController::Gains gains[3] {};
    for (uint8_t type : {
             uint8_t(0), uint8_t(1)
         }) {
        in.frame_type = type;
        utb.evaluate(in, gains, snapshot);
        EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::FRAME_MISMATCH);
        EXPECT_TRUE(snapshot.flags & AP_UTB::SHADOW_OBSERVABLE);
        EXPECT_FALSE(snapshot.flags & AP_UTB::MIXER_VALID);
        EXPECT_FALSE(snapshot.flags & AP_UTB::MIXER_EVALUATED);
        advance(in);
    }
    in.frame_type = 12;
    in.frame_class = 2;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::FRAME_MISMATCH);
    advance(in);
    in.frame_class = 1;
    in.fast_rate = true;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::FAST_RATE_UNSUPPORTED);
    advance(in);
    in.fast_rate = false;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::PRIMING);
    advance(in);
    utb.evaluate(in, gains, snapshot);
    EXPECT_TRUE(snapshot.flags & AP_UTB::SHADOW_HEALTHY);
    utb.evaluate(in, gains, snapshot); // duplicate INS update
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::STATE_INVALID);
    advance(in);
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::PRIMING);
    advance(in);
    in.mode = 1;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::PRIMING);
    in.mode = 2;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::STATE_INVALID);
    advance(in);
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::PRIMING);
}

TEST(UTBHealth, DtFailureDoesNotMakeFreshStateInvalid)
{
    AP_UTB utb;
    AP_UTB::Snapshot snapshot;
    auto in = engine_input();
    AP_UTB_RateController::Gains gains[3] {};
    in.acro_available = false;
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::ACRO_UNAVAILABLE);
    advance(in);
    in.acro_available = true;
    in.dt = 0;
    utb.evaluate(in, gains, snapshot);
    EXPECT_TRUE(snapshot.flags & AP_UTB::STATE_VALID);
    EXPECT_FALSE(snapshot.flags & AP_UTB::CONTROLLER_VALID);
    EXPECT_TRUE(snapshot.flags & AP_UTB::SHADOW_OBSERVABLE);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::DT_INVALID);
}

TEST(UTBHealth, ConsecutiveExecutionBudgetLatchAndReset)
{
    AP_UTB utb;
    AP_UTB::Snapshot snapshot;
    auto in = engine_input();
    AP_UTB_RateController::Gains gains[3] {};
    // Synthetic durations test the policy; they are not hardware timing measurements.
    utb.record_execution(126, 0.0025f);
    utb.record_execution(100, 0.0025f);
    utb.record_execution(126, 0.0025f);
    utb.record_execution(126, 0.0025f);
    EXPECT_FALSE(utb.time_budget_latched());
    utb.record_execution(126, 0.0025f);
    EXPECT_TRUE(utb.time_budget_latched());
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.policy, AP_UTB::PolicyReason::TIME_BUDGET);
    EXPECT_FALSE(snapshot.flags & AP_UTB::CONTROLLER_EVALUATED);
    utb.reset_shadow();
    EXPECT_FALSE(utb.time_budget_latched());
    advance(in);
    utb.evaluate(in, gains, snapshot);
    EXPECT_EQ(snapshot.calculation, AP_UTB::CalculationReason::PRIMING);
}

TEST(UTBQueue, BoundedOverflowAndFifo)
{
    RecordProperty("SnapshotBytes", int(sizeof(AP_UTB::Snapshot)));
    RecordProperty("QueueBytes", int(sizeof(AP_UTB::Queue)));
    AP_UTB::Queue queue;
    AP_UTB::Snapshot s;
    for (uint8_t i = 0; i < AP_UTB::Queue::CAPACITY; i++) {
        s.seq = i;
        EXPECT_TRUE(queue.push(s));
    }
    EXPECT_FALSE(queue.push(s));
    EXPECT_EQ(queue.size(), AP_UTB::Queue::CAPACITY);
    for (uint8_t i = 0; i < AP_UTB::Queue::CAPACITY; i++) {
        ASSERT_TRUE(queue.pop(s));
        EXPECT_EQ(s.seq, i);
    }
    EXPECT_FALSE(queue.pop(s));
    EXPECT_TRUE(queue.push(s));
    EXPECT_TRUE(queue.pop(s));
    EXPECT_FALSE(queue.pop(s));
}

// Denying try-lock proves the queue never falls back to a waiting semaphore API.
class UTBDeniedSemaphore : public AP_HAL::Semaphore
{
public:
    uint32_t attempts = 0;
    uint32_t blocking = 0;
    bool allow = false;
    bool take(uint32_t) override
    {
        blocking++;
        return false;
    }
    bool take_nonblocking() override
    {
        attempts++;
        return allow;
    }
    bool give() override
    {
        return true;
    }
};

TEST(UTBQueue, ContentionNeverCallsBlockingTake)
{
    UTBDeniedSemaphore sem;
    AP_UTB::Queue queue(&sem);
    AP_UTB::Snapshot sample;
    EXPECT_TRUE(queue.push(sample));
    EXPECT_TRUE(queue.pop(sample));
    EXPECT_FALSE(queue.read_status(sample));
    EXPECT_EQ(sem.blocking, 0U);
    EXPECT_EQ(sem.attempts, 2U);
    sem.allow = true;
    EXPECT_TRUE(queue.push(sample));
    EXPECT_TRUE(queue.pop(sample));
    EXPECT_EQ(sem.blocking, 0U);
}

TEST(UTBHealth, FullQueueDoesNotResetController)
{
    AP_UTB utb;
    AP_UTB::Snapshot sample;
    auto in = engine_input();
    AP_UTB_RateController::Gains gains[3] = {{0, 1, 0, 1, 0}, {}, {}};
    in.reference.x = 1;
    utb.evaluate(in, gains, sample);
    float previous_i = 0;
    for (uint8_t n = 0; n < 10; n++) {
        advance(in);
        utb.evaluate(in, gains, sample);
        ASSERT_TRUE(sample.control.valid);
        ASSERT_TRUE(sample.mixer.valid);
        EXPECT_GT(sample.control.axis[0].i, previous_i);
        previous_i = sample.control.axis[0].i;
        utb.publish_diagnostics(sample, true);
    }
    EXPECT_EQ(utb.pipeline().dropped, 6U);
    EXPECT_EQ(utb.pipeline().requested, 10U);
    EXPECT_EQ(utb.pipeline().enqueued, 4U);
    EXPECT_EQ(utb.pipeline().high_water, 4U);
}

TEST(UTBHealth, BackendFailureOnlyChangesObservation)
{
    AP_UTB utb;
    AP_UTB::Snapshot sample;
    auto in = engine_input();
    AP_UTB_RateController::Gains gains[3] = {{0, 1, 0, 1, 0}, {}, {}};
    in.reference.x = 1;
    utb.evaluate(in, gains, sample);
    advance(in);
    utb.evaluate(in, gains, sample);
    const float previous_i = sample.control.axis[0].i;
    utb.log_result(false);
    advance(in);
    utb.evaluate(in, gains, sample);
    EXPECT_TRUE(sample.control.valid);
    EXPECT_TRUE(sample.mixer.valid);
    EXPECT_GT(sample.control.axis[0].i, previous_i);
    EXPECT_FALSE(sample.flags & AP_UTB::SHADOW_OBSERVABLE);
    EXPECT_EQ(sample.observation, AP_UTB::ObservationReason::WRITE_FAILED);
    EXPECT_EQ(utb.pipeline().missing, 1U);
    utb.log_result(true);
    advance(in);
    utb.evaluate(in, gains, sample);
    EXPECT_TRUE(sample.flags & AP_UTB::SHADOW_OBSERVABLE);
    utb.log_result(false);
    utb.log_result(true); // failure/recovery between producer cycles must still be observed
    advance(in);
    utb.evaluate(in, gains, sample);
    EXPECT_TRUE(sample.control.valid);
    EXPECT_FALSE(sample.flags & AP_UTB::SHADOW_OBSERVABLE);
    EXPECT_EQ(sample.observation, AP_UTB::ObservationReason::WRITE_FAILED);
    advance(in);
    utb.evaluate(in, gains, sample);
    EXPECT_TRUE(sample.flags & AP_UTB::SHADOW_OBSERVABLE);
}

TEST(UTBHealth, PrimaryGyroChangePrimesAllHistory)
{
    AP_UTB utb;
    AP_UTB::Snapshot sample;
    auto in = engine_input();
    in.state.primary_gyro = 0;
    in.reference.x = 0.2f;
    AP_UTB_RateController::Gains gains[3] = {{0.1f, 1, 0.001f, 1, 20}, {}, {}};
    utb.evaluate(in, gains, sample);
    advance(in);
    utb.evaluate(in, gains, sample);
    ASSERT_TRUE(sample.control.valid);
    EXPECT_GT(sample.control.axis[0].i, 0);
    advance(in);
    in.state.primary_gyro = 1;
    in.state.rates_rads.x = 100;
    utb.evaluate(in, gains, sample);
    EXPECT_EQ(sample.calculation, AP_UTB::CalculationReason::PRIMARY_GYRO_CHANGED);
    EXPECT_FALSE(sample.control.valid);
    EXPECT_FALSE(sample.mixer.valid);
    advance(in);
    utb.evaluate(in, gains, sample);
    ASSERT_TRUE(sample.control.valid);
    EXPECT_FLOAT_EQ(sample.control.axis[0].d, 0);
    EXPECT_NEAR(sample.control.axis[0].i, (0.2f - 100) * 0.0025f, 1.0e-6f);
    advance(in);
    utb.evaluate(in, gains, sample);
    EXPECT_TRUE(sample.control.valid); // unchanged index does not prime again
    EXPECT_EQ(sample.calculation, AP_UTB::CalculationReason::NONE);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
TEST(UTBQueue, ConcurrentCopiesRemainCoherent)
{
    AP_UTB::Queue queue;
    std::atomic<bool> done{false};
    std::atomic<uint32_t> accepted{0};
    std::atomic<uint32_t> received{0};
    std::atomic<uint32_t> corrupted{0};
    std::thread consumer([&]() {
        AP_UTB::Snapshot value;
        while (!done.load() || queue.size() != 0) {
            if (queue.pop(value)) {
                if (value.input.time_us != uint64_t(value.seq) * 2500 || value.epoch != value.seq) {
                    ++corrupted;
                }
                ++received;
            } else {
                std::this_thread::yield();
            }
        }
    });
    for (uint32_t n = 1; n <= 20000; n++) {
        AP_UTB::Snapshot value;
        value.seq = n;
        value.epoch = n;
        value.input.time_us = uint64_t(n) * 2500;
        if (queue.push(value)) {
            ++accepted;
        }
    }
    done.store(true);
    consumer.join();
    EXPECT_EQ(received.load(), accepted.load());
    EXPECT_EQ(corrupted.load(), 0U);
    EXPECT_LE(queue.high_water(), 4U);
}
#endif

// Verify the actual API under ArduPilot build flags, including single-precision constants.
TEST(UTBSourceConvention, ActualAPExpoAtParameterBoundary)
{
    EXPECT_DOUBLE_EQ(double(0.95f), 0.94999998807907104);
    EXPECT_FLOAT_EQ(input_expo(0.5f, 0.95f), 0.5f);
    EXPECT_FLOAT_EQ(input_expo(0.5f, nextafterf(0.95f, 1.0f)), 0.5f);
}
#endif // AP_UTB_ENABLED


#if AP_UTB_ENABLED
TEST(UTBMixer, BothGeometriesSignedAxesAndInverse)
{
    // Independent literal columns from AP MotorDef + per-axis normalization.
    const float columns[2][3][4] = {
        {{-0.5f, -0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f, -0.5f}},
        {{-0.5f, -0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, -0.5f, 0.5f}}
    };
    for (uint8_t geometry = 0; geometry < 2; geometry++) {
        const uint8_t type = geometry == 0 ? 12 : 18;
        ASSERT_TRUE(AP_UTB_MotorMixer::supports_frame(1, type));
        for (float thrust : {
                 0.0f, 0.5f, 1.0f
             }) {
            const auto zero = allocate(thrust, {}, type);
            ASSERT_TRUE(zero.valid);
            for (float motor : zero.thrust) {
                EXPECT_FLOAT_EQ(motor, thrust);
            }
        }
        for (uint8_t axis = 0; axis < 3; axis++) {
            for (float sign : {
                     -1.0f, 1.0f
                 }) {
                Vector3f command;
                command[axis] = sign * 0.2f;
                const auto result = allocate(0.5f, command, type);
                ASSERT_TRUE(result.valid);
                for (uint8_t m = 0; m < 4; m++) {
                    EXPECT_NEAR(result.thrust[m], 0.5f + columns[geometry][axis][m] * command[axis], 1.0e-6f);
                }
                for (uint8_t j = 0; j < 3; j++) {
                    EXPECT_NEAR(result.achieved[j], command[j], 1.0e-6f);
                }
            }
        }
        const auto combined = allocate(0.5f, {0.1f, -0.2f, 0.3f}, type);
        ASSERT_TRUE(combined.valid);
        const float expected[2][4] = {{0.4f, 0.5f, 0.8f, 0.3f}, {0.7f, 0.2f, 0.5f, 0.6f}};
        for (uint8_t m = 0; m < 4; m++) {
            EXPECT_NEAR(combined.thrust[m], expected[geometry][m], 1.0e-6f);
        }
    }
    for (Vector3f command : {
             Vector3f(0.2f, 0, 0), Vector3f(0, -0.2f, 0)
         }) {
        const auto a = allocate(0.5f, command, 12);
        const auto b = allocate(0.5f, command, 18);
        for (uint8_t m = 0; m < 4; m++) {
            EXPECT_FLOAT_EQ(a.thrust[m], b.thrust[m]);
        }
    }
    const auto a = allocate(0.5f, {0, 0, 0.2f}, 12);
    const auto b = allocate(0.5f, {0, 0, 0.2f}, 18);
    for (uint8_t m = 0; m < 4; m++) {
        EXPECT_NEAR(a.thrust[m] + b.thrust[m], 1, 1.0e-6f);
    }
}

TEST(UTBMixer, ReversedYawSaturationAndFiniteDomain)
{
    const auto scaled = allocate(0.5f, {1, 1, 1}, 18);
    ASSERT_TRUE(scaled.valid);
    EXPECT_FLOAT_EQ(scaled.scale, 0.5f);
    const float expected[4] = {0, 0, 0, 1};
    for (uint8_t m = 0; m < 4; m++) {
        EXPECT_FLOAT_EQ(scaled.thrust[m], expected[m]);
    }
    EXPECT_FLOAT_EQ(scaled.achieved_thrust, 0.25f);
    EXPECT_FLOAT_EQ(scaled.collective_shift, -0.25f);
    for (uint8_t axis = 0; axis < 3; axis++) {
        EXPECT_FLOAT_EQ(scaled.achieved[axis], 0.5f);
    }
    EXPECT_EQ(scaled.positive_limits, 15);
    EXPECT_EQ(scaled.lower_mask, 7);
    EXPECT_EQ(scaled.upper_mask, 8);
    for (uint8_t type : {
             12, 18
         }) {
        for (int roll = -10; roll <= 10; roll++) {
            for (int pitch = -10; pitch <= 10; pitch++) {
                for (int yaw = -10; yaw <= 10; yaw++) {
                    const Vector3f command(roll * 0.1f, pitch * 0.1f, yaw * 0.1f);
                    const auto result = allocate((roll + 10) * 0.05f, command, type);
                    ASSERT_TRUE(result.valid);
                    for (float motor : result.thrust) {
                        EXPECT_GE(motor, 0);
                        EXPECT_LE(motor, 1);
                    }
                    for (uint8_t j = 0; j < 3; j++) {
                        EXPECT_NEAR(result.achieved[j], result.scale * command[j], 1.0e-5f);
                    }
                }
            }
        }
        for (uint8_t axis = 0; axis < 3; axis++) {
            for (float invalid : {
                     NAN, INFINITY, -INFINITY, 1.1f, -1.1f
                 }) {
                Vector3f command;
                command[axis] = invalid;
                EXPECT_FALSE(allocate(0.5f, command, type).valid);
                EXPECT_FALSE(allocate(invalid, {}, type).valid);
            }
        }
    }
}

TEST(UTBRate, BothGeometriesDirectionalYawAntiWindup)
{
    for (uint8_t type : {
             12, 18
         }) {
        for (float sign : {
                 -1.0f, 1.0f
             }) {
            AP_UTB_RateController controller;
            AP_UTB_RateController::Gains gains[3] = {{0.9f, 1, 0, 1, 0}, {0.9f, 1, 0, 1, 0}, {0.9f, 1, 0, 1, 0}};
            ASSERT_TRUE(controller.configure(gains, 0.01f));
            controller.update(zero_state(), {}, 0.01f);
            const Vector3f demand(1, 1, sign);
            const auto first = controller.update(zero_state(), demand, 0.01f);
            const auto mixed = AP_UTB_MotorMixer().mix(first, 0.5f, 1, type);
            ASSERT_TRUE(mixed.valid);
            EXPECT_NEAR(mixed.achieved.z, sign * 0.5f, 1.0e-6f);
            controller.feedback(first, mixed.achieved);
            const auto blocked = controller.update(zero_state(), demand, 0.01f);
            EXPECT_TRUE(blocked.axis[2].integration_blocked);
            EXPECT_FLOAT_EQ(blocked.axis[2].i, first.axis[2].i);
            const auto unwind = controller.update(zero_state(), {1, 1, -sign}, 0.01f);
            EXPECT_FALSE(unwind.axis[2].integration_blocked);
            EXPECT_NEAR(unwind.axis[2].i, 0, 1.0e-6f);
        }
    }
}

TEST(UTBHealth, GeometryTransitionResetsHistoryAndDiagnosticEpoch)
{
    AP_UTB utb;
    AP_UTB::Snapshot sample;
    auto in = engine_input();
    in.reference = {1, 1, 1};
    AP_UTB_RateController::Gains gains[3] = {{0.9f, 1, 0.001f, 1, 20}, {0.9f, 1, 0.001f, 1, 20}, {0.9f, 1, 0.001f, 1, 20}};
    utb.evaluate(in, gains, sample);
    advance(in);
    utb.evaluate(in, gains, sample);
    ASSERT_TRUE(sample.mixer.moments_scaled);
    ASSERT_TRUE(utb.publish_diagnostics(sample, true));
    uint32_t previous_epoch = sample.epoch;
    for (uint8_t type : {
             18, 12, 18
         }) {
        advance(in);
        in.frame_type = type;
        in.state.rates_rads = {0.1f, 0.1f, 0.1f};
        utb.evaluate(in, gains, sample);
        EXPECT_EQ(sample.calculation, AP_UTB::CalculationReason::PRIMING);
        EXPECT_FALSE(sample.control.valid);
        EXPECT_FALSE(sample.mixer.valid);
        EXPECT_EQ(sample.epoch, previous_epoch + 1);
        if (previous_epoch == 0) {
            AP_UTB::Snapshot old;
            ASSERT_TRUE(utb.pop_log(old));
            EXPECT_NE(old.epoch, utb.epoch());
        }
        previous_epoch = sample.epoch;
        advance(in);
        utb.evaluate(in, gains, sample);
        ASSERT_TRUE(sample.flags & AP_UTB::SHADOW_HEALTHY);
        for (uint8_t axis = 0; axis < 3; axis++) {
            EXPECT_FALSE(sample.control.axis[axis].integration_blocked);
            EXPECT_NEAR(sample.control.axis[axis].i, 0.9f * in.dt, 1.0e-6f);
            EXPECT_FLOAT_EQ(sample.control.axis[axis].d, 0);
        }
    }
    utb.record_execution(126, 0.0025f);
    utb.record_execution(126, 0.0025f);
    utb.record_execution(126, 0.0025f);
    advance(in);
    in.frame_type = 12;
    utb.evaluate(in, gains, sample);
    EXPECT_EQ(sample.policy, AP_UTB::PolicyReason::TIME_BUDGET);
}
#endif // AP_UTB_ENABLED

AP_GTEST_MAIN()
