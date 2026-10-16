/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
 */
#include <AP_gtest.h>
#include <AP_UTB/AP_UTB_SITLRate.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
#if AP_UTB_RATE_SITL_AVAILABLE
static SRV_Channels channels;

// Invoke the real allocator only. No output(), output_to_motors() or HAL writes.
class MatrixProbe : public AP_MotorsMatrix
{
public:
    MatrixProbe() : AP_MotorsMatrix(400)
    {
        setup_motors(MOTOR_FRAME_QUAD, MOTOR_FRAME_TYPE_BF_X_REV);
        set_dt_s(.0025f);
    }
    AP_UTB_SITLAllocation::Result allocate(const Vector3f &raw, uint32_t seq, uint32_t stamp,
                                           float throttle = .3f, float average = .3f, float headroom = .2f,
                                           float ceiling = 1)
    {
        AP_UTB_SITLAllocation::Capture c;
        c.raw = raw; c.pid_valid = true; c.output_seq = c.command_seq = seq; c.sampled_us = stamp;
        c.armed = c.full_spool = c.factors_available = true;
        _throttle_filter.reset(throttle); set_throttle_avg_max(average);
        _yaw_headroom.set(int16_t(headroom * 1000)); _throttle_thrust_max = ceiling;
        _thrust_boost = false; _thrust_boost_ratio = 0;
        limit.set_all(false);
        for (uint8_t a = 0; a < 3; a++) {
            c.command[a] = constrain_float(raw[a], -1, 1);
        }
        set_roll(c.command.x); set_pitch(c.command.y); set_yaw(c.command.z);
        set_roll_ff(0); set_pitch_ff(0); set_yaw_ff(0);
        output_armed_stabilizing();
        c.gain = thr_lin.get_compensation_gain();
        c.limits = limit.roll | (limit.pitch << 1) | (limit.yaw << 2) |
                   (limit.throttle_lower << 3) | (limit.throttle_upper << 4);
        for (uint8_t m = 0; m < 4; m++) {
            uint8_t order;
            c.factors_available &= get_factors(m, c.factors[m].x, c.factors[m].y, c.factors[m].z, c.collective[m], order);
            c.thrust[m] = get_thrust_rpyt_out(m);
            // Unit fixture has an ideal post-allocation stage; actual vehicle captures its inverse linearization.
            c.post_thrust[m] = c.thrust[m] / c.gain;
        }
        return AP_UTB_SITLAllocation::reconstruct(c);
    }
};
static MatrixProbe matrix;

static AP_UTB_SITLRate::Inputs input(uint32_t seq, float jitter = 0)
{
    AP_UTB_SITLRate::Inputs in;
    in.seq = seq; in.now_us = 100000 + seq * 2500 + 10;
    in.state.sampled_us = in.state.imu_updated_us = in.now_us - 10;
    in.state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
    in.state.attitude_valid = in.state.rates_valid = true; in.state.primary_gyro = 0;
    in.dt = .0025f + jitter; in.nominal_dt = .0025f;
    in.frame_class = 1; in.frame_type = 18; in.armed = in.full_spool = true;
    return in;
}
static const AP_UTB_RateController::Gains gains[3] = {{3, 1, 0, .6f, 20}, {3, 1, 0, .6f, 20}, {3, 1, 0, .6f, 20}};

TEST(UTBAPAllocation, ActualLinearAllocationAndYawHeadroom)
{
    auto f = matrix.allocate({.1f, -.2f, .3f}, 1, 102500, .5f, .5f);
    ASSERT_TRUE(f.valid); EXPECT_LT((f.allocated - f.capture.raw).length(), 1.0e-6f);
    f = matrix.allocate({.95f, .95f, .9f}, 2, 105000, .5f, .5f);
    ASSERT_TRUE(f.valid); EXPECT_TRUE(f.capture.limits & 4); EXPECT_GT(f.residual.z, .1f);
    EXPECT_GT(f.residual.x, .1f); EXPECT_GT(f.residual.y, .1f);
}
TEST(UTBAPAllocation, CollectiveOnlyLimitIsNotInventedMomentLoss)
{
    const auto f = matrix.allocate({}, 1, 102500, 1, 1, .2f, .8f);
    ASSERT_TRUE(f.valid); EXPECT_TRUE(f.capture.limits & 16); EXPECT_LT(f.allocated.length(), 1.0e-6f);
}
TEST(UTBAPAllocation, LongSaturationBlocksGrowthAndSignReversalUnwinds)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains, true));
    float held = 0;
    for (uint32_t seq = 1; seq <= 10002; seq++) {
        auto in = input(seq); in.target.z = .3f;
        const auto result = rate.step(in, true);
        ASSERT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::NONE);
        Vector3f raw(result.axis[0].raw, result.axis[1].raw, result.axis[2].raw);
        auto f = matrix.allocate(raw, seq, in.now_us); f.capture.pid_valid = result.valid;
        rate.capture_allocation(f);
        if (seq == 2) {
            held = result.axis[2].i;
        }
        if (seq > 2) {
            EXPECT_TRUE(result.axis[2].integration_blocked);
            EXPECT_FLOAT_EQ(result.axis[2].i, held);
        }
    }
    auto in = input(10003); in.target.z = -.05f;
    auto result = rate.step(in, true);
    EXPECT_FALSE(result.axis[2].integration_blocked); EXPECT_LT(result.axis[2].i, held);
    rate.capture_allocation(matrix.allocate({0, 0, result.axis[2].raw}, 10003, in.now_us));
    in = input(10004); in.target.z = -.05f; result = rate.step(in, true);
    EXPECT_FALSE(result.axis[2].integration_blocked);
}
TEST(UTBAPAllocation, ControllerClippingIsIncludedAndReverseCanUnwind)
{
    AP_UTB_RateController::Gains clipped[3] = {{5, 1, 0, .6f, 20}, {5, 1, 0, .6f, 20}, {5, 1, 0, .6f, 20}};
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(clipped, true));
    float previous_i = 0;
    for (uint32_t seq = 1; seq <= 4; seq++) {
        auto in = input(seq); in.target.z = seq < 4 ? .3f : -.3f;
        const auto result = rate.step(in, true);
        ASSERT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::NONE);
        if (seq == 2) {
            EXPECT_GT(result.axis[2].raw, 1); EXPECT_FLOAT_EQ(result.command.z, 1);
        }
        if (seq == 3) {
            EXPECT_TRUE(result.axis[2].integration_blocked); EXPECT_FLOAT_EQ(result.axis[2].i, previous_i);
        }
        if (seq == 4) {
            EXPECT_FALSE(result.axis[2].integration_blocked); EXPECT_LT(result.axis[2].i, previous_i);
            EXPECT_FLOAT_EQ(result.command.z, -1);
        }
        const Vector3f raw(result.axis[0].raw, result.axis[1].raw, result.axis[2].raw);
        auto f = matrix.allocate(raw, seq, in.now_us); f.capture.pid_valid = result.valid;
        ASSERT_TRUE(f.valid);
        if (seq == 2) {
            EXPECT_GT(f.residual.z, .5f);
        }
        rate.capture_allocation(f); previous_i = result.axis[2].i;
    }
}
TEST(UTBAPAllocation, SimultaneousRollPitchAndShortSaturationRecovery)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains, true));
    bool blocked = false, recovered = false;
    for (uint32_t seq = 1; seq <= 60; seq++) {
        auto in = input(seq); in.target = seq < 25 ? Vector3f(.3f, .3f, 0) : Vector3f(.02f, .02f, 0);
        auto result = rate.step(in, true);
        ASSERT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::NONE);
        if (seq > 2 && seq < 25) {
            blocked |= result.axis[0].integration_blocked && result.axis[1].integration_blocked;
        }
        if (seq > 26) {
            recovered |= !result.axis[0].integration_blocked && !result.axis[1].integration_blocked;
        }
        Vector3f raw(result.axis[0].raw, result.axis[1].raw, result.axis[2].raw);
        auto f = matrix.allocate(raw, seq, in.now_us); f.capture.pid_valid = result.valid;
        rate.capture_allocation(f);
    }
    EXPECT_TRUE(blocked); EXPECT_TRUE(recovered);
}
TEST(UTBAPAllocation, UnknownBoostSlewNonfiniteAndWrongBasisRejected)
{
    const auto good = matrix.allocate({.1f, .1f, .1f}, 1, 102500);
    ASSERT_TRUE(good.valid);
    for (uint8_t fault = 0; fault < 5; fault++) {
        auto c = good.capture;
        switch (fault) {
        case 0: c.boost = true; break;
        case 1: c.post_thrust[0] += .01f; break;
        case 2: c.thrust[0] = NAN; break;
        case 3: c.factors[0].z *= -1; break;
        case 4: c.gain = 0; break;
        }
        EXPECT_FALSE(AP_UTB_SITLAllocation::reconstruct(c).valid);
    }
}
TEST(UTBAPAllocation, StaleOrMisassignedFeedbackFaultsWithoutNextPID)
{
    for (uint8_t fault = 0; fault < 3; fault++) {
        AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains, true));
        EXPECT_TRUE(rate.step(input(1), true).priming);
        auto f = matrix.allocate({}, 1, 102510);
        if (fault == 0) {
            f.valid = false;
        }
        if (fault == 1) {
            f.capture.output_seq = 0;
        }
        if (fault == 2) {
            f.capture.sampled_us -= 10000;
        }
        rate.capture_allocation(f);
        EXPECT_FALSE(rate.step(input(2), true).valid);
        EXPECT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::FEEDBACK); EXPECT_EQ(rate.calls(), 1U);
        EXPECT_FALSE(rate.step(input(3), true).valid);
        ASSERT_TRUE(rate.configure(gains, true)); EXPECT_TRUE(rate.step(input(4), true).priming);
    }
}
TEST(UTBAPAllocation, ExplicitDelayedFeedbackAndJitterRemainFinite)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains, true, true));
    AP_UTB_RateController::Result previous;
    for (uint32_t seq = 1; seq <= 80; seq++) {
        auto in = input(seq, seq % 2 ? .0005f : -.0005f); in.target.z = .15f;
        auto result = rate.step(in, true); ASSERT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::NONE);
        Vector3f raw(previous.axis[0].raw, previous.axis[1].raw, previous.axis[2].raw);
        auto f = matrix.allocate(raw, seq, in.now_us); f.capture.command_seq = seq-1;
        f.capture.pid_valid = previous.valid; rate.capture_allocation(f); previous = result;
        if (result.valid) {
            EXPECT_FALSE(result.command.is_nan() || result.command.is_inf());
        }
    }
}
TEST(UTBAPAllocation, DefaultPolicyStillStopsAndClockWrapIsSupported)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
    auto in = input(1); rate.step(in, true); in = input(2); in.allocation_limited = true;
    EXPECT_FALSE(rate.step(in, true).valid); EXPECT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::SATURATION);
    ASSERT_TRUE(rate.configure(gains, true)); in = input(1);
    in.state.sampled_us = in.state.imu_updated_us = UINT32_MAX-1000; in.now_us = UINT32_MAX-990;
    EXPECT_TRUE(rate.step(in, true).priming);
    auto f = matrix.allocate({}, 1, in.now_us); rate.capture_allocation(f);
    in = input(2); in.state.sampled_us = in.state.imu_updated_us = 1499; in.now_us = 1509;
    EXPECT_TRUE(rate.step(in, true).valid);
}
#endif
AP_GTEST_MAIN()
