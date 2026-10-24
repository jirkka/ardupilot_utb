// GNU GPL v3 or later; original oracle mathematics: ArduPilot contributors.
/// @file test_utb_native_shadow.cpp
/// @brief Independent live-state history replay tests; no physical output or endpoint.
/// @author Jiri Kovar
/// @version 2026-10-21
/// @date 2026-10-21
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_UTB/AP_UTB_NativeShadow.h>
#include <AC_PID/AC_PID.h>
#include <chrono>
#include <algorithm>
#include <cstdio>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#if AP_UTB_NATIVE_SHADOW_AVAILABLE

// Local fixtures use static zero-initialised storage, like the actual vehicle controller.
template <unsigned scenario> static void exercise(bool continuous, unsigned steps)
{
    static AC_PID native[3] {{.135f,.135f,.0036f,.07f,.5f,15,7,20,0,1,.002f},
        {.135f,.135f,.0036f,.07f,.5f,15,7,20,0,1,.002f},
        {.18f,.018f,0,.04f,.5f,10,3,20,0,1,.001f}
    };
    static UTB_RateShadow engine;
    UTB_RateShadow::Frame frame;
    unsigned expected_desync = 0, expected_invalid = 0, expected_unsupported = 0;
    static uint64_t timing_ns[6000]; // Host test storage only; never linked into vehicle firmware.
    std::chrono::steady_clock::time_point clock_start;
    uint64_t sum_ns = 0;
    engine.start(continuous);
    for (unsigned sample = 0; sample < steps; sample++) {
        frame = {};
        frame.seq = sample + 1; frame.time_us = uint32_t(0xffff0000U + sample * 2500U);
        frame.execution_supported = frame.captured = frame.completed = true;
        frame.input.dt = .0025f + (sample % 3)*.00001f;
        frame.input.yaw_ff_scale = .7f;
        if (scenario == 2 && sample == 500) {
            native[0].set_kP(.17f);
            expected_desync++;
        }
        if (scenario == 3 && sample >= 500) {
            frame.seq++;
        }
        if (scenario == 3 && sample == 500) {
            expected_desync++;
        }
        for (uint8_t axis = 0; axis < 3; axis++) {
            frame.input.target[axis] = sinf(sample*.03f + axis)*3;
            frame.input.gyro[axis] = cosf(sample*.021f + axis);
            frame.input.pd_scale[axis] = .8f; frame.input.i_scale[axis] = .7f;
            frame.input.actuator[axis] = axis * .01f;
            frame.input.limited[axis] = (sample%90) < 40;
            if (sample % 113 == 0) {
                native[axis].reset_filter();
            }
            if (sample % 281 == 0) {
                native[axis].reset_I();
            }
            if (sample % 307 == 0) {
                native[axis].set_integrator(.1f);
            }
            native[axis].set_pdmax(.2f);
            frame.before[axis] = native[axis].shadow_snapshot();
            frame.expected.control[axis] = native[axis].update_all(frame.input.target[axis], frame.input.gyro[axis],
                                           frame.input.dt, frame.input.limited[axis], .8f, .7f) + frame.input.actuator[axis];
            frame.expected.feedforward[axis] = native[axis].get_ff();
            frame.after[axis] = native[axis].shadow_snapshot();
        }
        frame.expected.feedforward[2] *= frame.input.yaw_ff_scale;
        if (scenario == 4 && sample == 500) {
            frame.input.dt = NAN;
            expected_invalid++;
        }
        if (scenario == 5 && sample == 500) {
            frame.before[0].notch = frame.after[0].notch = true;
            expected_unsupported++;
        }
        if (scenario == 6 && sample == 500) {
            frame.before[0].config[AC_PID_Shadow::SMAX] = frame.after[0].config[AC_PID_Shadow::SMAX] = 50;
            expected_unsupported++;
        }
        if (scenario == 7 && sample == 500) {
            frame.expected.control[0] += .01f;
        }
        if (scenario == 8 && sample == 500) {
            frame.completed = false;
            expected_desync++;
        }
        if (scenario == 9 && sample == 500) {
            frame.execution_supported = false;
            expected_unsupported++;
        }
        if (scenario == 10 && sample == 500) {
            frame.before[0].derivative += 1;
            expected_desync++;
        }
        if (scenario == 11 && sample == 500) {
            frame.input.target.x = NAN;
            expected_invalid++;
        }
        if (scenario == 12 && sample == 500) {
            frame.input.gyro.y = INFINITY;
            expected_invalid++;
        }
        if (scenario == 13 && sample == 500) {
            frame.before[0].config[AC_PID_Shadow::P] = NAN;
            expected_invalid++;
        }
        if (scenario == 14 && sample >= 500) {
            frame.gyro_instance = 1;
        }
        if (scenario == 14 && sample == 500) {
            expected_desync++;
        }
        if (scenario == 15 && sample == 500) {
            frame.input.dt = 0;
            expected_invalid++;
        }
        if (scenario == 16 && sample == 500) {
            frame.after[2].info.DFF = NAN;
            expected_invalid++;
        }
        if (scenario == 17 && sample == 500) {
            frame.input.dt = -.0025f;
            expected_invalid++;
        }
        if (scenario == 18 && sample == 500) {
            frame.captured = false;
            expected_desync++;
        }
        if (scenario == 19 && sample == 500) {
            frame.after[0].config[AC_PID_Shadow::P] += .01f;
            expected_desync++;
        }
        if (scenario == 20) {
            frame.seq = uint32_t(0xffffff00U + sample);
        }
        if (scenario == 22) {
            clock_start = std::chrono::steady_clock::now();
        }
        engine.process(frame);
        if (scenario == 22) {
            timing_ns[sample] = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now() - clock_start).count();
            sum_ns += timing_ns[sample];
        }
        // The diagnostic receives no mutable AP controller reference; verify native history also stayed intact.
        for (uint8_t axis = 0; axis < 3; axis++) {
            AC_PID_Shadow after = native[axis].shadow_snapshot();
            if (scenario != 16 || sample != 500 || axis != 2) {
                EXPECT_TRUE(UTB_RateShadow::same_history(frame.after[axis], after));
            }
        }
    }
    if (scenario == 22) {
        std::sort(timing_ns, timing_ns + steps);
        printf("UTB_HOST_ENGINE_NS N=%u mean=%.1f p50=%llu p95=%llu p99=%llu max=%llu\n", steps,
               double(sum_ns) / steps, (unsigned long long)timing_ns[steps/2],
               (unsigned long long)timing_ns[steps*95/100], (unsigned long long)timing_ns[steps*99/100],
               (unsigned long long)timing_ns[steps-1]);
    }
    const auto &s = engine.statistics();
    EXPECT_EQ(s.seen, steps);
    EXPECT_EQ(s.period_count, steps - 1);
    EXPECT_EQ(s.period_sum_us, uint64_t(steps - 1)*2500U);
    EXPECT_EQ(s.period_min_us, 2500U); EXPECT_EQ(s.period_max_us, 2500U);
    EXPECT_EQ(s.jitter_sum_us, 0U);
    EXPECT_EQ(s.invalid, expected_invalid);
    EXPECT_EQ(s.unsupported, expected_unsupported);
    EXPECT_EQ(s.desync, expected_desync);
    EXPECT_EQ(s.valid + s.invalid + s.unsupported + s.desync, steps);
    EXPECT_EQ(s.mismatch, scenario == 7 ? 1U : 0U);
    if (continuous && scenario == 1) {
        EXPECT_EQ(s.synchronizations, 1U);
        EXPECT_GT(s.i_events, 0U); EXPECT_GT(s.filter_resets, 0U);
    }
    if (!continuous) {
        EXPECT_EQ(s.synchronizations, s.valid);
    }
    if (scenario == 2 || scenario == 19) {
        EXPECT_EQ(s.config_changes, 1U);
    }
    if (scenario != 7) {
        for (const auto &axis : s.maximum) {
            for (float value : axis) {
                EXPECT_FLOAT_EQ(value, 0.0f);
            }
        }
        for (const auto &axis : s.squares) {
            for (double value : axis) {
                EXPECT_DOUBLE_EQ(value, 0.0);
            }
        }
    } else {
        EXPECT_GT(s.maximum[0][11], .009f);
        EXPECT_EQ(s.first_difference, 501U);
    }
}
TEST(UTBNativeShadow, SingleStepHistoryFFDFiltersAndSaturation)
{
    exercise<0>(false, 6000);
}
TEST(UTBNativeShadow, IndependentContinuousHistoryAndRealResets)
{
    exercise<1>(true, 6000);
}
TEST(UTBNativeShadow, ParameterChangeEpoch)
{
    exercise<2>(true, 1000);
}
TEST(UTBNativeShadow, MissingSequenceIsDesync)
{
    exercise<3>(true, 1000);
}
TEST(UTBNativeShadow, InvalidDtIsNotZeroError)
{
    exercise<4>(true, 1000);
}
TEST(UTBNativeShadow, NotchUnsupported)
{
    exercise<5>(true, 1000);
}
TEST(UTBNativeShadow, SlewUnsupported)
{
    exercise<6>(true, 1000);
}
TEST(UTBNativeShadow, NumericDifferenceIsRetained)
{
    exercise<7>(true, 1000);
}
TEST(UTBNativeShadow, MissingCaptureIsDesync)
{
    exercise<8>(true, 1000);
}
TEST(UTBNativeShadow, RateThreadUnsupported)
{
    exercise<9>(true, 1000);
}
TEST(UTBNativeShadow, CorruptPreHistoryIsDesync)
{
    exercise<10>(true, 1000);
}
TEST(UTBNativeShadow, NaNTarget)
{
    exercise<11>(true, 1000);
}
TEST(UTBNativeShadow, InfiniteGyro)
{
    exercise<12>(true, 1000);
}
TEST(UTBNativeShadow, InvalidConfiguration)
{
    exercise<13>(true, 1000);
}
TEST(UTBNativeShadow, PrimaryGyroChangeEpoch)
{
    exercise<14>(true, 1000);
}
TEST(UTBNativeShadow, ZeroDt)
{
    exercise<15>(true, 1000);
}
TEST(UTBNativeShadow, InvalidNativePostState)
{
    exercise<16>(true, 1000);
}
TEST(UTBNativeShadow, NegativeDt)
{
    exercise<17>(true, 1000);
}
TEST(UTBNativeShadow, MissingPreCapture)
{
    exercise<18>(true, 1000);
}
TEST(UTBNativeShadow, ConfigurationChangedDuringCapture)
{
    exercise<19>(true, 1000);
}
TEST(UTBNativeShadow, SequenceAndClockWrap)
{
    exercise<20>(true, 1000);
}

TEST(UTBNativeShadow, HostOnlyEngineWallCost)
{
    exercise<22>(true, 6000);
}
// Public unit access is limited to the diagnostic session, never an AP flight controller.
class UTBNativeShadowTestAccess
{
public:
    static void request(AP_UTB_NativeShadow &session)
    {
        session._run.set(2);
        session._count.set(10);
    }
    static uint8_t reason(const AP_UTB_NativeShadow &session)
    {
        return session._stop_reason;
    }
    static uint32_t prepared(const AP_UTB_NativeShadow &session)
    {
        return session._seq;
    }
    static uint32_t seen(const AP_UTB_NativeShadow &session)
    {
        return session._engine.statistics().seen;
    }
    static int8_t run(const AP_UTB_NativeShadow &session)
    {
        return session._run.get();
    }
};
TEST(UTBNativeShadow, MissingConsumerFreezesWithoutOverwriteAndCannotRestart)
{
    static AP_UTB_NativeShadow session;
    session.init();
    UTBNativeShadowTestAccess::request(session);
    session.prepare(true, 0);
    session.prepare(true, 0);
    EXPECT_EQ(UTBNativeShadowTestAccess::reason(session), 7U);
    EXPECT_EQ(UTBNativeShadowTestAccess::prepared(session), 1U);
    EXPECT_EQ(UTBNativeShadowTestAccess::seen(session), 0U);
    EXPECT_EQ(UTBNativeShadowTestAccess::run(session), 0);
    UTBNativeShadowTestAccess::request(session);
    session.prepare(true, 0);
    EXPECT_EQ(UTBNativeShadowTestAccess::prepared(session), 1U);
    EXPECT_EQ(UTBNativeShadowTestAccess::seen(session), 0U);
    EXPECT_EQ(UTBNativeShadowTestAccess::run(session), 0);
}

#endif
AP_GTEST_MAIN()
