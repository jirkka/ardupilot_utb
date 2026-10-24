// GNU GPL v3 or later; native ArduPilot is the independent numerical oracle.
/// @file test_utb_attitude_shadow.cpp
/// @brief Attitude history, commands, reset and diagnostic rejection regression tests.
/// @author Jiri Kovar
/// @version 2026-10-23
#include <AP_gtest.h>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <AP_HAL/AP_HAL.h>
#include <AP_UTB/AP_UTB_AttitudeShadow.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#if AP_UTB_ATTITUDE_SHADOW_AVAILABLE
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
using namespace UTBAttitudeCapture;
class NativeOracle : public AC_AttitudeControl_Multi
{
public:
    NativeOracle(AP_AHRS_View &view, AP_MotorsMulticopter &motors) : AC_AttitudeControl_Multi(view, motors, false) {}
    void seed()
    {
        _attitude_target.initialise(); _attitude_ang_error.initialise();
        _euler_angle_target_rad.zero(); _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero(); _ang_accel_target_rads.zero(); _ang_vel_body_rads.zero();
        _thrust_angle_rad = 0; _thrust_error_angle_rad = 0; _feedforward_scalar = 1;
        _angle_P_scale = {1,1,1}; _rate_bf_ff_enabled.set(1);
        _rate_rp_tc = .1f; _rate_y_tc = .1f; _dt_s = .0025f;
    }
    void gyro(const Vector3f &v)
    {
        _rate_gyro_rads = v;
    }
    void shaping(bool ff)
    {
        _rate_bf_ff_enabled.set(ff);
    }
    void execute(Command kind, const Vector3f &c, bool reset)
    {
        switch (kind) {
        case Command::STABILIZE: input_euler_angle_roll_pitch_euler_rate_yaw_rad(c.x,c.y,c.z); break;
        case Command::ACRO: input_rate_bf_roll_pitch_yaw_rads(c.x,c.y,c.z); break;
        case Command::RATE_ONLY: input_rate_bf_roll_pitch_yaw_2_rads(c.x,c.y,c.z); break;
        case Command::RESET: reset_target_and_rate(reset); break;
        case Command::RESET_YAW: reset_yaw_target_and_rate(reset); break;
        case Command::RELAX: relax_attitude_controllers(); break;
        case Command::INERTIAL: inertial_frame_reset(); break;
        }
    }
};
static NativeOracle &oracle()
{
    static AP_Scheduler scheduler;
    static AP_InertialSensor ins;
    static AP_AHRS ahrs;
    static AP_AHRS_View view(ahrs, ROTATION_NONE);
    static AP_MotorsMatrix motors;
    static NativeOracle native(view, motors);
    return native;
}
// Scenarios intentionally perturb the transport, not the original native algorithm.
template <unsigned scenario> static void compare(bool continuous)
{
    static UTB_AttitudeShadowEngine engine;
    NativeOracle &native = oracle();
    Frame f;
    uint32_t bad = 0;
    static uint64_t ns[2000];
    uint64_t sum = 0;
    native.seed(); engine.start(continuous);
    for (unsigned i = 0; i < 2000; i++) {
        f = {}; f.seq = i+1; f.loop = i+1;
        f.kind = scenario == 1 ? Command::ACRO : scenario == 2 ? Command::RATE_ONLY : Command::STABILIZE;
        if (scenario == 3) {
            native.shaping(false);
        }
        f.command = {sinf(i*.02f)*.4f, cosf(i*.011f)*.35f, sinf(i*.015f)};
        native.gyro({.1f*sinf(i*.03f), .2f*cosf(i*.02f), .15f});
        if (scenario == 4 && i%100 == 0) {
            f.kind = Command::RESET;
            f.reset_rate = (i%200 == 0);
        }
        if (scenario == 5 && i%100 == 0) {
            f.kind = Command::RESET_YAW;
            f.reset_rate = true;
        }
        if (scenario == 6 && i%100 == 0) {
            f.kind = Command::RELAX;
        }
        if (scenario == 7 && i%100 == 0) {
            f.kind = Command::INERTIAL;
        }
        f.config = native.attitude_shadow_config(); f.sensors = native.attitude_shadow_sensors();
        f.before = native.attitude_shadow_history();
        native.execute(f.kind, f.command, f.reset_rate);
        f.after = native.attitude_shadow_history();
        if (scenario == 8 && i == 100) {
            f.command.x = NAN;
            bad++;
        }
        if (scenario == 9 && i >= 100) {
            f.seq++;
            if (i == 100) {
                bad++;
            }
        }
        if (scenario == 10 && i == 100) {
            f.before.body.x += 1;
            bad++;
        }
        if (scenario == 11 && i == 100) {
            f.config.input_tc += .1f;
            bad++;
        }
        if (scenario == 12 && i == 100) {
            f.config.trim = .1f;
            bad++;
        }
        if (scenario == 13 && i == 100) {
            f.after.body.x += .01f;
            bad++;
        }
        if (scenario == 14 && i == 100) {
            f.after.target.q1 *= -1; f.after.target.q2 *= -1; f.after.target.q3 *= -1; f.after.target.q4 *= -1;
        }
        if (scenario == 15 && i >= 1000) {
            f.mode = 1;
        }
        if (scenario == 18 && i == 100) {
            f.before.target.q1 = 2;
            bad++;
        }
        if (scenario == 19 && i == 100) {
            f.sensors.attitude.q1 = 2;
            bad++;
        }
        if (scenario == 21 && i == 100) {
            f.config.rate_max.x = nextafterf(f.config.rate_max.x, INFINITY);
            bad++;
        }
        if (scenario == 20 && i == 100) {
            f.after.target.q1 = 2;
            bad++;
        }
        if (scenario == 16 && i == 100) {
            f.config.dt = 0;
            bad++;
        }
        const auto start = std::chrono::steady_clock::now();
        if (scenario == 17 && i == 100) {
            engine.invalidate_epoch();
        }
        engine.process(f);
        ns[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
        sum += ns[i];
        ASSERT_EQ(engine.statistics().seen, i+1);
        if (scenario < 8 || scenario == 14 || scenario == 15 || scenario == 17) {
            ASSERT_EQ(engine.statistics().mismatch, 0U) << i;
            ASSERT_EQ(engine.statistics().desync, 0U) << i;
            ASSERT_EQ(engine.statistics().invalid, 0U) << i;
        }
    }
    if (scenario == 0 && continuous) {
        printf("HOST_SIZEOF,wrapper=%zu,engine=%zu,math=%zu,frame=%zu,pid=%zu,state=%zu\n",
               sizeof(AP_UTB_AttitudeShadow), sizeof(UTB_AttitudeShadowEngine), sizeof(UTB_AttitudeShadowMath),
               sizeof(Frame), sizeof(UTB_PID), sizeof(UTB_State));
        std::sort(ns, ns+2000);
        printf("HOST_ONLY_ATTITUDE_PROCESS_NS,mean=%.3f,p50=%llu,p95=%llu,p99=%llu,max=%llu\n",
               double(sum)/2000, (unsigned long long)ns[1000], (unsigned long long)ns[1900],
               (unsigned long long)ns[1980], (unsigned long long)ns[1999]);
    }
    const auto &s = engine.statistics();
    EXPECT_EQ(s.seen, 2000U);
    if (scenario < 8 || scenario == 14 || scenario == 15 || scenario == 17) {
        EXPECT_EQ(s.valid, 2000U);
        EXPECT_EQ(s.synchronizations, continuous ? ((scenario == 15 || scenario == 17) ? 2U : 1U) : 2000U);
        if (scenario == 3) {
            EXPECT_EQ(s.shaping, 0U);
        } else {
            EXPECT_GT(s.shaping, 1U);
        }
        if (scenario >= 4 && scenario <= 7) {
            EXPECT_EQ(s.reset_compared, 20U);
        }
    } else if (scenario == 8 || scenario == 16 || (scenario >= 18 && scenario <= 20)) {
        EXPECT_EQ(s.invalid, bad);
    } else if (scenario == 12) {
        EXPECT_EQ(s.unsupported, bad);
    } else if (scenario == 13) {
        EXPECT_EQ(s.mismatch, bad);
    } else {
        EXPECT_EQ(s.desync, bad);
    }
    if (scenario == 14) {
        EXPECT_GT(s.quaternion_sign_equivalent, 0U);
    }
}
TEST(UTBAttitudeShadow, StabilizeOneStep)
{
    compare<0>(false);
}
TEST(UTBAttitudeShadow, StabilizeContinuous)
{
    compare<0>(true);
}
TEST(UTBAttitudeShadow, AcroContinuous)
{
    compare<1>(true);
}
TEST(UTBAttitudeShadow, RateOnlyContinuous)
{
    compare<2>(true);
}
TEST(UTBAttitudeShadow, NoFeedforward)
{
    compare<3>(true);
}
TEST(UTBAttitudeShadow, TargetReset)
{
    compare<4>(true);
}
TEST(UTBAttitudeShadow, YawReset)
{
    compare<5>(true);
}
TEST(UTBAttitudeShadow, Relax)
{
    compare<6>(true);
}
TEST(UTBAttitudeShadow, InertialReset)
{
    compare<7>(true);
}
TEST(UTBAttitudeShadow, InvalidCommand)
{
    compare<8>(true);
}
TEST(UTBAttitudeShadow, SequenceGap)
{
    compare<9>(true);
}
TEST(UTBAttitudeShadow, UnannouncedHistory)
{
    compare<10>(true);
}
TEST(UTBAttitudeShadow, ConfigurationChange)
{
    compare<11>(true);
}
TEST(UTBAttitudeShadow, UnsupportedTrim)
{
    compare<12>(true);
}
TEST(UTBAttitudeShadow, OutputMismatch)
{
    compare<13>(true);
}
TEST(UTBAttitudeShadow, EquivalentQuaternionSign)
{
    compare<14>(true);
}
TEST(UTBAttitudeShadow, ModeEpoch)
{
    compare<15>(true);
}
TEST(UTBAttitudeShadow, InvalidDt)
{
    compare<16>(true);
}

TEST(UTBAttitudeShadow, UnsupportedModeEpochReturn)
{
    compare<17>(true);
}
TEST(UTBAttitudeShadow, InvalidPreQuaternion)
{
    compare<18>(true);
}
TEST(UTBAttitudeShadow, InvalidSensorQuaternion)
{
    compare<19>(true);
}
TEST(UTBAttitudeShadow, InvalidPostQuaternion)
{
    compare<20>(true);
}
TEST(UTBAttitudeShadow, ConfigurationFingerprintSingleULP)
{
    compare<21>(true);
}
class UTBAttitudeShadowTestAccess
{
public:
    static void request(AP_UTB_AttitudeShadow &s, int8_t run, int count)
    {
        s._run.set(run); s._count.set(count);
    }
    static uint32_t reason(const AP_UTB_AttitudeShadow &s)
    {
        return s._reason;
    }
    static uint32_t attempts(const AP_UTB_AttitudeShadow &s)
    {
        return s._attempts;
    }
    static uint32_t unsupported(const AP_UTB_AttitudeShadow &s)
    {
        return s._unsupported;
    }
    static bool published(const AP_UTB_AttitudeShadow &s)
    {
        return s._published.load();
    }
};
template <unsigned scenario> static void lifecycle()
{
    NativeOracle &native = oracle();
    Frame f;
    native.seed();
    f.config = native.attitude_shadow_config(); f.sensors = native.attitude_shadow_sensors();
    f.before = native.attitude_shadow_history(); f.command = {.2f,-.1f,.3f};
    native.execute(Command::STABILIZE, f.command, false);
    f.after = native.attitude_shadow_history();
    static AP_UTB_AttitudeShadow wrapper;
    wrapper.init();
    UTBAttitudeShadowTestAccess::request(wrapper, scenario == 0 ? 0 : scenario == 3 ? 3 : 2, 2);
    wrapper.prepare(scenario != 2, 0);
    if (scenario == 0) {
        wrapper.finish();
        EXPECT_EQ(UTBAttitudeShadowTestAccess::attempts(wrapper), 0U);
        EXPECT_FALSE(UTBAttitudeShadowTestAccess::published(wrapper));
    } else if (scenario == 1) {
        wrapper.finish();
        EXPECT_EQ(UTBAttitudeShadowTestAccess::reason(wrapper), 3U);
    } else if (scenario == 2) {
        wrapper.finish(); wrapper.prepare(false, 0); wrapper.finish();
        EXPECT_EQ(UTBAttitudeShadowTestAccess::unsupported(wrapper), 2U);
        EXPECT_EQ(UTBAttitudeShadowTestAccess::reason(wrapper), 1U);
    } else if (scenario == 3) {
        EXPECT_EQ(UTBAttitudeShadowTestAccess::reason(wrapper), 4U);
    } else if (scenario == 4) {
        wrapper.prepare(true, 0);
        EXPECT_EQ(UTBAttitudeShadowTestAccess::reason(wrapper), 7U);
    } else {
        AP_UTB_AttitudeShadow::before(Command::STABILIZE, f.command, false, f.config, f.sensors, f.before);
        AP_UTB_AttitudeShadow::after(f.after);
        AP_UTB_AttitudeShadow::audit(scenario == 6);
        wrapper.finish();
        if (scenario == 6) {
            EXPECT_EQ(UTBAttitudeShadowTestAccess::reason(wrapper), 3U);
        } else {
            UTBAttitudeShadowTestAccess::request(wrapper, 0, 2); wrapper.prepare(true, 0);
            EXPECT_EQ(UTBAttitudeShadowTestAccess::reason(wrapper), 2U);
        }
        const uint32_t count = UTBAttitudeShadowTestAccess::attempts(wrapper);
        wrapper.prepare(true, 0); wrapper.finish();
        EXPECT_EQ(UTBAttitudeShadowTestAccess::attempts(wrapper), count);
        EXPECT_TRUE(UTBAttitudeShadowTestAccess::published(wrapper));
    }
}
TEST(UTBAttitudeShadowLifecycle, DefaultOff)
{
    lifecycle<0>();
}
TEST(UTBAttitudeShadowLifecycle, MissingNativeCapture)
{
    lifecycle<1>();
}
TEST(UTBAttitudeShadowLifecycle, UnsupportedExecution)
{
    lifecycle<2>();
}
TEST(UTBAttitudeShadowLifecycle, InvalidRequest)
{
    lifecycle<3>();
}
TEST(UTBAttitudeShadowLifecycle, MissingMainCompletion)
{
    lifecycle<4>();
}
TEST(UTBAttitudeShadowLifecycle, CancellationAndImmutablePublication)
{
    lifecycle<5>();
}
TEST(UTBAttitudeShadowLifecycle, NativeMotorMutationStopsDiagnostic)
{
    lifecycle<6>();
}
#else
TEST(UTBAttitudeShadow, CompiledOut)
{
    SUCCEED();
}
#endif
AP_GTEST_MAIN()
