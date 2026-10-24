// GNU GPL v3 or later; native AP is the numerical oracle.
/// @file test_utb_chain_shadow.cpp
/// @brief Real attitude output transport into independent rate histories, including injected faults.
/// @author Jiri Kovar
/// @version 2026-10-24
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_UTB/AP_UTB_ChainShadow.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
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

class UTBChainTestAccess
{
public:
    static void corrupt_target(UTB_ChainShadowEngine &e)
    {
        e._target.body.x += .25f;
    }
};
template<unsigned fault> static void exercise()
{
    static UTB_ChainShadowEngine chain;
    static AC_PID pid[3] {{.135f,.135f,.0036f,.07f,.5f,15,7,20,0,1,.002f},
        {.135f,.135f,.0036f,.07f,.5f,15,7,20,0,1,.002f},
        {.18f,.018f,0,.04f,.5f,10,3,20,0,1,.001f}
    };
    NativeOracle &native = oracle();
    UTB_RateShadow::Frame rate;
    Frame attitude;
    Vector3f sysid; // Common native excitation, independent from both attitude targets.
    uint32_t loop;
    uint8_t mode;
    native.seed(); chain.start();
    for (auto &p : pid) {
        p.reset_I();
        p.reset_filter();
    }
    for (uint32_t i = 0; i < 500; i++) {
        loop = fault == 10 ? uint32_t(0xffffff00U+i) : i;
        mode = fault == 5 && i >= 250 ? 1 : 0;
        if (fault == 8 && i == 250) {
            loop++;
        }
        chain.begin(loop,mode,!(fault == 9 && i == 250));
        rate = {}; rate.execution_supported=rate.captured=rate.completed=true;
        rate.input.dt=.0025f; rate.input.target=native.attitude_shadow_history().body;
        rate.input.yaw_ff_scale=native.attitude_shadow_history().feedforward;
        sysid = fault == 17 ? Vector3f{.1f,.2f,.3f} : Vector3f{};
        rate.input.target += sysid;
        rate.input.gyro= {sinf(i*.03f)*.2f,cosf(i*.02f)*.1f,.1f};
        rate.input.pd_scale=rate.input.i_scale= {1,1,1};
        rate.time_us=uint32_t(0xffff0000U+i*2500U); rate.mode=mode;
        for (uint8_t axis=0; axis<3; axis++) {
            if (i%127==0) {
                pid[axis].set_integrator(.01f);
            }
            if (i%91==0) {
                pid[axis].reset_filter();
            }
            rate.input.limited[axis]=i%20<10;
            if (fault == 17) {
                rate.input.actuator[axis] = .01f * axis;
            }
            rate.before[axis]=pid[axis].shadow_snapshot();
            rate.expected.control[axis]=pid[axis].update_all(rate.input.target[axis],rate.input.gyro[axis],.0025f,rate.input.limited[axis]) + rate.input.actuator[axis];
            rate.expected.feedforward[axis]=pid[axis].get_ff()*(axis==2?rate.input.yaw_ff_scale:1);
            rate.after[axis]=pid[axis].shadow_snapshot();
        }
        if (fault==1 && i==250) {
            UTBChainTestAccess::corrupt_target(chain);
        }
        if (fault==2 && i==250) {
            rate.expected.control[0]+=.1f;
        }
        if (fault==3 && i==250) {
            rate.input.dt=NAN;
        }
        if (fault==4 && i==250) {
            rate.before[0].config[0]+=.01f;
        }
        if (fault==11 && i==250) {
            rate.before[0].notch=rate.after[0].notch=true;
        }
        if (fault==12 && i==250) {
            rate.input.dt=0;
        }
        chain.rate(rate, sysid);
        if (fault == 2 && i == 249) {
            auto own_output = chain.rate_engine().output();
            EXPECT_TRUE(UTB_ChainShadowEngine::outputs_equal(own_output, rate.expected));
            own_output.control[0] += .1f; // Fault injected into the computed UTB value, native oracle unchanged.
            EXPECT_FALSE(UTB_ChainShadowEngine::outputs_equal(own_output, rate.expected));
            EXPECT_EQ(chain.statistics().upstream_mismatch, 0U);
        }
        if (fault==7 && i==250) {
            chain.rate(rate, sysid);
            if (fault == 2 && i == 249) {
                auto own_output = chain.rate_engine().output();
                EXPECT_TRUE(UTB_ChainShadowEngine::outputs_equal(own_output, rate.expected));
                own_output.control[0] += .1f; // Fault injected into the computed UTB value, native oracle unchanged.
                EXPECT_FALSE(UTB_ChainShadowEngine::outputs_equal(own_output, rate.expected));
                EXPECT_EQ(chain.statistics().upstream_mismatch, 0U);
            }
        }
        native.gyro(rate.input.gyro);
        attitude= {}; attitude.kind=mode==1?Command::ACRO:Command::STABILIZE;
        attitude.command= {.3f*sinf(i*.02f),.2f*cosf(i*.03f),.4f*sinf(i*.03f)};
        if (i%61==0) {
            attitude.kind = fault == 14 ? Command::RESET : fault == 15 ? Command::RELAX : fault == 16 ? Command::INERTIAL : Command::RESET_YAW; attitude.reset_rate=true;
            attitude.config=native.attitude_shadow_config(); attitude.sensors=native.attitude_shadow_sensors();
            attitude.before=native.attitude_shadow_history(); native.execute(attitude.kind, {},true);
            attitude.after=native.attitude_shadow_history(); chain.attitude(attitude);
            attitude.kind=mode==1?Command::ACRO:Command::STABILIZE;
        }
        attitude.config=native.attitude_shadow_config(); attitude.sensors=native.attitude_shadow_sensors();
        attitude.before=native.attitude_shadow_history(); native.execute(attitude.kind,attitude.command,false);
        attitude.after=native.attitude_shadow_history();
        if (!(fault==6 && i==250)) {
            chain.attitude(attitude);
        }
        chain.audit(fault==13 && i==250); chain.end();
        if (fault!=0 && fault!=5 && fault!=10 && fault<14 && i==250) {
            break;
        }
    }
    const auto &s=chain.statistics();
    const auto &a=chain.attitude_engine().statistics();
    const auto &p=chain.rate_engine().statistics();
    if (fault==0 || fault==5 || fault==10 || fault>=14) {
        EXPECT_EQ(s.paired,499U); EXPECT_EQ(s.warmup,1U); EXPECT_EQ(s.missing,0U); EXPECT_EQ(s.duplicate,0U);
        EXPECT_EQ(s.upstream_mismatch,0U); EXPECT_EQ(p.mismatch,0U); EXPECT_EQ(p.desync,0U);
        EXPECT_EQ(a.mismatch,0U); EXPECT_EQ(a.desync,0U); EXPECT_EQ(p.synchronizations,1U);
        EXPECT_EQ(a.synchronizations,fault==5?2U:1U); EXPECT_GT(s.independent_i,0U);
        EXPECT_GT(s.ff_peak,0); EXPECT_GT(a.shaping,0U); EXPECT_GT(a.resets,0U);
    } else if (fault==1) {
        EXPECT_EQ(s.upstream_mismatch,1U); EXPECT_EQ(p.mismatch,1U);
        EXPECT_FLOAT_EQ(chain.consumed_target().x-rate.input.target.x,.25f);
    } else if (fault==2) {
        EXPECT_EQ(s.upstream_mismatch,0U);
        EXPECT_EQ(p.mismatch,1U);
    } else if (fault==3 || fault==12) {
        EXPECT_EQ(p.invalid,1U);
    } else if (fault==4) {
        EXPECT_GT(p.config_changes,0U);
    } else if (fault==6 || fault==8) {
        EXPECT_GT(s.missing,0U);
    } else if (fault==7) {
        EXPECT_EQ(s.duplicate,1U);
    } else if (fault==9) {
        EXPECT_EQ(s.unsupported,1U);
    } else if (fault==11) {
        EXPECT_EQ(p.unsupported,1U);
    } else if (fault==13) {
        EXPECT_EQ(s.motor_changes,1U);
    }
}
TEST(UTBChain, PairingAndContinuous)
{
    exercise<0>();
}
TEST(UTBChain, UpstreamFaultActuallyReachesPID)
{
    exercise<1>();
}
TEST(UTBChain, DownstreamOutputOracleFault)
{
    exercise<2>();
}
TEST(UTBChain, NaN)
{
    exercise<3>();
}
TEST(UTBChain, Configuration)
{
    exercise<4>();
}
TEST(UTBChain, ModeEpoch)
{
    exercise<5>();
}
TEST(UTBChain, Missing)
{
    exercise<6>();
}
TEST(UTBChain, Duplicate)
{
    exercise<7>();
}
TEST(UTBChain, WrongPair)
{
    exercise<8>();
}
TEST(UTBChain, Unsupported)
{
    exercise<9>();
}
TEST(UTBChain, ClockAndLoopWrap)
{
    exercise<10>();
}
TEST(UTBChain, Notch)
{
    exercise<11>();
}
TEST(UTBChain, ZeroDt)
{
    exercise<12>();
}
TEST(UTBChain, MotorAudit)
{
    exercise<13>();
}
TEST(UTBChain, FullTargetReset)
{
    exercise<14>();
}
TEST(UTBChain, RelaxReplay)
{
    exercise<15>();
}
TEST(UTBChain, InertialReplay)
{
    exercise<16>();
}
TEST(UTBChain, SharedSysidAndActuator)
{
    exercise<17>();
}
#endif
AP_GTEST_MAIN()
