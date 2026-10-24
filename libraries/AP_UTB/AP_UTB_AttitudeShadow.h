// GNU GPL v3 or later; source algorithms: ArduPilot Copter 4.7.1.
/// @file AP_UTB_AttitudeShadow.h
/// @brief Independent existing UTB attitude math with finite, main-thread RAM diagnostics.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-23
#pragma once
#include "AP_UTB_config.h"
#if AP_UTB_ATTITUDE_SHADOW_AVAILABLE
#include "UTB_Attitude.h"
#include <atomic>
class ExpandingString;

// Inert adapter implements only attitude computation; no output sink exists.
class UTB_AttitudeShadowMath : public UTB_Attitude
{
public:
    UTB_AttitudeShadowMath() : UTB_Attitude(_state) {}
    void sample(const UTBAttitudeCapture::Sensors &s)
    {
        _state.shadow_sample(s.attitude, s.gyro, s.yaw);
        _rate_gyro_rads = s.gyro;
    }
    void execute(UTBAttitudeCapture::Command kind, const Vector3f &command, bool reset_rate);
    UTB_PID &get_rate_roll_pid() override
    {
        return _pid[0];
    }
    UTB_PID &get_rate_pitch_pid() override
    {
        return _pid[1];
    }
    UTB_PID &get_rate_yaw_pid() override
    {
        return _pid[2];
    }
    const UTB_PID &get_rate_roll_pid() const override
    {
        return _pid[0];
    }
    const UTB_PID &get_rate_pitch_pid() const override
    {
        return _pid[1];
    }
    const UTB_PID &get_rate_yaw_pid() const override
    {
        return _pid[2];
    }
    void rate_controller_run() override {}
    void update_althold_lean_angle_max(float) override {}
    void set_throttle_out(float, bool, float) override {}
private:
    UTB_State _state;
    // Required by original attitude yaw-error limit; no rate PID is executed here.
    UTB_PID _pid[3] {{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}};
};

class UTB_AttitudeShadowEngine
{
public:
    static constexpr uint8_t METRICS = 26;
    struct Statistics {
        uint32_t seen = 0, valid = 0, invalid = 0, desync = 0, unsupported = 0, mismatch = 0;
        uint32_t synchronizations = 0, resets = 0, reset_compared = 0, mode_changes = 0, config_changes = 0;
        uint32_t quaternion_sign_equivalent = 0, dynamic = 0, shaping = 0;
        uint32_t first_seq = 0, last_seq = 0, first_loop = 0, last_loop = 0;
        uint32_t math_max_us = 0, compare_max_us = 0, sync_max_us = 0;
        uint64_t math_sum_us = 0, compare_sum_us = 0, sync_sum_us = 0;
        uint32_t command_seen[7] {}, command_valid[7] {};
        double maximum[METRICS] {}, squares[METRICS] {};
    };
    void start(bool continuous);
    void process(const UTBAttitudeCapture::Frame &frame);
    void invalidate_epoch()
    {
        _synced = false;
    }
    const Statistics &statistics() const
    {
        return _stats;
    }
    // Own computed state, never the native post-history used as oracle.
    UTBAttitudeCapture::History history() const
    {
        return _math.shadow_history();
    }
    static void values(const UTBAttitudeCapture::History &h, float (&v)[METRICS]);
    static bool same_history(const UTBAttitudeCapture::History &a, const UTBAttitudeCapture::History &b);
private:
    UTB_AttitudeShadowMath _math;
    Statistics _stats {};
    UTBAttitudeCapture::Config _config {};
    bool _continuous = false, _synced = false;
    uint32_t _seq = 0;
    uint8_t _mode = 0;
};

class AP_UTB_AttitudeShadow
{
public:
    AP_UTB_AttitudeShadow();
    void init();
    void prepare(bool supported, uint8_t mode);
    void finish();
    static void dump(ExpandingString &text);
    static const AP_Param::GroupInfo var_info[];
    int8_t requested_run() const { return _run.get(); }
    static bool enabled();
    static void before(UTBAttitudeCapture::Command kind, const Vector3f &command, bool reset_rate,
                       const UTBAttitudeCapture::Config &config, const UTBAttitudeCapture::Sensors &sensors,
                       const UTBAttitudeCapture::History &history);
    static void after(const UTBAttitudeCapture::History &history);
    static void audit(bool changed);
    static void capture_time(bool post, uint32_t elapsed_us);
private:
    friend class UTBAttitudeShadowTestAccess;
    static AP_UTB_AttitudeShadow *_singleton;
    AP_Int8 _run;
    AP_Int32 _count;
    UTB_AttitudeShadowEngine _engine;
    UTBAttitudeCapture::Frame _frame;
    std::atomic<bool> _published {false};
    bool _initialized = false, _running = false, _supported = false, _pending = false;
    uint8_t _reason = 0, _mode = 0;
    uint32_t _attempts = 0, _calls = 0, _seq = 0, _limit = 0, _loop = 0;
    uint32_t _missing = 0, _duplicate = 0, _unsupported = 0;
    uint32_t _motor_checks = 0, _motor_changes = 0;
    uint32_t _consume_max = 0, _consume_last = 0;
    uint32_t _pre_max = 0, _post_max = 0, _period_count = 0;
    uint64_t _pre_sum = 0, _post_sum = 0;
    uint64_t _consume_sum = 0;
    uint32_t _native_start = 0, _native_max = 0, _period_last = 0, _period_min = UINT32_MAX, _period_max = 0;
    uint64_t _native_sum = 0, _period_sum = 0, _jitter_sum = 0;
    void freeze(uint8_t reason);
};
#endif
