// GNU GPL v3 or later; native AP algorithms retain their original copyright.
/// @file AP_UTB_NativeShadow.h
/// @brief Finite RAM-only native 400Hz Rate/PID comparison; immutable export after stop.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-21
/// @date 2026-10-21
#pragma once
#include "AP_UTB_config.h"
#if AP_UTB_NATIVE_SHADOW_AVAILABLE
#include "UTB_Rate.h"
#include <atomic>
class ExpandingString;

// Public value types and pure comparison engine; never holds an AP controller or motor reference.
class UTB_RateShadow
{
public:
    static constexpr uint8_t METRICS = 18;
    struct Frame {
        UTB_Rate::ShadowInput input;
        UTB_Rate::ShadowOutput expected;
        AC_PID_Shadow before[3], after[3];
        uint32_t seq = 0, time_us = 0, gyro_time_us = 0, imu_time_us = 0;
        uint8_t gyro_instance = 0;
        uint32_t capture_us = 0;
        uint8_t mode = 0;
        bool captured = false, completed = false, execution_supported = false;
    };
    struct Statistics {
        uint32_t seen = 0, valid = 0, invalid = 0, desync = 0, unsupported = 0, mismatch = 0;
        uint32_t synchronizations = 0, config_changes = 0, filter_resets = 0, i_events = 0, mode_changes = 0;
        uint32_t unsupported_execution = 0, unsupported_notch = 0, unsupported_smax = 0;
        uint32_t first_seq = 0, last_seq = 0, first_us = 0, last_us = 0, first_difference = 0;
        uint32_t period_count = 0, period_min_us = 0, period_max_us = 0;
        uint64_t period_sum_us = 0, jitter_sum_us = 0;
        uint32_t math_max_us = 0, consume_max_us = 0, capture_max_us = 0;
        uint64_t math_sum_us = 0, consume_sum_us = 0, capture_sum_us = 0;
        double maximum[3][METRICS] {};
        double squares[3][METRICS] {};
    };
    void start(bool continuous);
    void process(const Frame &frame);
    const Statistics &statistics() const
    {
        return _stats;
    }
    bool continuous() const
    {
        return _continuous;
    }
#if AP_UTB_CHAIN_SHADOW_ENABLED
    // Value-only observation for the integrated passive consumer.
    const UTB_Rate::ShadowOutput &output() const
    {
        return _output;
    }
    AC_PID_Shadow history(uint8_t axis) const
    {
        return _pid[axis].shadow_snapshot();
    }
#endif
    static bool same_history(const AC_PID_Shadow &a, const AC_PID_Shadow &b);
private:
    // One independent history per axis; never reconstructed in the fast path.
    UTB_PID _pid[3] {{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}};
    Statistics _stats {};
#if AP_UTB_CHAIN_SHADOW_ENABLED
    UTB_Rate::ShadowOutput _output;
#endif
    bool _continuous = false, _synchronized = false;
    uint32_t _last_seq = 0;
    uint8_t _last_mode = 0, _last_gyro = 0;
};

class AP_UTB_NativeShadow
{
public:
    AP_UTB_NativeShadow();
    void init();
    // Main-thread orchestration, called before AP rate and after physical AP motor output.
    void prepare(bool execution_supported, uint8_t mode);
    bool active() const
    {
        return _running;
    }
    bool can_audit() const
    {
        return _frame.execution_supported;
    }
    void consume(const UTB_Rate::ShadowOutput &native_before);
    void finish_motor_audit(const UTB_Rate::ShadowOutput &native_after);
    void finish_unsupported();
    // Native read-only hooks: all arguments are values or references to caller-owned value snapshots.
    bool capturing() const
    {
        return _capture;
    }
    void before(const UTB_Rate::ShadowInput &input, uint32_t gyro_time_us, uint32_t imu_time_us, uint8_t gyro_instance);
    void pre_axis(uint8_t axis, const AC_PID_Shadow &state);
    void post_axis(uint8_t axis, const AC_PID_Shadow &state, float output, float ff);
    void complete();
    static AP_UTB_NativeShadow *get_singleton()
    {
        return _singleton;
    }
    static void dump(ExpandingString &text);
    static const AP_Param::GroupInfo var_info[];
private:
    friend class UTBNativeShadowTestAccess;
    static AP_UTB_NativeShadow *_singleton;
    AP_Int8 _run;
    AP_Int32 _count;
    UTB_RateShadow _engine;
    UTB_RateShadow::Frame _frame;
    std::atomic<bool> _published {false};
    bool _running = false, _capture = false, _initialized = false, _waiting = false;
    uint32_t _seq = 0, _limit = 0, _capture_started_us = 0;
    uint8_t _stop_reason = 0, _pending_stop = 0;
    uint32_t _motor_checks = 0, _motor_changes = 0, _motor_skipped = 0;
    UTB_Rate::ShadowOutput _native_before;
    void freeze(uint8_t reason);
};
#endif
