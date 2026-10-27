// GNU GPL v3 or later; original ArduPilot algorithms retain their copyright.
/// @file AP_UTB_ChainShadow.h
/// @brief Value-only Attitude(N) to Rate(N+1) continuous diagnostic transport.
/// @author Jiri Kovar
/// @version 2026-10-24
#pragma once
#ifndef AP_UTB_CHAIN_SHADOW_ENABLED
#define AP_UTB_CHAIN_SHADOW_ENABLED 0
#endif
#include "AP_UTB_config.h"
#define AP_UTB_CHAIN_SHADOW_AVAILABLE (AP_UTB_ENABLED && AP_UTB_CHAIN_SHADOW_ENABLED)
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
#if !AP_UTB_NATIVE_SHADOW_AVAILABLE || !AP_UTB_ATTITUDE_SHADOW_AVAILABLE
#error Chain shadow requires the value capture interfaces of native rate and attitude shadow
#endif
#include "AP_UTB_AttitudeShadow.h"
#include "AP_UTB_NativeShadow.h"
#include "AP_UTB_FlightBin.h"
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
#include "AP_UTB_FlightShadow.h"
#endif

class UTB_ChainShadowEngine
{
public:
    struct Statistics {
        uint32_t attempts = 0, paired = 0, warmup = 0, missing = 0, duplicate = 0;
        uint32_t invalid = 0, unsupported = 0, upstream_mismatch = 0, downstream_mismatch = 0;
        uint32_t motor_checks = 0, motor_changes = 0, limited = 0, independent_i = 0;
        uint32_t source_loop = 0, rate_loop = 0;
        uint32_t rate_max_us = 0, rate_calls = 0;
        uint64_t rate_sum_us = 0;
        double target_max[4] {}, target_squares[4] {};
        float command_peak = 0, gyro_peak = 0, i_peak = 0, ff_peak = 0, dff_peak = 0;
    };
    void start();
    void begin(uint32_t loop, uint8_t mode, bool supported);
    void attitude(UTBAttitudeCapture::Frame frame);
    void rate(UTB_RateShadow::Frame frame, const Vector3f &sysid);
    void end();
    void audit(bool changed);
    const Statistics &statistics() const
    {
        return _stats;
    }
    const UTB_AttitudeShadowEngine &attitude_engine() const
    {
        return _attitude;
    }
    const UTB_RateShadow &rate_engine() const
    {
        return _rate;
    }
    const UTB_RateShadow::Frame &last_rate() const
    {
        return _last_rate;
    }
    const UTBAttitudeCapture::Frame &last_attitude() const
    {
        return _last_source;
    }
    const UTBAttitudeCapture::Frame &last_operation() const
    {
        return _last_attitude;
    }
    const UTBAttitudeCapture::History &consumed_attitude() const
    {
        return _source_own;
    }
    const Vector3f &consumed_target() const
    {
        return _consumed;
    }
    bool failed() const;
    static bool outputs_equal(const UTB_Rate::ShadowOutput &own, const UTB_Rate::ShadowOutput &native);
private:
    friend class UTBChainTestAccess;
    UTB_AttitudeShadowEngine _attitude;
    UTB_RateShadow _rate;
    Statistics _stats {};
    UTB_RateShadow::Frame _last_rate;
    UTBAttitudeCapture::Frame _last_attitude;
    UTBAttitudeCapture::Frame _last_source;
    UTBAttitudeCapture::History _source_own;
    UTBAttitudeCapture::History _target;
    Vector3f _consumed;
    uint32_t _loop = 0, _source = 0, _rate_seq = 0, _att_seq = 0;
    uint8_t _mode = 0, _primary = 0, _rate_calls = 0;
    bool _available = false, _supported = false, _open = false;
};

class AP_UTB_ChainShadow
{
public:
    AP_UTB_ChainShadow();
    void init();
    void prepare(bool supported, uint8_t mode);
    void finish();
    static bool enabled();
    static AP_UTB_ChainShadow *get_singleton()
    {
        return _singleton;
    }
    static void dump(ExpandingString &text);
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    static void dump_flight(ExpandingString &text, uint8_t page);
    void flight_context(bool armed, uint8_t spool, bool supported)
    {
        _flight.context(armed, spool);
        _flight_supported = supported;
#if AP_UTB_FLIGHT_BIN_AVAILABLE
        _bin_armed = armed;
#endif
    }
#endif
#if AP_UTB_FLIGHT_BIN_AVAILABLE
    void bin_worker_ready(bool ready)
    {
        _bin.worker_ready(ready);
    }
    void bin_failsafe(bool active)
    {
        _bin_failsafe = active;
    }
    void bin_worker(AP_Logger &logger, bool available)
    {
        _bin.worker(logger, available);
    }
#endif
    static const AP_Param::GroupInfo var_info[];
    void before_rate(const UTB_Rate::ShadowInput &input, const Vector3f &sysid,
                     uint32_t gyro_us, uint32_t imu_us, uint8_t gyro);
    void pre_axis(uint8_t axis, const AC_PID_Shadow &s)
    {
        _rate_frame.before[axis] = s;
    }
    void post_axis(uint8_t axis, const AC_PID_Shadow &s, float output, float ff);
    void complete_rate();
    void before_attitude(UTBAttitudeCapture::Command kind, const Vector3f &command, bool reset,
                         const UTBAttitudeCapture::Config &config, const UTBAttitudeCapture::Sensors &sensors,
                         const UTBAttitudeCapture::History &history);
    void after_attitude(const UTBAttitudeCapture::History &history);
    void capture_time(bool post, uint32_t elapsed_us);
    void audit(bool changed)
    {
        _engine.audit(changed);
    }
private:
    static AP_UTB_ChainShadow *_singleton;
    AP_Int8 _run;
    AP_Int32 _count;
#if AP_UTB_FLIGHT_BIN_AVAILABLE
    AP_UTB_FlightBin _bin;
    uint64_t _bin_now_us = 0;
    bool _bin_session = false, _bin_armed = false, _bin_failsafe = false;
#endif
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    friend class UTBFlightTestAccess;
    AP_UTB_FlightShadow _flight;
    bool _flight_session = false, _flight_supported = false;
#endif
    UTB_ChainShadowEngine _engine;
    UTB_RateShadow::Frame _rate_frame;
    UTBAttitudeCapture::Frame _att_frame;
    Vector3f _sysid;
    std::atomic<bool> _published {false};
    uint32_t _loop = 0, _limit = 0, _capture_start = 0;
    uint32_t _att_last_us = 0, _att_max_us = 0;
    uint64_t _pre_us = 0, _post_us = 0, _att_us = 0;
    uint8_t _reason = 0;
    bool _initialized = false, _running = false;
    void freeze(uint8_t reason);
};
#endif // AP_UTB_CHAIN_SHADOW_AVAILABLE
