/*
 * GNU General Public License version 3 or later.
 * @file AP_UTB_Timing.h
 * @brief Passive paired main/rate/attitude timestamps; no control authority.
 * @author Jiri Kovar
 * @version 2026-10-24
 */
#pragma once
#ifndef AP_UTB_TIMING_ENABLED
#define AP_UTB_TIMING_ENABLED 0
#endif
#if AP_UTB_TIMING_ENABLED
#include <AP_Param/AP_Param.h>
#include <AP_Common/ExpandingString.h>
#include <atomic>

class AP_UTB_TimingEngine
{
public:
    static constexpr uint8_t METRICS = 5, GROUPS = 4, PRE = 8, CONTEXT = 17;
    // Presence uses counters, so timestamp zero and uint32 wrap are valid.
    struct Record {
        uint32_t cycle = 0, main = 0, rate = 0, att = 0;
        uint8_t mode = 255, uas = 255, rate_calls = 0, att_calls = 0;
        uint8_t kind = 255, yaw_resets = 0;
        bool supported = false, missing_finish = false;
    };
    struct Metric {
        int32_t minimum = INT32_MAX, maximum = INT32_MIN;
        int64_t sum = 0;
        uint64_t absolute_error = 0;
        uint32_t count = 0, over_nominal = 0, over_two_periods = 0;
        uint32_t min_cycle = 0, max_cycle = 0, min_us = 0, max_us = 0;
    };
    struct Statistics {
        Metric metrics[GROUPS][METRICS] {};
        uint32_t cycles = 0, missing_rate = 0, missing_att = 0, duplicate_rate = 0, duplicate_att = 0;
        uint32_t missing_finish = 0, invalid_time = 0, cycle_gaps = 0, phase_changes = 0, unsupported = 0;
    };
    void start(uint32_t nominal_us);
    void begin(uint32_t cycle, uint32_t now);
    void context(uint8_t mode, int8_t uas, bool supported);
    void rate(uint32_t now);
    void attitude(uint32_t now, uint8_t kind);
    void yaw_reset();
    void finish();
    void close_pending();
    bool pending() const
    {
        return _pending;
    }
    const Statistics &stats() const
    {
        return _stats;
    }
    const Record &worst(uint8_t i) const
    {
        return _worst[i];
    }
    uint8_t worst_count() const
    {
        return _worst_count;
    }
    uint32_t worst_cycle() const
    {
        return _worst_cycle;
    }
private:
    Statistics _stats {};
    Record _current {}, _previous {}, _ring[PRE] {}, _worst[CONTEXT] {};
    uint32_t _nominal = 2500, _worst_error = 0, _worst_cycle = 0;
    uint8_t _ring_next = 0, _ring_count = 0, _worst_count = 0, _following = 0;
    bool _pending = false, _have_previous = false, _have_worst = false;
    void value(uint8_t metric, int32_t v, uint8_t group, bool paired_group);
    void accumulate(Metric &m, uint8_t metric, int32_t v);
    void save_context(bool new_worst);
};

class AP_UTB_Timing
{
public:
    AP_UTB_Timing();
    void init()
    {
        _run.set(0);
        _initialized = true;
    }
    static const AP_Param::GroupInfo var_info[];
    static void loop(uint32_t now);
    static void context(uint8_t mode, int8_t uas, bool supported);
    static void rate();
    static void attitude(uint8_t kind);
    static void yaw_reset();
    static void finish();
    static void dump(ExpandingString &text);
private:
    static AP_UTB_Timing *_singleton;
    AP_Int8 _run;
    AP_Int32 _count;
    AP_UTB_TimingEngine _engine;
    std::atomic<bool> _published {false};
    bool _initialized = false, _running = false;
    uint32_t _cycle = 0, _limit = 0, _hook_max = 0, _hook_calls = 0;
    uint64_t _hook_sum = 0;
    uint8_t _reason = 0;
    static bool active();
    void overhead(uint32_t start);
    void freeze(uint8_t reason);
};
#endif // AP_UTB_TIMING_ENABLED
