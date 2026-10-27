/* GNU GPL v3 or later.
 * @file AP_UTB_ExecutionTiming.h
 * @brief Value-only extension of the finite UTT session; no control interfaces.
 * @author Jiri Kovar
 * @version 2026-10-25
 */
#pragma once
#ifndef AP_UTB_TIMING_EXEC_ENABLED
#define AP_UTB_TIMING_EXEC_ENABLED 0
#endif
#if AP_UTB_TIMING_EXEC_ENABLED
#include <stdint.h>
#include <limits.h>

class AP_UTB_ExecutionTiming
{
public:
    enum Event : uint8_t { RATE, CONSUME_BEGIN, CONSUME_END, MOTOR_BEGIN, MOTOR_END,
                           ATTITUDE, MODE_END, FAST_END, SCHED_END, EVENTS
                         };
    static constexpr uint8_t METRICS = 8, PRE = 8, CONTEXT = 17;
    struct Record {
        uint32_t cycle = 0; // UTT local cycle, distinct from Flight global source/consume IDs.
        uint32_t main = 0;
        uint32_t time[EVENTS] {};
        uint16_t present = 0;
        bool bad = false;
    };
    struct Metric {
        uint64_t sum = 0;
        uint32_t count = 0, minimum = UINT32_MAX, maximum = 0;
        uint32_t min_cycle = 0, max_cycle = 0, min_us = 0, max_us = 0;
        uint32_t over_nominal = 0, over_two_periods = 0;
    };
    struct Statistics {
        Metric metric[METRICS] {};
        uint32_t cycles = 0, missing = 0, duplicate = 0, invalid = 0;
        uint32_t gaps = 0, unsupported = 0;
    };
    void start();
    void begin(uint32_t cycle, uint32_t main);
    void event(Event kind, uint32_t cycle, uint32_t now);
    void close();
    void unsupported();
    const Statistics &stats() const
    {
        return _stats;
    }
    uint8_t context_count() const
    {
        return _worst_count;
    }
    const Record &context(uint8_t index) const
    {
        return _worst[index];
    }
    bool failed() const
    {
        return _stats.missing || _stats.duplicate || _stats.invalid || _stats.gaps;
    }
private:
    Statistics _stats {};
    Record _current {}, _ring[PRE] {}, _worst[CONTEXT] {};
    uint32_t _previous_cycle = 0, _previous_main = 0, _worst_duration = 0;
    uint8_t _last_event = 0, _ring_next = 0, _ring_count = 0, _worst_count = 0, _following = 0;
    bool _open = false, _have_previous = false, _have_worst = false;
    void save(bool worst);
};
#endif // AP_UTB_TIMING_EXEC_ENABLED
