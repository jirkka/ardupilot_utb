/* GNU GPL v3 or later.
 * @file AP_UTB_ExecutionTiming.cpp
 * @brief Bounded monotonic event accounting, independent of actuator values.
 * @author Jiri Kovar
 * @version 2026-10-25
 */
#include "AP_UTB_ExecutionTiming.h"
#if AP_UTB_TIMING_EXEC_ENABLED
void AP_UTB_ExecutionTiming::start()
{
    _stats = {}; _current = {}; _ring_next = 0; _ring_count = 0; _worst_count = 0;
    _following = 0; _open = false; _have_previous = false; _have_worst = false; _worst_duration = 0;
}
void AP_UTB_ExecutionTiming::begin(uint32_t cycle, uint32_t main)
{
    if (_open) {
        close(); // Missing scheduler-end is visible; never borrow the next cycle's event.
    }
    _current = {}; _current.cycle = cycle; _current.main = main; _last_event = 0; _open = true;
    if (_have_previous && uint32_t(cycle - _previous_cycle) != 1) {
        _stats.gaps++; _current.bad = true;
    }
    if (_have_previous && uint32_t(main - _previous_main) > INT32_MAX) {
        _stats.invalid++; _current.bad = true;
    }
    _previous_cycle = cycle; _previous_main = main; _have_previous = true;
}
void AP_UTB_ExecutionTiming::event(Event kind, uint32_t cycle, uint32_t now)
{
    uint32_t previous; // Last accepted timestamp, or Main start for the first event.
    if (!_open || kind >= EVENTS || cycle != _current.cycle) {
        _stats.invalid++;
        if (_open) {
            _current.bad = true;
        }
        return;
    }
    if (_current.present & (1U << kind)) {
        _stats.duplicate++; _current.bad = true; return;
    }
    previous = _current.present ? _current.time[_last_event] : _current.main;
    if ((_current.present && kind < _last_event) || uint32_t(now - previous) > INT32_MAX ||
        uint32_t(now - _current.main) > INT32_MAX) {
        _stats.invalid++; _current.bad = true;
    }
    _current.time[kind] = now; _current.present |= 1U << kind; _last_event = kind;
}
void AP_UTB_ExecutionTiming::unsupported()
{
    _stats.unsupported++;
    if (_open) {
        _current.bad = true;
    }
}
void AP_UTB_ExecutionTiming::save(bool worst)
{
    if (worst) {
        _worst_count = 0;
        for (uint8_t i = 0; i < _ring_count; i++) {
            _worst[_worst_count++] = _ring[(_ring_next + PRE - _ring_count + i) % PRE];
        }
        _worst[_worst_count++] = _current; _following = PRE;
    } else if (_following) {
        _worst[_worst_count++] = _current; _following--;
    }
    _ring[_ring_next] = _current; _ring_next = (_ring_next + 1) % PRE;
    if (_ring_count < PRE) {
        _ring_count++;
    }
}
void AP_UTB_ExecutionTiming::close()
{
    static constexpr uint16_t required = (1U << RATE) | (1U << MOTOR_BEGIN) | (1U << MOTOR_END) |
                                         (1U << ATTITUDE) | (1U << MODE_END) | (1U << FAST_END) | (1U << SCHED_END);
    static constexpr uint8_t left[METRICS] = {EVENTS, MOTOR_BEGIN, EVENTS, EVENTS, RATE,
                                              CONSUME_BEGIN, EVENTS, EVENTS
                                             };
    static constexpr uint8_t right[METRICS] = {MOTOR_BEGIN, MOTOR_END, MOTOR_END, SCHED_END,
                                               MOTOR_BEGIN, CONSUME_END, MODE_END, FAST_END
                                              };
    bool worst = false; // Raw context is centred on the largest valid Main-to-motor-return span.
    bool consume_begin, consume_end;
    uint32_t first, value;
    if (!_open) {
        return;
    }
    _stats.cycles++;
    for (uint8_t k = 0; k < EVENTS; k++) {
        if ((required & (1U << k)) && !(_current.present & (1U << k))) {
            _stats.missing++; _current.bad = true;
        }
    }
    consume_begin = _current.present & (1U << CONSUME_BEGIN);
    consume_end = _current.present & (1U << CONSUME_END);
    if (consume_begin != consume_end) {
        _stats.missing++; _current.bad = true;
    }
    if (!_current.bad) {
        for (uint8_t k = 0; k < METRICS; k++) {
            if (k == 5 && !consume_begin) {
                continue; // Runtime OFF is absent, not a fake zero-duration UTB sample.
            }
            first = left[k] == EVENTS ? _current.main : _current.time[left[k]];
            value = _current.time[right[k]] - first;
            Metric &m = _stats.metric[k];
            if (!m.count || value < m.minimum) {
                m.minimum = value; m.min_cycle = _current.cycle; m.min_us = _current.main;
            }
            if (!m.count || value > m.maximum) {
                m.maximum = value; m.max_cycle = _current.cycle; m.max_us = _current.main;
            }
            m.sum += value; m.count++; m.over_nominal += value > 2500; m.over_two_periods += value >= 5000;
            if (k == 2 && (!_have_worst || value > _worst_duration)) {
                _have_worst = true; _worst_duration = value; worst = true;
            }
        }
    }
    save(worst); _open = false;
}
#endif // AP_UTB_TIMING_EXEC_ENABLED
