/*
 * GNU General Public License version 3 or later.
 * @file AP_UTB_Timing.cpp
 * @brief Fixed RAM diagnostics, separate from native controller mathematics.
 * @author Jiri Kovar
 * @version 2026-10-24
 */
#include "AP_UTB_Timing.h"
#if AP_UTB_TIMING_ENABLED
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Snapshot publication must be lock-free");

void AP_UTB_TimingEngine::start(uint32_t nominal_us)
{
    _stats = {}; _nominal = nominal_us; _pending = false; _have_previous = false;
    _ring_next = 0; _ring_count = 0; _worst_count = 0; _following = 0;
    _worst_error = 0; _worst_cycle = 0; _have_worst = false;
}
void AP_UTB_TimingEngine::close_pending()
{
    if (_pending) {
        _current.missing_finish = true;
        finish();
    }
}
void AP_UTB_TimingEngine::begin(uint32_t cycle, uint32_t now)
{
    if (_pending) {
        _current.missing_finish = true;
        finish();
    }
    _current = {}; _current.cycle = cycle; _current.main = now; _pending = true;
}
void AP_UTB_TimingEngine::context(uint8_t mode, int8_t uas, bool supported)
{
    if (_pending) {
        _current.mode = mode; _current.uas = uint8_t(uas); _current.supported = supported;
    }
}
void AP_UTB_TimingEngine::rate(uint32_t now)
{
    if (_pending) {
        if (_current.rate_calls == 0) {
            _current.rate = now;
        }
        if (_current.rate_calls < UINT8_MAX) {
            _current.rate_calls++;
        }
    }
}
void AP_UTB_TimingEngine::attitude(uint32_t now, uint8_t kind)
{
    if (_pending) {
        if (_current.att_calls == 0) {
            _current.att = now; _current.kind = kind;
        }
        if (_current.att_calls < UINT8_MAX) {
            _current.att_calls++;
        }
    }
}
void AP_UTB_TimingEngine::yaw_reset()
{
    if (_pending && _current.yaw_resets < UINT8_MAX) {
        _current.yaw_resets++;
    }
}
void AP_UTB_TimingEngine::accumulate(Metric &m, uint8_t metric, int32_t v)
{
    const int32_t expected = metric < 3 ? int32_t(_nominal) : 0; // Period versus delay/delta semantics.
    const int64_t error = int64_t(v) - expected;
    if (v < m.minimum) {
        m.minimum = v; m.min_cycle = _current.cycle; m.min_us = _current.main;
    }
    if (v > m.maximum) {
        m.maximum = v; m.max_cycle = _current.cycle; m.max_us = _current.main;
    }
    m.sum += v; m.absolute_error += error < 0 ? -error : error; m.count++;
    m.over_nominal += v > int32_t(_nominal);
    m.over_two_periods += v >= int32_t(2U * _nominal);
}
void AP_UTB_TimingEngine::value(uint8_t metric, int32_t v, uint8_t group, bool paired_group)
{
    accumulate(_stats.metrics[0][metric], metric, v);
    if (paired_group) {
        accumulate(_stats.metrics[group][metric], metric, v);
    }
}
void AP_UTB_TimingEngine::save_context(bool new_worst)
{
    if (new_worst) {
        _worst_count = 0;
        for (uint8_t i = 0; i < _ring_count; i++) {
            _worst[_worst_count++] = _ring[(_ring_next + PRE - _ring_count + i) % PRE];
        }
        _worst[_worst_count++] = _current; _following = PRE;
    } else if (_following != 0) {
        _worst[_worst_count++] = _current; _following--;
    }
    _ring[_ring_next] = _current; _ring_next = (_ring_next + 1) % PRE;
    if (_ring_count < PRE) {
        _ring_count++;
    }
}
void AP_UTB_TimingEngine::finish()
{
    uint8_t group; // Group 0=all, 1=UAS off, 2=one-step, 3=other.
    bool adjacent, same_phase, new_worst = false;
    uint32_t periods[3], delay, previous_delay, error;
    int64_t change;
    if (!_pending) {
        return;
    }
    group = _current.uas < 2 ? _current.uas + 1 : 3;
    adjacent = _have_previous && uint32_t(_current.cycle - _previous.cycle) == 1;
    same_phase = adjacent && _current.uas == _previous.uas && _current.mode == _previous.mode;
    _stats.cycles++;
    _stats.missing_rate += _current.rate_calls == 0; _stats.missing_att += _current.att_calls == 0;
    _stats.duplicate_rate += _current.rate_calls > 1; _stats.duplicate_att += _current.att_calls > 1;
    _stats.missing_finish += _current.missing_finish; _stats.unsupported += !_current.supported;
    _stats.cycle_gaps += _have_previous && !adjacent;
    _stats.phase_changes += adjacent && !same_phase;
    if (adjacent) {
        periods[0] = _current.main - _previous.main;
        periods[1] = _current.rate - _previous.rate;
        periods[2] = _current.att - _previous.att;
        for (uint8_t i = 0; i < 3; i++) {
            if ((i == 1 && (_current.rate_calls != 1 || _previous.rate_calls != 1)) ||
                (i == 2 && (_current.att_calls != 1 || _previous.att_calls != 1))) {
                continue;
            }
            if (periods[i] > INT32_MAX) {
                _stats.invalid_time++; continue;
            }
            value(i, int32_t(periods[i]), group, same_phase);
            if (i == 2) {
                error = periods[i] >= _nominal ? periods[i] - _nominal : _nominal - periods[i];
                if (!_have_worst || error > _worst_error) {
                    _have_worst = true; _worst_error = error; _worst_cycle = _current.cycle; new_worst = true;
                }
            }
        }
    }
    if (_current.rate_calls == 1 && _current.att_calls == 1) {
        delay = _current.att - _current.rate;
        if (delay <= INT32_MAX && uint32_t(_current.rate - _current.main) <= INT32_MAX) {
            value(3, int32_t(delay), group, true);
            if (adjacent && _previous.rate_calls == 1 && _previous.att_calls == 1) {
                previous_delay = _previous.att - _previous.rate;
                if (previous_delay <= INT32_MAX) {
                    change = int64_t(delay) - previous_delay;
                    value(4, int32_t(change), group, same_phase);
                } else {
                    _stats.invalid_time++;
                }
            }
        } else {
            _stats.invalid_time++;
        }
    }
    save_context(new_worst); _previous = _current; _have_previous = true; _pending = false;
}

AP_UTB_Timing *AP_UTB_Timing::_singleton;
const AP_Param::GroupInfo AP_UTB_Timing::var_info[] = {
    // @Param: RUN
    // @DisplayName: Paired attitude timing session
    // @Description: One finite DISARMED RAM-only timing session per boot. Does not activate attitude shadow or change control. Cleared at boot.
    // @Values: 0:Off,1:Capture
    // @User: Advanced
    AP_GROUPINFO("RUN", 0, AP_UTB_Timing, _run, 0),
    // @Param: COUNT
    // @DisplayName: Paired timing main loop count
    // @Description: Main loops to capture, including missing or unsupported native calls. Export only after completion.
    // @Range: 1 20000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("COUNT", 1, AP_UTB_Timing, _count, 4000),
    AP_GROUPEND
};
AP_UTB_Timing::AP_UTB_Timing()
{
    AP_Param::setup_object_defaults(this, var_info); _singleton = this;
}
bool AP_UTB_Timing::active()
{
    return hal.scheduler->in_main_thread() && _singleton != nullptr && _singleton->_running;
}
void AP_UTB_Timing::overhead(uint32_t start)
{
    const uint32_t elapsed = AP_HAL::micros() - start;
    _hook_sum += elapsed; _hook_calls++;
    if (elapsed > _hook_max) {
        _hook_max = elapsed;
    }
}
void AP_UTB_Timing::freeze(uint8_t reason)
{
    _reason = reason; _run.set(0); _running = false;
    _published.store(true, std::memory_order_release);
}
void AP_UTB_Timing::loop(uint32_t now)
{
    uint32_t start;
    if (_singleton == nullptr || !_singleton->_initialized || _singleton->_published.load(std::memory_order_relaxed)) {
        return;
    }
    AP_UTB_Timing &s = *_singleton;
    if (!s._running && s._run.get() == 0) {
        return;
    }
    start = AP_HAL::micros();
    if (!s._running) {
        if (s._run.get() != 1 || s._count.get() < 1 || s._count.get() > 20000 || hal.util->get_soft_armed()) {
            s.freeze(4); return;
        }
        s._engine.start(2500); s._limit = uint32_t(s._count.get()); s._running = true;
    }
    if (s._run.get() == 0 || hal.util->get_soft_armed()) {
        s._engine.close_pending(); s.overhead(start); s.freeze(2); return;
    }
    s._engine.begin(++s._cycle, now); s.overhead(start);
}
void AP_UTB_Timing::context(uint8_t mode, int8_t uas, bool supported)
{
    if (active()) {
        const uint32_t start = AP_HAL::micros();
        _singleton->_engine.context(mode, uas, supported && (mode == 0 || mode == 1)); _singleton->overhead(start);
    }
}
void AP_UTB_Timing::rate()
{
    if (active()) {
        const uint32_t start = AP_HAL::micros();
        _singleton->_engine.rate(start); _singleton->overhead(start);
    }
}
void AP_UTB_Timing::attitude(uint8_t kind)
{
    if (active()) {
        const uint32_t start = AP_HAL::micros();
        _singleton->_engine.attitude(start, kind); _singleton->overhead(start);
    }
}
void AP_UTB_Timing::yaw_reset()
{
    if (active()) {
        const uint32_t start = AP_HAL::micros();
        _singleton->_engine.yaw_reset(); _singleton->overhead(start);
    }
}
void AP_UTB_Timing::finish()
{
    if (active()) {
        AP_UTB_Timing &s = *_singleton;
        const uint32_t start = AP_HAL::micros();
        s._engine.finish(); s.overhead(start);
        if (s._engine.stats().cycles >= s._limit) {
            s.freeze(1);
        }
    }
}
void AP_UTB_Timing::dump(ExpandingString &text)
{
    if (_singleton == nullptr || !_singleton->_published.load(std::memory_order_acquire)) {
        text.printf("UTBTimingV1,pending\n"); return;
    }
    if (hal.util->get_soft_armed()) {
        text.printf("UTBTimingV1,armed-unavailable\n"); return;
    }
    const AP_UTB_Timing &s = *_singleton;
    const auto &v = s._engine.stats();
    text.printf("UTBTimingV1,frozen,reason=%u\n", unsigned(s._reason));
#define STAT(name) text.printf(#name ",%u\n", unsigned(v.name))
    STAT(cycles); STAT(missing_rate); STAT(missing_att); STAT(duplicate_rate); STAT(duplicate_att);
    STAT(missing_finish); STAT(invalid_time); STAT(cycle_gaps); STAT(phase_changes); STAT(unsupported);
#undef STAT
    text.printf("hook_sum_us,%llu\nhook_max_us,%u\nhook_calls,%u\nworst_cycle,%u\n",
                (unsigned long long)s._hook_sum, unsigned(s._hook_max), unsigned(s._hook_calls), unsigned(s._engine.worst_cycle()));
    text.printf("stat,group,metric,count,min,max,sum,absolute_error,over_nominal,over_two_periods,min_cycle,max_cycle,min_us,max_us\n");
    for (uint8_t g = 0; g < AP_UTB_TimingEngine::GROUPS; g++) {
        for (uint8_t k = 0; k < AP_UTB_TimingEngine::METRICS; k++) {
            const auto &m = v.metrics[g][k];
            text.printf("stat,%u,%u,%u,%ld,%ld,%lld,%llu,%u,%u,%u,%u,%u,%u\n", unsigned(g), unsigned(k), unsigned(m.count),
                        long(m.minimum), long(m.maximum), (long long)m.sum, (unsigned long long)m.absolute_error,
                        unsigned(m.over_nominal), unsigned(m.over_two_periods), unsigned(m.min_cycle), unsigned(m.max_cycle),
                        unsigned(m.min_us), unsigned(m.max_us));
        }
    }
    text.printf("record,cycle,main,rate,att,mode,uas,rate_calls,att_calls,kind,yaw_resets,supported,missing_finish\n");
    for (uint8_t i = 0; i < s._engine.worst_count(); i++) {
        const auto &f = s._engine.worst(i);
        text.printf("record,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", unsigned(f.cycle), unsigned(f.main), unsigned(f.rate), unsigned(f.att),
                    unsigned(f.mode), unsigned(f.uas), unsigned(f.rate_calls), unsigned(f.att_calls), unsigned(f.kind),
                    unsigned(f.yaw_resets), unsigned(f.supported), unsigned(f.missing_finish));
    }
}
#endif // AP_UTB_TIMING_ENABLED
