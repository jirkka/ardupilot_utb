// GNU GPL v3 or later.
/// @file AP_UTB_FlightBin.cpp
/// @brief Fixed value transport. Main never calls the logger or waits for the consumer.
/// @author Jiri Kovar
/// @version 2026-10-26
#include "AP_UTB_FlightBin.h"
#if AP_UTB_FLIGHT_BIN_AVAILABLE
#include "AP_UTB_ChainShadow.h"
#include "FlightBinLogStructure.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>
#include <cstring>
#include <AP_Common/ExpandingString.h>

const AP_Param::GroupInfo AP_UTB_FlightBin::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Passive Flight BIN enable
    // @Description: Opt in before ARM. One finite native STABILIZE session per boot. No motor authority. Existing RAM diagnostics must be unused. Changing this while armed does not trigger a new session.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_UTB_FlightBin, _enable, 0, AP_PARAM_FLAG_ENABLE),
    // @Param: RATE
    // @DisplayName: Flight BIN sample frequency
    // @Description: Decimated complete sets, independent of the 400 Hz controller. Other values reject the diagnostic session.
    // @Values: 10:10 Hz,20:20 Hz
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RATE", 1, AP_UTB_FlightBin, _rate, 10),
    // @Param: SECS
    // @DisplayName: Maximum Flight BIN session duration
    // @Description: Wall clock and 400 Hz attempt count both bound the one session. Default 60 seconds. A stopped session cannot restart until a new boot.
    // @Range: 1 120
    // @Units: s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SECS", 2, AP_UTB_FlightBin, _seconds, 60),
    AP_GROUPEND
};

AP_UTB_FlightBin::AP_UTB_FlightBin()
{
    AP_Param::setup_object_defaults(this, var_info);
}
bool AP_UTB_FlightBin::Queue::push(const Sample &sample)
{
    const uint32_t head = _head.load(std::memory_order_relaxed); // Producer-owned cursor.
    const uint32_t tail = _tail.load(std::memory_order_acquire); // Consumer releases only after its copy.
    if (uint32_t(head - tail) >= CAPACITY) {
        return false;
    }
    _producer_depth = uint32_t(head - tail) + 1U;
    _slots[head % CAPACITY] = sample;
    _head.store(head + 1U, std::memory_order_release);
    return true;
}
bool AP_UTB_FlightBin::Queue::pop(Sample &sample)
{
    const uint32_t tail = _tail.load(std::memory_order_relaxed); // Consumer-owned cursor.
    const uint32_t head = _head.load(std::memory_order_acquire); // Sample bytes precede publication.
    if (tail == head) {
        return false;
    }
    sample = _slots[tail % CAPACITY];
    _tail.store(tail + 1U, std::memory_order_release);
    return true;
}
uint32_t AP_UTB_FlightBin::Queue::depth() const
{
    const uint32_t tail = _tail.load(std::memory_order_acquire);
    const uint32_t head = _head.load(std::memory_order_acquire);
    return MIN(uint32_t(head-tail), CAPACITY);
}
bool AP_UTB_FlightBin::arm_edge(bool armed, bool mode_supported, bool available, uint64_t now_us)
{
    const bool edge = armed && !_last_armed; // Main owns ARM observation; no native state is modified.
    _last_armed = armed;
    if (!armed) {
        _opted_in_disarmed = _enable.get() == 1;
        return false;
    }
    if (!edge || !_opted_in_disarmed || _enable.get() == 0 || _attempted) {
        return false;
    }
    _attempted = true;
    _start_us = now_us;
    if (_enable.get() != 1 || (_rate.get() != 10 && _rate.get() != 20) || _seconds.get() < 1 || _seconds.get() > 120) {
        stop(Reason::CONFIG); return false;
    }
    _hz = uint8_t(_rate.get()); _duration = uint16_t(_seconds.get());
    if (!_worker_ready) {
        stop(Reason::WORKER); return false;
    }
    if (!available) {
        stop(Reason::CONFLICT); return false;
    }
    if (!mode_supported) {
        stop(Reason::UNSUPPORTED); return false;
    }
    _state.store(uint32_t(State::RUNNING), std::memory_order_release);
    return true;
}
AP_UTB_FlightBin::Reason AP_UTB_FlightBin::stop_condition(bool armed, uint8_t mode, bool supported, uint64_t now_us, bool failsafe) const
{
    if (!armed) {
        return Reason::DISARM;
    }
    if (failsafe) {
        return Reason::FAILSAFE;
    }
    if (mode != 0) {
        return Reason::MODE;
    }
    if (!supported) {
        return Reason::UNSUPPORTED;
    }
    if (_enable.get() != 1) {
        return Reason::DISABLED;
    }
    if (_rate.get() != _hz || _seconds.get() != _duration) {
        return Reason::CONFIG;
    }
    if (now_us - _start_us >= uint64_t(_duration)*1000000U) {
        return Reason::TIME;
    }
    return Reason::NONE;
}
void AP_UTB_FlightBin::stop(Reason reason)
{
    if (state() == State::STOPPED) {
        return;
    }
    if (reason == Reason::UNSUPPORTED) {
        _unsupported.fetch_add(1, std::memory_order_relaxed);
    }
    _reason.store(uint32_t(reason), std::memory_order_relaxed);
    _state.store(uint32_t(State::STOPPED), std::memory_order_release);
}
bool AP_UTB_FlightBin::expand_time(uint32_t low, uint64_t reference, uint64_t &result)
{
    const int32_t delta = int32_t(low - uint32_t(reference)); // Nearest matching 32-bit clock epoch.
    if (delta < 0 && reference < uint64_t(-int64_t(delta))) {
        return false;
    }
    result = delta < 0 ? reference - uint64_t(-int64_t(delta)) : reference + uint32_t(delta);
    return true;
}
bool AP_UTB_FlightBin::make_sample(const UTB_ChainShadowEngine &engine, uint32_t loop, uint32_t seq,
                                   uint64_t reference, Sample &sample)
{
    const auto &stats = engine.statistics(); // Actual full-rate comparator result.
    const auto &rate = engine.last_rate();
    const auto &source = engine.last_attitude();
    const auto &target = engine.consumed_attitude();
    if (engine.failed() || engine.attitude_engine().statistics().mismatch || stats.downstream_mismatch ||
        stats.rate_loop != loop || stats.source_loop + 1U != loop || stats.paired == 0) {
        return false;
    }
    sample = {};
    if (!expand_time(rate.time_us, reference, sample.time_us) ||
        !expand_time(source.time_us, reference, sample.source_us) || sample.source_us > sample.time_us ||
        sample.time_us-sample.source_us > 100000U || !isfinite(rate.input.dt) || rate.input.dt <= 0) {
        return false;
    }
    sample.seq = seq; sample.loop = loop; sample.source = stats.source_loop; sample.mode = source.mode;
    sample.dt = rate.input.dt;
    for (uint8_t axis = 0; axis < 3; axis++) {
        const auto own = engine.rate_engine().history(axis);
        const auto &native = rate.after[axis].info;
        const float terms[] = {native.P, native.I, native.D, native.FF, native.DFF,
                               own.info.P, own.info.I, own.info.D, own.info.FF, own.info.DFF
                              };
        memcpy(sample.pid[axis], terms, sizeof(terms));
        const float values[] = {rate.input.gyro[axis], rate.input.target[axis], engine.consumed_target()[axis],
                                rate.expected.control[axis], engine.rate_engine().output().control[axis],
                                rate.expected.feedforward[axis], engine.rate_engine().output().feedforward[axis]
                               };
        memcpy(sample.rate[axis], values, sizeof(values));
        sample.flags[axis] = 3U | (uint16_t(rate.input.limited[axis]) << 2) |
                             (uint16_t(rate.before[axis].i_set) << 3) | (uint16_t(rate.before[axis].reset_filter) << 4) |
                             (uint16_t(!rate.before[axis].i_set && !is_zero(own.integrator)) << 5);
    }
    const Quaternion quaternions[] = {source.sensors.attitude, source.after.target, target.target};
    for (uint8_t i = 0; i < 3; i++) {
        sample.attitude[i*4] = quaternions[i].q1; sample.attitude[i*4+1] = quaternions[i].q2;
        sample.attitude[i*4+2] = quaternions[i].q3; sample.attitude[i*4+3] = quaternions[i].q4;
    }
    return true;
}
void AP_UTB_FlightBin::sample(const UTB_ChainShadowEngine &engine, uint32_t loop, uint64_t now_us)
{
    Sample value; // One local selected sample; queue is never a live engine reference.
    uint32_t seq;
    if (engine.statistics().attempts % (400U/_hz) != 0) {
        return;
    }
    seq = _requested.fetch_add(1, std::memory_order_relaxed) + 1U;
    if (!make_sample(engine, loop, seq, now_us, value)) {
        _invalid.fetch_add(1, std::memory_order_relaxed); return;
    }
    enqueue(value);
}
void AP_UTB_FlightBin::enqueue(const Sample &value)
{
    uint32_t depth; // Conservative producer occupancy; the consumer can only reduce it.
    if (!_queue.push(value)) {
        _dropped.fetch_add(1, std::memory_order_relaxed); return;
    }
    _queued.fetch_add(1, std::memory_order_relaxed);
    depth = _queue.producer_depth();
    if (depth > _hwm.load(std::memory_order_relaxed)) {
        _hwm.store(depth, std::memory_order_relaxed);
    }
}
AP_UTB_FlightBin::Counts AP_UTB_FlightBin::counts() const
{
    return {_requested.load(), _queued.load(), _accepted.load(), _dropped.load(), _invalid.load(),
            _unsupported.load(), _rejected.load(), _hwm.load(), _queue.depth()};
}
void AP_UTB_FlightBin::dump(ExpandingString &text) const
{
    const bool finished = _final_sent.load(std::memory_order_acquire);
    const Counts c = counts(); // Scalar counters remain readable when the backend rejects every status packet.
    text.printf("bin,state=%u,reason=%u,finished=%u,requested=%u,queued=%u,accepted=%u,dropped=%u,invalid=%u,unsupported=%u,rejected=%u,hwm=%u,depth=%u\n",
                unsigned(state()), unsigned(_reason.load()), unsigned(finished), unsigned(c.requested),
                unsigned(c.queued), unsigned(c.accepted), unsigned(c.dropped), unsigned(c.invalid),
                unsigned(c.unsupported), unsigned(c.rejected), unsigned(c.hwm), unsigned(c.depth));
}
void AP_UTB_FlightBin::worker(AP_Logger &logger, bool available)
{
    Sample value; // Consumer copy is released from queue before any blocking backend call.
    bool complete;
    uint32_t now_ms;
    const State current = state();
    if (current == State::IDLE || _final_sent) {
        return;
    }
    for (uint8_t i = 0; i < Queue::CAPACITY && _queue.pop(value); i++) {
        complete = available;
        if (available) {
            complete = UTBFlightLog::write_sample(value, logger);
        }
        if (complete) {
            _accepted.fetch_add(1, std::memory_order_relaxed);
        } else {
            _rejected.fetch_add(1, std::memory_order_relaxed);
        }
    }
    now_ms = AP_HAL::millis();
    if (!_status_sent || now_ms-_last_status_ms >= 1000U || (current == State::STOPPED && _queue.depth() == 0)) {
        const Counts c = counts(); // Atomic scalar snapshots; intermediate counters may span producer updates.
        const UTBFlightLog::Status status = {LOG_PACKET_HEADER_INIT(LOG_UFLS_MSG), AP_HAL::micros64(),_start_us,1,
                                             uint8_t(current),uint8_t(_reason.load()),_hz,c.requested,c.queued,c.accepted,c.dropped,c.invalid,c.unsupported,
                                             c.rejected,uint8_t(c.hwm),uint8_t(c.depth),_duration
                                            };
        if (available) {
            logger.WriteBlock_first_succeed(&status, sizeof(status));
        }
        _status_sent = true; _last_status_ms = now_ms;
        if (current == State::STOPPED && c.depth == 0) {
            _final_sent = true;
        }
    }
}
#endif
