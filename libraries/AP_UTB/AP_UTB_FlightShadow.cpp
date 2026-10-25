// GNU GPL v3 or later; original ArduPilot algorithms retain their copyright.
/// @file AP_UTB_FlightShadow.cpp
/// @brief Fixed RAM sampling and frozen paged export; no persistent logging in this phase.
/// @author Jiri Kovar
/// @version 2026-10-25
#include "AP_UTB_FlightShadow.h"
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
#include "AP_UTB_ChainShadow.h"
#include <AP_Common/ExpandingString.h>
#include <AP_HAL/AP_HAL.h>
#include <cstring>

// Local value helpers: no live controller or motor interface is accepted.
static void copy_quaternion(float (&destination)[4], const Quaternion &value)
{
    destination[0] = value.q1; destination[1] = value.q2;
    destination[2] = value.q3; destination[3] = value.q4;
}
static void dump_float(ExpandingString &text, float value)
{
    uint32_t bits; // Exact binary32 avoids platform-specific decimal formatter precision limits.
    memcpy(&bits, &value, sizeof(bits));
    text.printf(",%08x", unsigned(bits));
}
template<size_t N> static void dump_values(ExpandingString &text, const float (&values)[N])
{
    for (float value : values) {
        dump_float(text, value);
    }
}
static void dump_pid(ExpandingString &text, const float (&values)[3][5])
{
    for (const auto &axis : values) {
        dump_values(text, axis);
    }
}
static_assert(sizeof(AP_UTB_FlightShadow::Record) <= 320, "Fixed recorder RAM budget exceeded");

const AP_Param::GroupInfo AP_UTB_FlightShadow::var_info[] = {
    // @Param: RUN
    // @DisplayName: Passive native STABILIZE RAM recording
    // @Description: One bounded session per boot, shared with UCS_RUN. Native AP retains all control. Export only frozen and disarmed. No persistent DataFlash recording. Cleared at boot.
    // @Values: 0:Off,1:Record
    // @User: Advanced
    AP_GROUPINFO("RUN", 0, AP_UTB_FlightShadow, _run, 0),
    // @Param: COUNT
    // @DisplayName: Passive flight main loop attempts
    // @Description: Fixed numerical window including warmup, at nominal 400 Hz. All supported steps are compared regardless of record decimation. Mode departure or a diagnostic fault ends the session.
    // @Range: 2 48000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("COUNT", 1, AP_UTB_FlightShadow, _count, 24000),
    // @Param: DECIM
    // @DisplayName: RAM record main loop decimation
    // @Description: Store one actual step every N main attempts, plus a terminal fault sample. Default 200 gives 2 Hz at 400 Hz. Fixed 128-row ring overwrites oldest diagnostic rows and counts every overwrite. Decimated records cannot reconstruct full-rate dynamics.
    // @Range: 1 400
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DECIM", 2, AP_UTB_FlightShadow, _decim, 200),
    AP_GROUPEND
};
AP_UTB_FlightShadow::AP_UTB_FlightShadow()
{
    AP_Param::setup_object_defaults(this, var_info);
}
void AP_UTB_FlightShadow::init()
{
    _run.set(0);
}
bool AP_UTB_FlightShadow::start()
{
    if (_started || _closed || _run.get() != 1 || _count.get() < 2 || _count.get() > 48000 ||
        _decim.get() < 1 || _decim.get() > 400) {
        return false;
    }
    _limit = uint32_t(_count.get());
    _stride = uint16_t(_decim.get());
    _start_us = AP_HAL::micros64();
    _stats.epochs = 1;
    _started = true;
    return true;
}
void AP_UTB_FlightShadow::stop(uint8_t reason)
{
    _reason = reason;
    _stop_us = AP_HAL::micros64();
    _run.set(0);
    _closed = true;
}
void AP_UTB_FlightShadow::context(bool armed, uint8_t spool)
{
    if (_closed) {
        return;
    }
    if (_started && _armed != armed) {
        _stats.epochs++;
        _stats.arm_changes++;
    }
    _armed = armed;
    _spool = spool;
}
void AP_UTB_FlightShadow::record(const UTB_ChainShadowEngine &engine, uint32_t loop, bool force)
{
    uint32_t started; // Monotonic wall-time bracket for this recorder, not the native controller.
    AP_PIDInfo own; // Value copy of the already-computed independent UTB PID information.
    if (!_started || _closed) {
        return;
    }
    started = AP_HAL::micros();
    _stats.candidates++;
    if (!force && (_stats.candidates - 1U) % _stride != 0) {
        _stats.decimated++;
    } else {
        const auto &s = engine.statistics();
        const auto &rate = engine.last_rate();
        const bool paired = s.rate_loop == loop && s.rate_calls != 0;
        const auto &source = paired ? engine.last_attitude() : engine.last_operation();
        const auto &attitude = paired ? engine.consumed_attitude() : engine.attitude_engine().history();
        Record &row = _records[_head];
        row = {};
        row.id = ++_stats.selected;
        row.loop = loop;
        row.source_loop = paired ? s.source_loop : loop;
        row.time_us = paired ? rate.time_us : source.time_us;
        row.source_us = source.time_us;
        row.epoch = _stats.epochs;
        row.resets_at_end = engine.attitude_engine().statistics().resets;
        row.mode = source.mode;
        row.spool = _spool;
        row.flags = uint8_t(paired) | (uint8_t(force) << 1) | (uint8_t(_armed) << 2);
        row.dt = paired ? rate.input.dt : source.config.dt;
        copy_quaternion(row.attitude, source.sensors.attitude);
        copy_quaternion(row.native_attitude_target, source.after.target);
        copy_quaternion(row.own_attitude_target, attitude.target);
        for (uint8_t axis = 0; axis < 3; axis++) {
            row.command[axis] = source.command[axis];
            if (!paired) {
                continue; // Undefined rate fields remain zero with has_rate=0; never plotted as valid data.
            }
            row.gyro[axis] = rate.input.gyro[axis];
            row.native_target[axis] = rate.input.target[axis];
            row.own_target[axis] = engine.consumed_target()[axis];
            own = engine.rate_engine().history(axis).info;
            const auto &native = rate.after[axis].info;
            const float native_terms[] = {native.P, native.I, native.D, native.FF, native.DFF};
            const float own_terms[] = {own.P, own.I, own.D, own.FF, own.DFF};
            for (uint8_t term = 0; term < 5; term++) {
                row.native_pid[axis][term] = native_terms[term];
                row.own_pid[axis][term] = own_terms[term];
            }
            row.native_output[axis] = rate.expected.control[axis];
            row.own_output[axis] = engine.rate_engine().output().control[axis];
            row.native_ff_output[axis] = rate.expected.feedforward[axis];
            row.own_ff_output[axis] = engine.rate_engine().output().feedforward[axis];
            row.limits |= uint8_t(rate.input.limited[axis]) << axis;
            row.reset_flags |= uint8_t(rate.before[axis].reset_filter) << axis;
            row.reset_flags |= uint8_t(rate.before[axis].i_set) << (axis + 3);
        }
        _head = (_head + 1U) % CAPACITY;
        if (_used < CAPACITY) {
            _used++;
        } else {
            _stats.overwritten++;
        }
    }
    started = AP_HAL::micros() - started;
    _stats.record_sum_us += started;
    _stats.record_max_us = MAX(_stats.record_max_us, started);
}
const AP_UTB_FlightShadow::Record &AP_UTB_FlightShadow::oldest(uint16_t index) const
{
    return _records[(_head + CAPACITY - _used + index) % CAPACITY];
}
void AP_UTB_FlightShadow::dump_header(ExpandingString &text) const
{
    text.printf("UTBFlightV1,frozen,reason=%u,start_us=%llu,stop_us=%llu\n", unsigned(_reason),
                (unsigned long long)_start_us, (unsigned long long)_stop_us);
    text.printf("storage,record_bytes=%u,capacity=%u,rows=%u,pages=8,stride=%u,limit=%u,persistent=0\n",
                unsigned(sizeof(Record)), unsigned(CAPACITY), unsigned(_used), unsigned(_stride), unsigned(_limit));
    text.printf("records,candidates=%u,selected=%u,decimated=%u,overwritten=%u\n",
                unsigned(_stats.candidates), unsigned(_stats.selected), unsigned(_stats.decimated), unsigned(_stats.overwritten));
    text.printf("context,epochs=%u,arm_changes=%u,mode_exits=%u,unsupported=%u\n", unsigned(_stats.epochs),
                unsigned(_stats.arm_changes), unsigned(_stats.mode_exits), unsigned(_stats.unsupported));
    text.printf("record_timing,sum_us=%llu,max_us=%u\n", (unsigned long long)_stats.record_sum_us, unsigned(_stats.record_max_us));
}
void AP_UTB_FlightShadow::dump_page(ExpandingString &text, uint8_t page) const
{
    uint16_t first; // Fixed bounded page; text allocation is only in the frozen reader.
    if (page >= CAPACITY / PAGE_ROWS) {
        return;
    }
    text.printf("UTBFlightRowsV1,start_us=%llu,page=%u,rows=%u,selected=%u,overwritten=%u,float_bits=32\n",
                (unsigned long long)_start_us, unsigned(page), unsigned(_used), unsigned(_stats.selected), unsigned(_stats.overwritten));
    text.printf("Id,Loop,Source,TimeUS,SourceUS,Epoch,Resets,Mode,Spool,Flags,Limits,ResetFlags,Dt");
    for (const char *prefix : {
             "Q", "AQT", "UQT"
         }) {
        for (uint8_t q = 0; q < 4; q++) {
            text.printf(",%s%u", prefix, unsigned(q));
        }
    }
    for (const char *prefix : {
             "Cmd", "Gyro", "ATar", "UTar"
         }) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            text.printf(",%s%u", prefix, unsigned(axis));
        }
    }
    for (const char *prefix : {
             "AP", "UTB"
         }) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            for (const char *term : {
                     "P", "I", "D", "FF", "DFF"
                 }) {
                text.printf(",%s%u%s", prefix, unsigned(axis), term);
            }
        }
    }
    for (const char *prefix : {
             "AOut", "UOut", "AFFOut", "UFFOut"
         }) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            text.printf(",%s%u", prefix, unsigned(axis));
        }
    }
    text.printf("\n");
    first = uint16_t(page) * PAGE_ROWS;
    for (uint16_t index = first; index < MIN(uint16_t(first + PAGE_ROWS), _used); index++) {
        const Record &row = oldest(index);
        text.printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", unsigned(row.id), unsigned(row.loop),
                    unsigned(row.source_loop), unsigned(row.time_us), unsigned(row.source_us), unsigned(row.epoch),
                    unsigned(row.resets_at_end), unsigned(row.mode), unsigned(row.spool), unsigned(row.flags),
                    unsigned(row.limits), unsigned(row.reset_flags));
        dump_float(text, row.dt);
        dump_values(text, row.attitude);
        dump_values(text, row.native_attitude_target);
        dump_values(text, row.own_attitude_target);
        dump_values(text, row.command);
        dump_values(text, row.gyro);
        dump_values(text, row.native_target);
        dump_values(text, row.own_target);
        dump_pid(text, row.native_pid);
        dump_pid(text, row.own_pid);
        dump_values(text, row.native_output);
        dump_values(text, row.own_output);
        dump_values(text, row.native_ff_output);
        dump_values(text, row.own_ff_output);
        text.printf("\n");
    }
}
#endif // AP_UTB_FLIGHT_SHADOW_AVAILABLE
