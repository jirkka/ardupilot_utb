// GNU GPL v3 or later; native AP algorithms retain their original copyright.
/// @file AP_UTB_NativeShadow.cpp
/// @brief Main-only capture and independent histories; no logger, motor, HAL output or blocking API.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-21
/// @date 2026-10-21
#include "AP_UTB_NativeShadow.h"
#if AP_UTB_NATIVE_SHADOW_AVAILABLE
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include <cstring>
#include <type_traits>
extern const AP_HAL::HAL &hal;

// Local functions: exact scalar bits, never object padding or pointer-bearing objects.
static bool same_float(float a, float b)
{
    return memcmp(&a, &b, sizeof(float)) == 0;
}
static bool same_config(const AC_PID_Shadow &a, const AC_PID_Shadow &b)
{
    for (uint8_t i = 0; i < AC_PID_Shadow::CONFIG_COUNT; i++) {
        if (!same_float(a.config[i], b.config[i])) {
            return false;
        }
    }
    return a.slew_scale == b.slew_scale && a.notch == b.notch;
}
static void values(const AC_PID_Shadow &s, float output, float ff, float (&v)[UTB_RateShadow::METRICS])
{
    // target/error LPF and both derivatives are the complete relevant filter history without notch.
    const float all[] = {s.target, s.error, s.derivative, s.target_derivative, s.integrator,
                         s.info.P, s.info.I, s.info.D, s.info.FF, s.info.DFF, s.info.Dmod,
                         output, ff, float(s.info.limit), float(s.info.PD_limit), float(s.info.reset),
                         float(s.info.I_term_set), s.info.actual
                        };
    for (uint8_t i = 0; i < UTB_RateShadow::METRICS; i++) {
        v[i] = all[i];
    }
}
static bool finite_state(const AC_PID_Shadow &s)
{
    float v[UTB_RateShadow::METRICS]; // All compared scalar state, excluding inactive slew telemetry.
    values(s, 0, 0, v);
    for (float x : v) {
        if (!isfinite(x)) {
            return false;
        }
    }
    for (float x : s.config) {
        if (!isfinite(x)) {
            return false;
        }
    }
    return true;
}

// Local scope records every attempt, including rejected inputs and unsupported profiles.
class ShadowTiming
{
public:
    ShadowTiming(UTB_RateShadow::Statistics &stats, uint32_t capture_us) :
        _stats(stats), _start(AP_HAL::micros()), _capture_us(capture_us) {}
    ~ShadowTiming()
    {
        const uint32_t elapsed = AP_HAL::micros() - _start;
        _stats.consume_sum_us += elapsed;
        _stats.consume_max_us = MAX(_stats.consume_max_us, elapsed);
        _stats.capture_sum_us += _capture_us;
        _stats.capture_max_us = MAX(_stats.capture_max_us, _capture_us);
    }
private:
    UTB_RateShadow::Statistics &_stats;
    const uint32_t _start, _capture_us;
};

// Public engine functions. A start is allowed only before the wrapper publishes an immutable session.
void UTB_RateShadow::start(bool continuous)
{
    // Numeric, trivially-copyable statistics only; no PID/filter/atomic object is cleared.
    static_assert(std::is_trivially_copyable<Statistics>::value, "Statistics must remain plain values");
    memset(static_cast<void *>(&_stats), 0, sizeof(_stats));
    _stats.period_min_us = UINT32_MAX;
    _continuous = continuous;
    _synchronized = false;
    _last_seq = 0;
    _last_mode = 0; _last_gyro = 0;
}
bool UTB_RateShadow::same_history(const AC_PID_Shadow &a, const AC_PID_Shadow &b)
{
    return same_float(a.integrator, b.integrator) && same_float(a.target, b.target) && same_float(a.error, b.error) &&
           same_float(a.derivative, b.derivative) && same_float(a.target_derivative, b.target_derivative) &&
           a.reset_filter == b.reset_filter && a.i_set == b.i_set && same_float(a.info.P, b.info.P) && same_float(a.info.D, b.info.D);
}
void UTB_RateShadow::process(const Frame &frame)
{
    // Fixed local storage; timings are AP_HAL wall time, never interpreted as hardware timings in SITL.
    uint32_t math_start_us, math_us;
    uint32_t smax = 0, notch = 0;
    bool valid = true, config_changed = false, history_equal = true, different = false;
    AC_PID_Shadow own;
    UTB_Rate::ShadowOutput output;
    float actual[METRICS], expected[METRICS];
    double error;
    ShadowTiming timing(_stats, frame.capture_us);
    _stats.seen++;
    if (_stats.seen == 1) {
        _stats.first_seq = frame.seq;
        _stats.first_us = frame.time_us;
    }
    if (_stats.seen > 1) {
        // Rate-entry period is distinct from native scheduler dt and computation wall spans.
        const uint32_t period = frame.time_us - _stats.last_us;
        _stats.period_count++;
        _stats.period_sum_us += period;
        _stats.period_min_us = MIN(_stats.period_min_us, period);
        _stats.period_max_us = MAX(_stats.period_max_us, period);
        _stats.jitter_sum_us += period >= 2500U ? period - 2500U : 2500U - period;
    }
    _stats.last_seq = frame.seq; _stats.last_us = frame.time_us;
    if (_stats.seen > 1 && frame.mode != _last_mode) {
        _stats.mode_changes++;
    }
    _last_mode = frame.mode;
    if (!frame.execution_supported) {
        _stats.unsupported++; _stats.unsupported_execution++; _synchronized = false; _last_seq = frame.seq;
        return;
    }
    if (!frame.captured || !frame.completed || (_stats.seen > 1 && frame.seq != _last_seq + 1U)) {
        _stats.desync++; _synchronized = false; _last_seq = frame.seq;
        return;
    }
    _last_seq = frame.seq;
    if (_synchronized && frame.gyro_instance != _last_gyro) {
        _stats.desync++; _stats.config_changes++; _synchronized = false;
        _last_gyro = frame.gyro_instance; _last_seq = frame.seq;
        return;
    }
    _last_gyro = frame.gyro_instance;
    valid = isfinite(frame.input.dt) && frame.input.dt > 0 && isfinite(frame.input.yaw_ff_scale);
    for (uint8_t axis = 0; axis < 3; axis++) {
        valid &= isfinite(frame.input.target[axis]) && isfinite(frame.input.gyro[axis]) &&
                 isfinite(frame.input.pd_scale[axis]) && isfinite(frame.input.i_scale[axis]) &&
                 isfinite(frame.input.actuator[axis]) && finite_state(frame.before[axis]) && finite_state(frame.after[axis]) &&
                 isfinite(frame.expected.control[axis]) && isfinite(frame.expected.feedforward[axis]);
        valid &= frame.before[axis].config[AC_PID_Shadow::SRT] > 0;
        smax |= fpclassify(frame.before[axis].config[AC_PID_Shadow::SMAX]) != FP_ZERO;
        notch |= frame.before[axis].notch;
        // Malformed filter/gain configuration is diagnosed, never repaired in AP.
        for (uint8_t k = AC_PID_Shadow::IMAX; k <= AC_PID_Shadow::FLTD; k++) {
            valid &= frame.before[axis].config[k] >= 0;
        }
    }
    if (!valid) {
        _stats.invalid++;
        _synchronized = false;
        return;
    }
    if (smax || notch) {
        _stats.unsupported++; _stats.unsupported_smax += smax; _stats.unsupported_notch += notch;
        _synchronized = false; return;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
        if (!same_config(frame.before[axis], frame.after[axis])) {
            config_changed = true;
        }
        if (_synchronized && !same_config(frame.before[axis], _pid[axis].shadow_snapshot())) {
            config_changed = true;
        }
    }
    if (config_changed) {
        _stats.config_changes++; _stats.desync++; _synchronized = false; return;
    }
    if (!_continuous || !_synchronized) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            _pid[axis].shadow_restore(frame.before[axis]);
        }
        _stats.synchronizations++;
        _synchronized = true;
    } else {
        for (uint8_t axis = 0; axis < 3; axis++) {
            // Replay only real native external state events, identified by AC_PID's pending flags.
            // No integrator import on ordinary continuous cycles. Count every replay for qualification.
            if (frame.before[axis].i_set) {
                _pid[axis].set_integrator(frame.before[axis].integrator); _stats.i_events++;
            }
            if (frame.before[axis].reset_filter) {
                _pid[axis].reset_filter();
                _stats.filter_resets++;
            }
            own = _pid[axis].shadow_snapshot();
            history_equal &= same_history(frame.before[axis], own);
        }
        if (!history_equal) {
            _stats.desync++;
            _synchronized = false;
            return;
        }
    }
    math_start_us = AP_HAL::micros();
    output = UTB_Rate::shadow_rate_step(_pid, frame.input);
#if AP_UTB_CHAIN_SHADOW_ENABLED
    _output = output;
#endif
    math_us = AP_HAL::micros() - math_start_us;
    // Reject the entire diagnostic result before accumulating any metrics.
    for (uint8_t axis = 0; axis < 3; axis++) {
        valid &= finite_state(_pid[axis].shadow_snapshot()) && isfinite(output.control[axis]) && isfinite(output.feedforward[axis]);
    }
    if (!valid) {
        _stats.invalid++;
        _synchronized = false;
        return;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
        own = _pid[axis].shadow_snapshot();
        values(own, output.control[axis], output.feedforward[axis], actual);
        values(frame.after[axis], frame.expected.control[axis], frame.expected.feedforward[axis], expected);
        for (uint8_t k = 0; k < METRICS; k++) {
            if (!isfinite(actual[k])) {
                valid = false;
                continue;
            }
            error = double(actual[k]) - double(expected[k]);
            _stats.maximum[axis][k] = MAX(_stats.maximum[axis][k], fabs(error));
            _stats.squares[axis][k] += error * error;
            different |= fpclassify(error) != FP_ZERO;
        }
    }
    if (!valid) {
        _stats.invalid++;
        _synchronized = false;
    } else {
        _stats.valid++;
        if (different) {
            _stats.mismatch++;
            if (_stats.mismatch == 1) {
                _stats.first_difference = frame.seq;
            }
            _synchronized = false;
        }
    }
    _stats.math_sum_us += math_us; _stats.math_max_us = MAX(_stats.math_max_us, math_us);
}

AP_UTB_NativeShadow *AP_UTB_NativeShadow::_singleton;
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Shadow publication must be lock-free");
const AP_Param::GroupInfo AP_UTB_NativeShadow::var_info[] = {
    // @Param: RUN
    // @DisplayName: Native rate shadow finite RAM session
    // @Description: Starts one session per boot. One-step imports native control-relevant PID history each sample; continuous imports only initial history and real external reset events. Never writes to flight controllers or motors. Export only after stop. Always cleared at boot.
    // @Values: 0:Off,1:One step,2:Continuous
    // @User: Advanced
    AP_GROUPINFO("RUN", 0, AP_UTB_NativeShadow, _run, 0),
    // @Param: COUNT
    // @DisplayName: Native rate shadow sample count
    // @Description: Main-loop attempts in one finite RAM-only window. The upper bound limits diagnostic accumulators; it is not a flight safety limit. Unsupported and skipped attempts are included.
    // @Range: 1 200000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("COUNT", 1, AP_UTB_NativeShadow, _count, 4000),
    AP_GROUPEND
};
AP_UTB_NativeShadow::AP_UTB_NativeShadow()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}
void AP_UTB_NativeShadow::init()
{
    _run.set(0); _initialized = true;
}
void AP_UTB_NativeShadow::freeze(uint8_t reason)
{
    _stop_reason = reason; _capture = false; _running = false; _run.set(0);
    _published.store(true, std::memory_order_release);
}
void AP_UTB_NativeShadow::prepare(bool execution_supported, uint8_t mode)
{
    if (!_initialized) {
        return;
    }
    if (_published.load(std::memory_order_relaxed)) {
        // A second request cannot overwrite a snapshot being read by another thread.
        if (_run.get() != 0) {
            _run.set(0);
        }
        return;
    }
    if (_waiting) {
        // A missing consume/audit must not silently overwrite a prepared native step.
        freeze(7);
        return;
    }
    if (!_running) {
        if (_run.get() == 0) {
            return;
        }
        if ((_run.get() != 1 && _run.get() != 2) || _count.get() < 1 || _count.get() > 200000) {
            freeze(4);
            return;
        }
        _engine.start(_run.get() == 2); _limit = uint32_t(_count.get()); _running = true;
    }
    if (_run.get() == 0) {
        freeze(2);
        return;
    }
    // Every supported capture overwrites all input/pre/post values before completed=true.
    // Rejected/missing captures read only these validity markers, never stale payload.
    _frame.captured = false;
    _frame.completed = false;
    _frame.capture_us = 0;
    _frame.seq = ++_seq; _frame.time_us = AP_HAL::micros(); _frame.mode = mode;
    _frame.execution_supported = execution_supported;
    _waiting = true;
    _capture = execution_supported;
}
void AP_UTB_NativeShadow::before(const UTB_Rate::ShadowInput &input, uint32_t gyro_time_us, uint32_t imu_time_us, uint8_t gyro_instance)
{
    _capture_started_us = AP_HAL::micros();
    _frame.input = input; _frame.gyro_time_us = gyro_time_us; _frame.imu_time_us = imu_time_us;
    _frame.gyro_instance = gyro_instance; _frame.captured = true;
}
void AP_UTB_NativeShadow::pre_axis(uint8_t axis, const AC_PID_Shadow &state)
{
    _frame.before[axis] = state;
}
void AP_UTB_NativeShadow::post_axis(uint8_t axis, const AC_PID_Shadow &state, float output, float ff)
{
    _frame.after[axis] = state; _frame.expected.control[axis] = output; _frame.expected.feedforward[axis] = ff;
}
void AP_UTB_NativeShadow::complete()
{
    // This wall span includes native AP PID work; labelled separately from additional capture overhead.
    _frame.capture_us = AP_HAL::micros() - _capture_started_us;
    _frame.completed = true; _capture = false;
}
void AP_UTB_NativeShadow::consume(const UTB_Rate::ShadowOutput &native_before)
{
    if (!_running) {
        return;
    }
    _native_before = native_before;
    _engine.process(_frame);
    if (_engine.statistics().mismatch != 0) {
        _pending_stop = 3;
    } else if (_engine.statistics().consume_max_us >= 2500U) {
        _pending_stop = 5;
    } else if (_engine.statistics().seen >= _limit) {
        _pending_stop = 1;
    }
}
void AP_UTB_NativeShadow::finish_motor_audit(const UTB_Rate::ShadowOutput &native_after)
{
    if (!_running) {
        return;
    }
    _waiting = false;
    _motor_checks++;
    for (uint8_t axis = 0; axis < 3; axis++) {
        if (!same_float(_native_before.control[axis], native_after.control[axis]) ||
            !same_float(_native_before.feedforward[axis], native_after.feedforward[axis])) {
            _motor_changes++; _pending_stop = 6;
        }
    }
    // Publish only after the read-only AP command audit has completed.
    if (_pending_stop != 0) {
        freeze(_pending_stop);
    }
}
void AP_UTB_NativeShadow::finish_unsupported()
{
    if (!_running) {
        return;
    }
    _waiting = false;
    _motor_skipped++;
    if (_pending_stop != 0) {
        freeze(_pending_stop);
    }
}
void AP_UTB_NativeShadow::dump(ExpandingString &text)
{
    const AP_UTB_NativeShadow *self = _singleton;
    if (self == nullptr || !self->_published.load(std::memory_order_acquire)) {
        text.printf("UTBNativeShadowV1,pending\n"); return;
    }
    if (hal.util->get_soft_armed()) {
        text.printf("UTBNativeShadowV1,armed-unavailable\n");
        return;
    }
    const auto &s = self->_engine.statistics();
    text.printf("UTBNativeShadowV1,frozen,reason=%u,continuous=%u\n", unsigned(self->_stop_reason), unsigned(self->_engine.continuous()));
    // No 64-bit live reads: publication precedes every reader and the snapshot is never reused.
#define STAT(name) text.printf(#name ",%u\n", unsigned(s.name))
    STAT(seen); STAT(valid); STAT(invalid); STAT(desync); STAT(unsupported); STAT(mismatch);
    STAT(synchronizations); STAT(config_changes); STAT(filter_resets); STAT(i_events); STAT(mode_changes);
    STAT(unsupported_execution); STAT(unsupported_notch); STAT(unsupported_smax);
    STAT(first_seq); STAT(last_seq); STAT(first_us); STAT(last_us); STAT(first_difference);
    STAT(period_count); STAT(period_min_us); STAT(period_max_us);
    text.printf("period_sum_us,%llu\njitter_sum_us,%llu\n",
                static_cast<unsigned long long>(s.period_sum_us), static_cast<unsigned long long>(s.jitter_sum_us));
    STAT(math_max_us); STAT(consume_max_us); STAT(capture_max_us);
#undef STAT
    text.printf("timing_sum_us,math=%llu,consume=%llu,native_and_capture=%llu\n",
                static_cast<unsigned long long>(s.math_sum_us), static_cast<unsigned long long>(s.consume_sum_us),
                static_cast<unsigned long long>(s.capture_sum_us));
    text.printf("native_motor_checks,%u\nnative_motor_changes,%u\n", unsigned(self->_motor_checks), unsigned(self->_motor_changes));
    text.printf("native_motor_skipped,%u\n", unsigned(self->_motor_skipped));
    text.printf("prepared_attempts,%u\nunconsumed_attempts,%u\n", unsigned(self->_seq),
                unsigned(self->_seq - s.seen));
    text.printf("last_input,seq=%u,mode=%u,captured=%u,complete=%u,dt=%.9g,imu_us=%u,gyro_read_us=%u,gyro_index=%u\n",
                unsigned(self->_frame.seq), unsigned(self->_frame.mode), unsigned(self->_frame.captured),
                unsigned(self->_frame.completed), double(self->_frame.captured ? self->_frame.input.dt : NAN),
                unsigned(self->_frame.imu_time_us), unsigned(self->_frame.gyro_time_us), unsigned(self->_frame.gyro_instance));
    text.printf("axis,metric,max_abs,rmse\n");
    for (uint8_t axis = 0; axis < 3; axis++) {
        for (uint8_t k = 0; k < UTB_RateShadow::METRICS; k++) {
            text.printf("%u,%u,%.9g,%.9g\n", unsigned(axis), unsigned(k), s.valid ? double(s.maximum[axis][k]) : double(NAN),
                        s.valid ? sqrt(s.squares[axis][k] / s.valid) : double(NAN));
        }
    }
}
#endif
