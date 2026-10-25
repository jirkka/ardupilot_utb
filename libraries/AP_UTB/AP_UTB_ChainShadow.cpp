// GNU GPL v3 or later; original ArduPilot algorithms retain their copyright.
/// @file AP_UTB_ChainShadow.cpp
/// @brief Finite main-thread chain comparison without a motor sink.
/// @author Jiri Kovar
/// @version 2026-10-24
#include "AP_UTB_ChainShadow.h"
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
#include <cstring>
// HAL printf formats floats as binary32; export binary64 metric bits without narrowing.
static void flight_metric(ExpandingString &text, const char *section, uint8_t axis, uint8_t metric,
                          double maximum, double squares, uint32_t count)
{
    const double rmse = count ? sqrt(squares / count) : double(NAN);
    uint64_t maximum_bits, rmse_bits; // Scalar bit representations, not pointers or object padding.
    static_assert(sizeof(double) == sizeof(uint64_t), "Metric bit export requires binary64");
    memcpy(&maximum_bits, &maximum, sizeof(maximum_bits));
    memcpy(&rmse_bits, &rmse, sizeof(rmse_bits));
    text.printf("flight_metric64,%s,%u,%u,%08x%08x,%08x%08x\n", section, unsigned(axis), unsigned(metric),
                unsigned(maximum_bits >> 32), unsigned(uint32_t(maximum_bits)),
                unsigned(rmse_bits >> 32), unsigned(uint32_t(rmse_bits)));
}
#endif
extern const AP_HAL::HAL &hal;
using namespace UTBAttitudeCapture;

void UTB_ChainShadowEngine::start()
{
    _stats = {}; _attitude.start(true); _rate.start(true);
    _available = false; _open = false; _rate_seq = 0; _att_seq = 0;
}
void UTB_ChainShadowEngine::begin(uint32_t loop, uint8_t mode, bool supported)
{
    if (_open) {
        _stats.missing++;
    }
    _loop = loop; _mode = mode; _supported = supported && mode <= 1;
    _primary = 0; _rate_calls = 0; _open = true; _stats.attempts++;
    if (!_supported) {
        _stats.unsupported++;
    }
}
void UTB_ChainShadowEngine::attitude(Frame frame)
{
    if (!_open || !_supported) {
        return;
    }
    frame.loop = _loop; frame.mode = _mode; frame.seq = ++_att_seq;
    _last_attitude = frame;
    _attitude.process(frame);
    _stats.command_peak = MAX(_stats.command_peak, frame.command.length());
    if (uint8_t(frame.kind) < uint8_t(Command::RESET)) {
        _primary++;
        if (_primary > 1) {
            _stats.duplicate++;
        }
        // This is the independently computed UTB state, never frame.after.
        _target = _attitude.history(); _source = _loop; _available = true;
    }
}
void UTB_ChainShadowEngine::rate(UTB_RateShadow::Frame frame, const Vector3f &sysid)
{
    uint32_t started;
    float actual[4], expected[4];
    double error;
    bool target_mismatch = false, independent_i = false, limited = false;
    if (!_open || !_supported) {
        return;
    }
    if (++_rate_calls != 1) {
        _stats.duplicate++; return;
    }
    if (!_available) {
        if (_stats.attempts == 1) {
            _stats.warmup++;
        } else {
            _stats.missing++;
        }
        return;
    }
    if (_loop != _source + 1U) {
        _stats.missing++; return;
    }
    _available = false;
    _stats.source_loop = _source; _stats.rate_loop = _loop;
    _last_rate = frame;
    _last_source = _last_attitude; _source_own = _target;
    _consumed = _target.body + sysid;
    for (uint8_t axis = 0; axis < 3; axis++) {
        actual[axis] = _consumed[axis]; expected[axis] = frame.input.target[axis];
    }
    actual[3] = _target.feedforward; expected[3] = frame.input.yaw_ff_scale;
    for (uint8_t k = 0; k < 4; k++) {
        if (!isfinite(actual[k]) || !isfinite(expected[k])) {
            _stats.invalid++; return;
        }
        error = double(actual[k]) - expected[k];
        _stats.target_max[k] = MAX(_stats.target_max[k], fabs(error));
        _stats.target_squares[k] += error * error;
        target_mismatch |= fpclassify(error) != FP_ZERO;
    }
    _stats.upstream_mismatch += target_mismatch;
    // The sole dataflow assignment into the independent PID. Native target is oracle only.
    frame.input.target = _consumed;
    frame.input.yaw_ff_scale = _target.feedforward;
    frame.seq = ++_rate_seq;
    started = AP_HAL::micros();
    _rate.process(frame);
    started = AP_HAL::micros() - started;
    _stats.rate_sum_us += started; _stats.rate_max_us = MAX(_stats.rate_max_us, started); _stats.rate_calls++;
    if (_rate.statistics().valid == _rate_seq) {
        _stats.paired++;
    } else {
        return;
    }
    _stats.downstream_mismatch += !outputs_equal(_rate.output(), frame.expected);
    _stats.gyro_peak = MAX(_stats.gyro_peak, frame.input.gyro.length());
    for (uint8_t axis = 0; axis < 3; axis++) {
        const AC_PID_Shadow own = _rate.history(axis);
        _stats.i_peak = MAX(_stats.i_peak, fabsf(own.integrator));
        _stats.ff_peak = MAX(_stats.ff_peak, fabsf(own.info.FF));
        _stats.dff_peak = MAX(_stats.dff_peak, fabsf(own.info.DFF));
        independent_i |= !frame.before[axis].i_set && fabsf(own.integrator) > 1.0e-5f;
        limited |= frame.input.limited[axis];
    }
    _stats.limited += limited; _stats.independent_i += independent_i;
}
void UTB_ChainShadowEngine::end()
{
    if (_supported && (_primary != 1 || _rate_calls != 1)) {
        _stats.missing++;
    }
    _open = false;
}
bool UTB_ChainShadowEngine::outputs_equal(const UTB_Rate::ShadowOutput &own, const UTB_Rate::ShadowOutput &native)
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        if (!isfinite(own.control[axis]) || !isfinite(own.feedforward[axis]) ||
            fpclassify(own.control[axis] - native.control[axis]) != FP_ZERO ||
            fpclassify(own.feedforward[axis] - native.feedforward[axis]) != FP_ZERO) {
            return false;
        }
    }
    return true;
}
void UTB_ChainShadowEngine::audit(bool changed)
{
    _stats.motor_checks++; _stats.motor_changes += changed;
}
bool UTB_ChainShadowEngine::failed() const
{
    const auto &a = _attitude.statistics();
    const auto &r = _rate.statistics();
    return _stats.missing || _stats.duplicate || _stats.invalid || _stats.motor_changes || _stats.upstream_mismatch ||
           a.invalid || a.desync || a.unsupported || r.invalid || r.desync || r.unsupported || r.mismatch;
}

AP_UTB_ChainShadow *AP_UTB_ChainShadow::_singleton;
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Immutable publication must be lock-free");
const AP_Param::GroupInfo AP_UTB_ChainShadow::var_info[] = {
    // @Param: RUN
    // @DisplayName: Integrated native chain RAM comparison
    // @Description: One finite continuous passive Attitude to Rate comparison per boot. No motor authority. Cleared at boot.
    // @Values: 0:Off,1:Continuous
    // @User: Advanced
    AP_GROUPINFO("RUN", 0, AP_UTB_ChainShadow, _run, 0),
    // @Param: COUNT
    // @DisplayName: Integrated chain main loop attempts
    // @Description: Finite capture window including the initial unpaired rate step and unsupported attempts.
    // @Range: 2 200000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("COUNT", 1, AP_UTB_ChainShadow, _count, 4000),
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    // @Group: F_
    // @Path: AP_UTB_FlightShadow.cpp
    AP_SUBGROUPINFO(_flight, "F_", 2, AP_UTB_ChainShadow, AP_UTB_FlightShadow),
#endif
    AP_GROUPEND
};
AP_UTB_ChainShadow::AP_UTB_ChainShadow()
{
    AP_Param::setup_object_defaults(this, var_info); _singleton = this;
}
void AP_UTB_ChainShadow::init()
{
    _run.set(0);
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    _flight.init();
#endif
    _initialized = true;
}
void AP_UTB_ChainShadow::freeze(uint8_t reason)
{
    _reason = reason; _running = false; _run.set(0);
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    if (_flight_session) {
        _flight.stop(reason);
    }
#endif
    _published.store(true, std::memory_order_release);
}
bool AP_UTB_ChainShadow::enabled()
{
    return hal.scheduler->in_main_thread() && _singleton != nullptr && _singleton->_running;
}
void AP_UTB_ChainShadow::prepare(bool supported, uint8_t mode)
{
    _loop++;
    if (!_initialized) {
        return;
    }
    if (_published.load(std::memory_order_relaxed)) {
        if (_run.get() != 0) {
            _run.set(0);
        }
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
        // Clear a new request without modifying the already published immutable recording.
        _flight.init();
#endif
        return;
    }
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    if (_running && ((_flight_session && _run.get() != 0) || (!_flight_session && _flight.requested()))) {
        _flight_session = true; freeze(4); return; // Conflicting diagnostics never change native control.
    }
    if (!_running && _flight.requested()) {
        _flight_session = true;
        if (_run.get() != 0 || !_flight.start()) {
            freeze(4); return;
        }
        _engine.start(); _limit = _flight.limit(); _running = true;
    }
    if (_flight_session && _running) {
        if (mode != 0) {
            _flight.mode_exit(); freeze(7); return;
        }
        if (!supported || !_flight_supported) {
            _flight.unsupported(); freeze(6); return;
        }
        if (!_flight.requested()) {
            freeze(2); return;
        }
        if (_flight.requested_run() != 1) {
            freeze(4); return;
        }
    }
#endif
    if (!_running && _run.get() != 0) {
        if (_run.get() != 1 || _count.get() < 2 || _count.get() > 200000) {
            freeze(4); return;
        }
        _engine.start(); _limit = uint32_t(_count.get()); _running = true;
    }
    if (_running) {
        if (_run.get() == 0
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
            && !_flight_session
#endif
           ) {
            freeze(2); return;
        }
        _engine.begin(_loop, mode, supported);
        _rate_frame.captured = false; _rate_frame.completed = false;
        _rate_frame.execution_supported = supported; _rate_frame.mode = mode;
    }
}
void AP_UTB_ChainShadow::before_rate(const UTB_Rate::ShadowInput &input, const Vector3f &sysid,
                                     uint32_t gyro_us, uint32_t imu_us, uint8_t gyro)
{
    _capture_start = AP_HAL::micros();
    _rate_frame.input = input; _sysid = sysid;
    _rate_frame.time_us = _capture_start;
    _rate_frame.gyro_time_us = gyro_us; _rate_frame.imu_time_us = imu_us;
    _rate_frame.gyro_instance = gyro; _rate_frame.captured = true;
}
void AP_UTB_ChainShadow::post_axis(uint8_t axis, const AC_PID_Shadow &s, float output, float ff)
{
    _rate_frame.after[axis] = s; _rate_frame.expected.control[axis] = output; _rate_frame.expected.feedforward[axis] = ff;
}
void AP_UTB_ChainShadow::complete_rate()
{
    _rate_frame.capture_us = AP_HAL::micros() - _capture_start; _rate_frame.completed = true;
    _engine.rate(_rate_frame, _sysid);
}
void AP_UTB_ChainShadow::before_attitude(Command kind, const Vector3f &command, bool reset,
        const Config &config, const Sensors &sensors, const History &history)
{
    _att_frame.kind = kind; _att_frame.command = command; _att_frame.reset_rate = reset;
    _att_frame.config = config; _att_frame.sensors = sensors; _att_frame.before = history;
    _att_frame.time_us = AP_HAL::micros();
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    if (_flight_session && uint8_t(kind) < uint8_t(Command::RESET) && kind != Command::STABILIZE) {
        _flight.unsupported(); // A mid-cycle change must not silently become a supported Flight profile.
    }
#endif
}
void AP_UTB_ChainShadow::after_attitude(const History &history)
{
    const uint32_t start = AP_HAL::micros();
    _att_frame.after = history;
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    if (_flight_session && _flight.statistics().unsupported != 0) {
        return;
    }
#endif
    _engine.attitude(_att_frame);
    _att_last_us = AP_HAL::micros() - start;
    _att_us += _att_last_us; _att_max_us = MAX(_att_max_us, _att_last_us);
}
void AP_UTB_ChainShadow::capture_time(bool post, uint32_t elapsed_us)
{
    if (post) {
        _post_us += elapsed_us >= _att_last_us ? elapsed_us - _att_last_us : 0;
    } else {
        _pre_us += elapsed_us;
    }
}
void AP_UTB_ChainShadow::finish()
{
    if (!_running) {
        return;
    }
    _engine.end();
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
    if (_flight_session) {
        const bool bad = _engine.failed() || _engine.statistics().downstream_mismatch ||
                         _engine.attitude_engine().statistics().mismatch;
        const bool unsupported = _flight.statistics().unsupported || _engine.statistics().unsupported ||
                                 _engine.attitude_engine().statistics().unsupported || _engine.rate_engine().statistics().unsupported;
        _flight.record(_engine, _loop, bad || unsupported);
        if (unsupported) {
            freeze(6); return;
        }
        if (bad) {
            freeze(3); return;
        }
    }
#endif
    if (_engine.statistics().unsupported || _engine.attitude_engine().statistics().unsupported ||
        _engine.rate_engine().statistics().unsupported) {
        freeze(6);
    } else if (_engine.failed()) {
        freeze(3);
    } else if (_engine.statistics().attempts >= _limit) {
        freeze(_engine.attitude_engine().statistics().mismatch ? 3 : 1);
    }
}
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
void AP_UTB_ChainShadow::dump_flight(ExpandingString &text, uint8_t page)
{
    const AP_UTB_ChainShadow *self = _singleton;
    if (self == nullptr || !self->_published.load(std::memory_order_acquire) || !self->_flight_session) {
        text.printf("UTBFlightV1,pending\n"); return;
    }
    if (hal.util->get_soft_armed()) {
        text.printf("UTBFlightV1,armed-unavailable\n"); return;
    }
    if (page != 255) {
        self->_flight.dump_page(text, page); return;
    }
    self->_flight.dump_header(text);
    dump(text); // Existing full accounting; no new comparison algorithm.
    const auto &a = self->_engine.attitude_engine().statistics();
    const auto &s = self->_engine.statistics();
    const auto &r = self->_engine.rate_engine().statistics();
    const auto &last = self->_att_frame;
    text.printf("last_operation,loop=%u,mode=%u,kind=%u,time_us=%u\n",
                unsigned(last.loop), unsigned(last.mode), unsigned(last.kind), unsigned(last.time_us));
    const float context[] = {last.config.dt, last.command.x, last.command.y, last.command.z,
                             last.sensors.gyro.x, last.sensors.gyro.y, last.sensors.gyro.z
                            };
    text.printf("last_input32,dt,command_x,command_y,command_z,gyro_x,gyro_y,gyro_z\nlast_values32");
    for (float value : context) {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        text.printf(",%08x", unsigned(bits));
    }
    text.printf("\n");
    const auto own_now = self->_engine.attitude_engine().history();
    const float attitude_context[] = {
        last.sensors.attitude.q1, last.sensors.attitude.q2, last.sensors.attitude.q3, last.sensors.attitude.q4,
        last.after.target.q1, last.after.target.q2, last.after.target.q3, last.after.target.q4,
        own_now.target.q1, own_now.target.q2, own_now.target.q3, own_now.target.q4,
        last.after.body.x, last.after.body.y, last.after.body.z, own_now.body.x, own_now.body.y, own_now.body.z,
        last.after.feedforward, own_now.feedforward
    };
    text.printf("last_attitude_values32");
    for (float value : attitude_context) {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        text.printf(",%08x", unsigned(bits));
    }
    text.printf("\n");
    // Preserve all binary64 error bits; no small difference is rounded to a textual zero.
    for (uint8_t k = 0; k < 26; k++) {
        flight_metric(text, "att", 0, k, a.maximum[k], a.squares[k], a.valid);
    }
    for (uint8_t k = 0; k < 4; k++) {
        flight_metric(text, "target", 0, k, s.target_max[k], s.target_squares[k], s.rate_calls);
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
        for (uint8_t k = 0; k < UTB_RateShadow::METRICS; k++) {
            flight_metric(text, "pid", axis, k, r.maximum[axis][k], r.squares[axis][k], r.valid);
        }
    }
}
#endif
void AP_UTB_ChainShadow::dump(ExpandingString &text)
{
    const AP_UTB_ChainShadow *self = _singleton;
    if (self == nullptr || !self->_published.load(std::memory_order_acquire)) {
        text.printf("UTBChainV1,pending\n"); return;
    }
    if (hal.util->get_soft_armed()) {
        text.printf("UTBChainV1,armed-unavailable\n"); return;
    }
    const auto &s = self->_engine.statistics();
    const auto &a = self->_engine.attitude_engine().statistics();
    const auto &r = self->_engine.rate_engine().statistics();
    text.printf("UTBChainV1,frozen,reason=%u\n", unsigned(self->_reason));
#define STAT(x) text.printf(#x ",%u\n", unsigned(s.x))
    STAT(attempts); STAT(paired); STAT(warmup); STAT(missing); STAT(duplicate); STAT(invalid); STAT(unsupported);
    STAT(upstream_mismatch); STAT(downstream_mismatch); STAT(motor_checks); STAT(motor_changes);
    STAT(limited); STAT(independent_i); STAT(source_loop); STAT(rate_loop); STAT(rate_max_us); STAT(rate_calls);
#undef STAT
    text.printf("rate_sum_us,%llu\n", (unsigned long long)s.rate_sum_us);
    text.printf("capture_timing,pre=%llu,post_without_chain_att=%llu,att_consume=%llu,att_consume_max=%u\n",
                (unsigned long long)self->_pre_us, (unsigned long long)self->_post_us,
                (unsigned long long)self->_att_us, unsigned(self->_att_max_us));
#define ENGINE(prefix, obj, x) text.printf(prefix #x ",%u\n", unsigned(obj.x))
    ENGINE("att_", a, seen); ENGINE("att_", a, valid); ENGINE("att_", a, synchronizations);
    ENGINE("att_", a, resets); ENGINE("att_", a, mode_changes); ENGINE("att_", a, config_changes);
    ENGINE("att_", a, mismatch); ENGINE("att_", a, desync); ENGINE("att_", a, invalid); ENGINE("att_", a, unsupported);
    ENGINE("att_", a, dynamic); ENGINE("att_", a, shaping);
    ENGINE("pid_", r, seen); ENGINE("pid_", r, valid); ENGINE("pid_", r, synchronizations);
    ENGINE("pid_", r, i_events); ENGINE("pid_", r, filter_resets); ENGINE("pid_", r, config_changes);
    ENGINE("pid_", r, mismatch); ENGINE("pid_", r, desync); ENGINE("pid_", r, invalid); ENGINE("pid_", r, unsupported);
    ENGINE("pid_", r, first_seq); ENGINE("pid_", r, last_seq);
    ENGINE("pid_", r, first_us); ENGINE("pid_", r, last_us);
    ENGINE("pid_", r, period_count); ENGINE("pid_", r, period_min_us); ENGINE("pid_", r, period_max_us);
#undef ENGINE
    text.printf("pid_period_sum_us,%llu\npid_jitter_sum_us,%llu\n",
                (unsigned long long)r.period_sum_us, (unsigned long long)r.jitter_sum_us);
    for (uint8_t k = 0; k < 7; k++) {
        text.printf("att_command_%u_seen,%u\natt_command_%u_valid,%u\n", unsigned(k), unsigned(a.command_seen[k]),
                    unsigned(k), unsigned(a.command_valid[k]));
    }
    text.printf("peaks,%.9f,%.9f,%.9f,%.9f,%.9f\n", double(s.command_peak), double(s.gyro_peak), double(s.i_peak), double(s.ff_peak), double(s.dff_peak));
    text.printf("timing,att_math=%llu,att_sync=%llu,att_compare=%llu,pid_math=%llu,pid_consume=%llu,native_capture=%llu\n",
                (unsigned long long)a.math_sum_us, (unsigned long long)a.sync_sum_us, (unsigned long long)a.compare_sum_us,
                (unsigned long long)r.math_sum_us, (unsigned long long)r.consume_sum_us, (unsigned long long)r.capture_sum_us);
    const auto &last = self->_engine.last_rate();
    const auto &att = self->_engine.last_attitude();
    const auto &target = self->_engine.consumed_target();
    const auto &own_attitude = self->_engine.consumed_attitude();
    text.printf("last_attitude,command=%.9f:%.9f:%.9f,ahrs=%.9f:%.9f:%.9f:%.9f,target=%.9f:%.9f:%.9f:%.9f,error=%.9f:%.9f:%.9f:%.9f\n",
                double(att.command.x), double(att.command.y), double(att.command.z),
                double(att.sensors.attitude.q1), double(att.sensors.attitude.q2), double(att.sensors.attitude.q3), double(att.sensors.attitude.q4),
                double(own_attitude.target.q1), double(own_attitude.target.q2), double(own_attitude.target.q3), double(own_attitude.target.q4),
                double(own_attitude.error.q1), double(own_attitude.error.q2), double(own_attitude.error.q3), double(own_attitude.error.q4));
    for (uint8_t axis = 0; axis < 3; axis++) {
        const auto own = self->_engine.rate_engine().history(axis);
        const auto &native = last.after[axis];
        text.printf("last_rate,%u,target_ap=%.9f,target_utb=%.9f,gyro=%.9f,dt=%.9f,limit=%u,i_event=%u,reset=%u\n",
                    unsigned(axis), double(last.input.target[axis]), double(target[axis]), double(last.input.gyro[axis]),
                    double(last.input.dt), unsigned(last.input.limited[axis]), unsigned(last.before[axis].i_set), unsigned(last.before[axis].reset_filter));
        text.printf("last_pid,%u,P=%.9f:%.9f,I=%.9f:%.9f,D=%.9f:%.9f,FF=%.9f:%.9f,DFF=%.9f:%.9f,output=%.9f:%.9f\n",
                    unsigned(axis), double(native.info.P), double(own.info.P), double(native.info.I), double(own.info.I),
                    double(native.info.D), double(own.info.D), double(native.info.FF), double(own.info.FF),
                    double(native.info.DFF), double(own.info.DFF), double(last.expected.control[axis]), double(self->_engine.rate_engine().output().control[axis]));
    }
    text.printf("section,axis,metric,max_abs,rmse\n");
    for (uint8_t k = 0; k < 26; k++) {
        text.printf("att,0,%u,%.9f,", unsigned(k), a.maximum[k]);
        if (a.valid) {
            text.printf("%.9f\n", sqrt(a.squares[k]/a.valid));
        } else {
            text.printf("nan\n"); // No samples: do not pass NaN into the HAL float formatter.
        }
    }
    for (uint8_t k = 0; k < 4; k++) {
        text.printf("target,0,%u,%.9f,", unsigned(k), s.target_max[k]);
        if (s.rate_calls) {
            text.printf("%.9f\n", sqrt(s.target_squares[k]/s.rate_calls));
        } else {
            text.printf("nan\n");
        }
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
        for (uint8_t k = 0; k < UTB_RateShadow::METRICS; k++) {
            text.printf("pid,%u,%u,%.9f,", unsigned(axis), unsigned(k), r.maximum[axis][k]);
            if (r.valid) {
                text.printf("%.9f\n", sqrt(r.squares[axis][k]/r.valid));
            } else {
                text.printf("nan\n");
            }
        }
    }
}
#endif // AP_UTB_CHAIN_SHADOW_AVAILABLE
