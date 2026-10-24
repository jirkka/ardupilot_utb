// GNU GPL v3 or later; original ArduPilot algorithms retain their copyright.
/// @file AP_UTB_AttitudeShadow.cpp
/// @brief Passive attitude capture, independent history and immutable post-window export.
/// @author Jiri Kovar
/// @version 2026-10-23
#include "AP_UTB_AttitudeShadow.h"
#include "AP_UTB_ChainShadow.h"
#if AP_UTB_ATTITUDE_SHADOW_AVAILABLE
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include <cstring>
extern const AP_HAL::HAL &hal;
using namespace UTBAttitudeCapture;

void UTB_AttitudeShadowMath::execute(Command kind, const Vector3f &command, bool reset_rate)
{
    switch (kind) {
    case Command::STABILIZE:
        input_euler_angle_roll_pitch_euler_rate_yaw_rad(command.x, command.y, command.z); break;
    case Command::ACRO:
        input_rate_bf_roll_pitch_yaw_rads(command.x, command.y, command.z); break;
    case Command::RATE_ONLY:
        input_rate_bf_roll_pitch_yaw_2_rads(command.x, command.y, command.z); break;
    case Command::RESET:
        reset_target_and_rate(reset_rate); break;
    case Command::RESET_YAW:
        reset_yaw_target_and_rate(reset_rate); break;
    case Command::RELAX:
        relax_attitude_controllers(); break;
    case Command::INERTIAL:
        inertial_frame_reset(); break;
    }
}
void UTB_AttitudeShadowEngine::values(const History &h, float (&v)[METRICS])
{
    const float all[] = {h.target.q1,h.target.q2,h.target.q3,h.target.q4,
                         h.error.q1,h.error.q2,h.error.q3,h.error.q4,
                         h.euler.x,h.euler.y,h.euler.z,h.euler_rate.x,h.euler_rate.y,h.euler_rate.z,
                         h.velocity.x,h.velocity.y,h.velocity.z,h.acceleration.x,h.acceleration.y,h.acceleration.z,
                         h.body.x,h.body.y,h.body.z,h.thrust,h.thrust_error,h.feedforward
                        };
    static_assert(ARRAY_SIZE(all) == METRICS, "Attitude metric projection");
    memcpy(v, all, sizeof(v));
}
// Only quaternion component groups may change sign. Body rates are compared directly.
static bool comparable(const History &a, const History &b, float (&x)[UTB_AttitudeShadowEngine::METRICS],
                       float (&y)[UTB_AttitudeShadowEngine::METRICS], uint32_t &signs)
{
    UTB_AttitudeShadowEngine::values(a, x); UTB_AttitudeShadowEngine::values(b, y);
    for (uint8_t group = 0; group < 2; group++) {
        const uint8_t i = group * 4;
        if (x[i]*y[i] + x[i+1]*y[i+1] + x[i+2]*y[i+2] + x[i+3]*y[i+3] < 0) {
            for (uint8_t k = i; k < i+4; k++) {
                x[k] = -x[k];
            }
            signs++;
        }
    }
    for (uint8_t k = 0; k < UTB_AttitudeShadowEngine::METRICS; k++) {
        if (!isfinite(x[k]) || !isfinite(y[k])) {
            return false;
        }
    }
    return a.target.is_unit_length() && a.error.is_unit_length() &&
           b.target.is_unit_length() && b.error.is_unit_length();
}
bool UTB_AttitudeShadowEngine::same_history(const History &a, const History &b)
{
    float x[METRICS], y[METRICS]; // Scalar projections, never raw object padding.
    uint32_t signs = 0;
    if (!comparable(a, b, x, y, signs)) {
        return false;
    }
    for (uint8_t k = 0; k < METRICS; k++) {
        if (fabsf(x[k]-y[k]) > 1.0e-6f * MAX(1.0f, fabsf(y[k]))) {
            return false;
        }
    }
    return true;
}
static bool same_float(float a, float b)
{
    return memcmp(&a, &b, sizeof(float)) == 0;
}
static bool same_vector(const Vector3f &a, const Vector3f &b)
{
    return same_float(a.x, b.x) && same_float(a.y, b.y) && same_float(a.z, b.z);
}
static bool same_config(const Config &a, const Config &b)
{
    return same_vector(a.angle_p, b.angle_p) && same_vector(a.accel, b.accel) && same_vector(a.rate_max, b.rate_max) &&
           same_float(a.input_tc, b.input_tc) && same_float(a.rp_tc, b.rp_tc) && same_float(a.yaw_tc, b.yaw_tc) &&
           same_float(a.yaw_p, b.yaw_p) && same_float(a.trim, b.trim) && a.feedforward == b.feedforward &&
           a.sqrt_controller == b.sqrt_controller;
    // dt and angle_scale are per-call native inputs, not persistent configuration.
}
static bool valid_config(const Config &c)
{
    const float v[] = {c.angle_p.x,c.angle_p.y,c.angle_p.z,c.accel.x,c.accel.y,c.accel.z,
                       c.rate_max.x,c.rate_max.y,c.rate_max.z,c.angle_scale.x,c.angle_scale.y,c.angle_scale.z,
                       c.dt,c.input_tc,c.rp_tc,c.yaw_tc,c.yaw_p,c.trim
                      };
    for (float x : v) {
        if (!isfinite(x)) {
            return false;
        }
    }
    return c.dt > 0 && c.angle_p.x >= 0 && c.angle_p.y >= 0 && c.angle_p.z >= 0 &&
           c.accel.x >= 0 && c.accel.y >= 0 && c.accel.z >= 0 &&
           c.angle_scale.x >= 0 && c.angle_scale.y >= 0 && c.angle_scale.z >= 0 &&
           c.rate_max.x >= 0 && c.rate_max.y >= 0 && c.rate_max.z >= 0 && c.yaw_p >= 0;
}
void UTB_AttitudeShadowEngine::start(bool continuous)
{
    _stats = {}; _continuous = continuous; _synced = false; _seq = 0;
}
void UTB_AttitudeShadowEngine::process(const Frame &f)
{
    float actual[METRICS], expected[METRICS]; // Compared states after independent computation.
    uint32_t signs = 0, start, elapsed;
    bool different = false, reset = uint8_t(f.kind) >= uint8_t(Command::RESET);
    History own;
    const uint32_t sync_start = AP_HAL::micros();
    _stats.seen++;
    if (uint8_t(f.kind) > uint8_t(Command::INERTIAL)) {
        _stats.unsupported++;
        _synced = false;
        return;
    }
    _stats.command_seen[uint8_t(f.kind)]++;
    if (_stats.seen == 1) {
        _stats.first_seq = f.seq;
        _stats.first_loop = f.loop;
    }
    _stats.last_seq = f.seq; _stats.last_loop = f.loop;
    if ((_stats.seen > 1 && f.seq != _seq + 1U) || f.seq == 0) {
        _stats.desync++; _synced = false; _seq = f.seq; return;
    }
    _seq = f.seq;
    if (!valid_config(f.config) || f.command.is_nan() || f.command.is_inf() ||
        f.sensors.gyro.is_nan() || f.sensors.gyro.is_inf() || !isfinite(f.sensors.yaw) ||
        !f.sensors.attitude.is_unit_length() ||
        !comparable(f.before, f.after, actual, expected, signs)) {
        _stats.invalid++; _synced = false; return;
    }
    if (!is_zero(f.config.trim)) {
        _stats.unsupported++;
        _synced = false;
        return;
    }
    if (_stats.seen > 1 && f.mode != _mode) {
        _stats.mode_changes++; _synced = false;
    }
    _mode = f.mode;
    if (_synced && !same_config(f.config, _config)) {
        _stats.config_changes++; _stats.desync++; _synced = false; _config = f.config; return;
    }
    _config = f.config;
    _math.shadow_config(f.config); _math.sample(f.sensors);
    if (!_continuous || !_synced) {
        _math.shadow_history(f.before); _stats.synchronizations++; _synced = true;
    } else if (!same_history(_math.shadow_history(), f.before)) {
        _stats.desync++; _synced = false; return;
    }
    start = AP_HAL::micros();
    elapsed = start - sync_start;
    _stats.sync_sum_us += elapsed; _stats.sync_max_us = MAX(_stats.sync_max_us, elapsed);
    _math.execute(f.kind, f.command, f.reset_rate);
    elapsed = AP_HAL::micros() - start;
    _stats.math_sum_us += elapsed; _stats.math_max_us = MAX(_stats.math_max_us, elapsed);
    own = _math.shadow_history();
    signs = 0;
    start = AP_HAL::micros();
    if (!comparable(own, f.after, actual, expected, signs)) {
        _stats.invalid++; _synced = false; return;
    }
    _stats.quaternion_sign_equivalent += signs;
    for (uint8_t k = 0; k < METRICS; k++) {
        const double error = fabs(double(actual[k]) - expected[k]);
        _stats.maximum[k] = MAX(_stats.maximum[k], error);
        _stats.squares[k] += error*error;
        different |= error > 1.0e-6 * MAX(1.0, fabs(double(expected[k])));
    }
    elapsed = AP_HAL::micros() - start;
    _stats.compare_sum_us += elapsed; _stats.compare_max_us = MAX(_stats.compare_max_us, elapsed);
    _stats.valid++;
    _stats.command_valid[uint8_t(f.kind)]++;
    if (reset) {
        _stats.resets++;
        _stats.reset_compared++;
    }
    if (own.body.length_squared() > 1.0e-6f && own.thrust_error > 1.0e-4f) {
        _stats.dynamic++;
    }
    if (own.acceleration.length_squared() > 1.0e-6f) {
        _stats.shaping++;
    }
    if (different) {
        _stats.mismatch++;
        _synced = false;
    }
}

AP_UTB_AttitudeShadow *AP_UTB_AttitudeShadow::_singleton;
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Immutable publication must be lock-free");
const AP_Param::GroupInfo AP_UTB_AttitudeShadow::var_info[] = {
    // @Param: RUN
    // @DisplayName: Native attitude RAM shadow session
    // @Description: One finite session per boot, independent from native rate shadow. One-step imports prehistory per operation; continuous imports at epochs and replays real external reset commands. No control authority. Cleared at boot.
    // @Values: 0:Off,1:One step,2:Continuous
    // @User: Advanced
    AP_GROUPINFO("RUN", 0, AP_UTB_AttitudeShadow, _run, 0),
    // @Param: COUNT
    // @DisplayName: Native attitude shadow main loop attempts
    // @Description: Finite window including unsupported or missing native calls. Reset operations are counted separately from main-loop attempts.
    // @Range: 1 200000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("COUNT", 1, AP_UTB_AttitudeShadow, _count, 4000),
    AP_GROUPEND
};
AP_UTB_AttitudeShadow::AP_UTB_AttitudeShadow()
{
    AP_Param::setup_object_defaults(this, var_info); _singleton = this;
}
void AP_UTB_AttitudeShadow::init()
{
    _run.set(0);
    _initialized = true;
}
void AP_UTB_AttitudeShadow::freeze(uint8_t reason)
{
    _reason = reason; _running = false; _run.set(0); _pending = false;
    _published.store(true, std::memory_order_release);
}
void AP_UTB_AttitudeShadow::prepare(bool supported, uint8_t mode)
{
    if (!_initialized || _published.load(std::memory_order_relaxed)) {
        return;
    }
    if (_pending) {
        freeze(7);
        return;
    }
    if (!_running && _run.get() != 0) {
        if (_run.get() < 1 || _run.get() > 2 || _count.get() < 1 || _count.get() > 200000) {
            freeze(4);
            return;
        }
        _limit = uint32_t(_count.get()); _engine.start(_run.get() == 2); _running = true;
    }
    if (!_running) {
        return;
    }
    if (_run.get() == 0) {
        freeze(2);
        return;
    }
    _mode = mode; _supported = supported && (mode == 0 || mode == 1);
    _calls = 0; _pending = true; _loop++;
}
bool AP_UTB_AttitudeShadow::enabled()
{
    // Short-circuit before reading main-owned fields if a different thread invokes a native method.
    return hal.scheduler->in_main_thread() && _singleton != nullptr && _singleton->_running &&
           _singleton->_supported && _singleton->_pending;
}
void AP_UTB_AttitudeShadow::before(Command kind, const Vector3f &command, bool reset_rate,
                                   const Config &config, const Sensors &sensors, const History &history)
{
    AP_UTB_AttitudeShadow &s = *_singleton;
    s._frame.config = config; s._frame.sensors = sensors; s._frame.before = history;
    s._frame.kind = kind; s._frame.command = command; s._frame.reset_rate = reset_rate;
    s._frame.seq = ++s._seq; s._frame.loop = s._loop; s._frame.mode = s._mode;
    s._frame.time_us = AP_HAL::micros(); s._native_start = s._frame.time_us;
    if (uint8_t(kind) < uint8_t(Command::RESET)) {
        s._calls++;
        if (s._period_last != 0) {
            const uint32_t period = s._frame.time_us - s._period_last;
            s._period_count++;
            s._period_sum += period; s._period_min = MIN(s._period_min, period); s._period_max = MAX(s._period_max, period);
            s._jitter_sum += period >= 2500U ? period - 2500U : 2500U - period;
        }
        s._period_last = s._frame.time_us;
    }
}
void AP_UTB_AttitudeShadow::after(const History &history)
{
    AP_UTB_AttitudeShadow &s = *_singleton;
    const uint32_t native_us = AP_HAL::micros() - s._native_start;
    s._native_sum += native_us; s._native_max = MAX(s._native_max, native_us);
    s._frame.after = history;
    const uint32_t consume_start = AP_HAL::micros();
    s._engine.process(s._frame);
    const uint32_t consume_us = AP_HAL::micros() - consume_start;
    s._consume_last = consume_us;
    s._consume_sum += consume_us; s._consume_max = MAX(s._consume_max, consume_us);
}
void AP_UTB_AttitudeShadow::audit(bool changed)
{
    _singleton->_motor_checks++;
    if (changed) {
        _singleton->_motor_changes++;
    }
}
void AP_UTB_AttitudeShadow::capture_time(bool post, uint32_t elapsed_us)
{
    AP_UTB_AttitudeShadow &s = *_singleton;
    if (post) {
        // Post bracket includes consume; remove that measured nested span, not native math.
        const uint32_t capture_us = elapsed_us >= s._consume_last ? elapsed_us - s._consume_last : 0;
        s._post_sum += capture_us; s._post_max = MAX(s._post_max, capture_us);
    } else {
        s._pre_sum += elapsed_us; s._pre_max = MAX(s._pre_max, elapsed_us);
    }
}
void AP_UTB_AttitudeShadow::finish()
{
    if (!_running || !_pending) {
        return;
    }
    _pending = false; _attempts++;
    if (!_supported) {
        _unsupported++;
        _engine.invalidate_epoch();
    } else if (_calls == 0) {
        _missing++;
    } else if (_calls > 1) {
        _duplicate++;
    }
    const auto &s = _engine.statistics();
    if (s.invalid || s.desync || s.mismatch || _missing || _duplicate || _motor_changes) {
        freeze(3);
    } else if (_consume_max >= 2500U) {
        freeze(5);
    } else if (_attempts >= _limit) {
        freeze(1);
    }
}
void AP_UTB_AttitudeShadow::dump(ExpandingString &text)
{
    const AP_UTB_AttitudeShadow *self = _singleton;
    if (self == nullptr || !self->_published.load(std::memory_order_acquire)) {
        text.printf("UTBAttitudeShadowV1,pending\n"); return;
    }
    if (hal.util->get_soft_armed()) {
        text.printf("UTBAttitudeShadowV1,armed-unavailable\n");
        return;
    }
    const auto &s = self->_engine.statistics();
    text.printf("UTBAttitudeShadowV1,frozen,reason=%u\n", unsigned(self->_reason));
#define STAT(name) text.printf(#name ",%u\n", unsigned(s.name))
    STAT(seen); STAT(valid); STAT(invalid); STAT(desync); STAT(unsupported); STAT(mismatch);
    STAT(synchronizations); STAT(resets); STAT(reset_compared); STAT(mode_changes); STAT(config_changes);
    STAT(quaternion_sign_equivalent); STAT(dynamic); STAT(shaping);
    STAT(first_seq); STAT(last_seq); STAT(first_loop); STAT(last_loop);
    STAT(sync_max_us);
    STAT(math_max_us); STAT(compare_max_us);
#undef STAT
    text.printf("attempts,%u\nmissing,%u\nduplicate,%u\nunsupported_loops,%u\n", unsigned(self->_attempts),
                unsigned(self->_missing), unsigned(self->_duplicate), unsigned(self->_unsupported));
    text.printf("math_sum_us,%llu\ncompare_sum_us,%llu\nnative_capture_sum_us,%llu\nnative_capture_max_us,%u\n",
                (unsigned long long)s.math_sum_us, (unsigned long long)s.compare_sum_us,
                (unsigned long long)self->_native_sum, unsigned(self->_native_max));
    text.printf("period_min_us,%u\nperiod_max_us,%u\nperiod_sum_us,%llu\njitter_sum_us,%llu\n",
                unsigned(self->_period_min), unsigned(self->_period_max), (unsigned long long)self->_period_sum,
                (unsigned long long)self->_jitter_sum);
    text.printf("sync_sum_us,%llu\npre_capture_sum_us,%llu\npost_capture_audit_sum_us,%llu\npre_capture_max_us,%u\npost_capture_audit_max_us,%u\nperiod_count,%u\n",
                (unsigned long long)s.sync_sum_us, (unsigned long long)self->_pre_sum, (unsigned long long)self->_post_sum,
                unsigned(self->_pre_max), unsigned(self->_post_max), unsigned(self->_period_count));
    text.printf("consume_sum_us,%llu\nconsume_max_us,%u\n",
                (unsigned long long)self->_consume_sum, unsigned(self->_consume_max));
    text.printf("motor_checks,%u\nmotor_changes,%u\n", unsigned(self->_motor_checks), unsigned(self->_motor_changes));
    for (uint8_t k = 0; k < 7; k++) {
        text.printf("command_%u,%u,%u\n", unsigned(k), unsigned(s.command_seen[k]), unsigned(s.command_valid[k]));
    }
    text.printf("metric,max_abs,rmse\n");
    for (uint8_t k = 0; k < UTB_AttitudeShadowEngine::METRICS; k++) {
        text.printf("%u,%.9g,%.9g\n", unsigned(k), s.valid ? s.maximum[k] : double(NAN),
                    s.valid ? sqrt(s.squares[k]/s.valid) : double(NAN));
    }
}
namespace UTBAttitudeCapture
{
bool enabled()
{
    return AP_UTB_AttitudeShadow::enabled()
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
           || AP_UTB_ChainShadow::enabled()
#endif
           ;
}
void before(Command k, const Vector3f &c, bool reset, const Config &config, const Sensors &sensors, const History &history)
{
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_AttitudeShadow::enabled()) {
#endif
        AP_UTB_AttitudeShadow::before(k, c, reset, config, sensors, history);
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    }
#endif
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_ChainShadow::enabled()) {
        AP_UTB_ChainShadow::get_singleton()->before_attitude(k, c, reset, config, sensors, history);
    }
#endif
}
void after(const History &h)
{
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_AttitudeShadow::enabled()) {
#endif
        AP_UTB_AttitudeShadow::after(h);
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    }
#endif
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_ChainShadow::enabled()) {
        AP_UTB_ChainShadow::get_singleton()->after_attitude(h);
    }
#endif
}
void audit(bool changed)
{
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_AttitudeShadow::enabled()) {
#endif
        AP_UTB_AttitudeShadow::audit(changed);
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    }
#endif
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_ChainShadow::enabled()) {
        AP_UTB_ChainShadow::get_singleton()->audit(changed);
    }
#endif
}
void capture_time(bool post, uint32_t elapsed_us)
{
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_AttitudeShadow::enabled()) {
#endif
        AP_UTB_AttitudeShadow::capture_time(post, elapsed_us);
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    }
#endif
#if AP_UTB_CHAIN_SHADOW_AVAILABLE
    if (AP_UTB_ChainShadow::enabled()) {
        AP_UTB_ChainShadow::get_singleton()->capture_time(post, elapsed_us);
    }
#endif
}
}
#endif
