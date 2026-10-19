// GNU GPL v3 or later; math is the unchanged AP/UTB3A implementation.
/// @file AP_UTB_NativeBench.cpp
/// @brief DISARMED comparison, independent states and strictly RAM-only outputs.
/// @author Jiri Kovar
/// @version 2026-10-19
#include "AP_UTB_NativeBench.h"
#if AP_UTB_NATIVE_BENCH_AVAILABLE
#include "UTB_Rate.h"
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <new>
#include <cstring>

const AP_Param::GroupInfo AP_UTB_NativeBench::var_info[] = {
    // @Param: RUN
    // @DisplayName: Disarmed native AP and UTB comparison
    // @Description: Starts one finite 2000-step numerical experiment while disarmed and logging. Uses separate AP and UTB controller instances, RAM-only sinks, fixed test gains and no motor output. Stops if armed or logging is unavailable. Automatically returns to zero; not a flight controller.
    // @Values: 0:Stopped,1:Run once
    // @User: Advanced
    AP_GROUPINFO("RUN", 0, AP_UTB_NativeBench, _run, 0),
    AP_GROUPEND
};

// No mixer, motor init, RC output, HAL output, battery update or spool logic is run.
// All hardware-facing virtual entry points are inert. Controller setters only store floats.
class UTB_BenchMotorSink final : public AP_MotorsMulticopter
{
public:
    UTB_BenchMotorSink() : AP_MotorsMulticopter(AP_MOTORS_SPEED_DEFAULT, false) {}
    void init(motor_frame_class, motor_frame_type) override {}
    void set_frame_class_and_type(motor_frame_class, motor_frame_type) override {}
    void output() override {}
    void output_min() override {}
    uint32_t get_motor_mask() override
    {
        return 0;
    }
    void output_armed_stabilizing() override {}
    void output_to_motors() override {}
    void output_motor_mask(float, uint32_t, float) override {}
protected:
    const char *_get_frame_string() const override
    {
        return "RAM diagnostic sink";
    }
    void _output_test_seq(uint8_t, int16_t) override {}
};

struct AP_UTB_NativeBench::Session {
    // Storage is zero-initialised before construction, as for the original vehicle objects.
    UTB_BenchMotorSink ap_sink;
    UTB_BenchMotorSink utb_sink;
    AP_AHRS_View &view;
    UTB_State state;
    AC_AttitudeControl_Multi ap;
    UTB_Rate utb;
    uint64_t time_us = 0;
    uint32_t ap_us = 0, utb_us = 0, wall_us = 0;
    uint8_t phase = 0, flags = 0;
    float dt = 0;
    Vector3f reference, measured, ap_rates, utb_rates;
    Quaternion body, ap_target, utb_target;
    AP_PIDInfo ap_info[3], utb_info[3];
    float ap_out[3] {}, utb_out[3] {}, ap_ff[3] {}, utb_ff[3] {};
    float ap_motor_ff[3] {}, utb_motor_ff[3] {};
    Session(AP_AHRS_View &ahrs) : view(ahrs), state(ahrs), ap(ahrs, ap_sink, false), utb(state, utb_sink) {}
};

// Local functions: exact bit comparison detects asynchronous input changes (including tiny changes).
static bool same_bits(float a, float b)
{
    return memcmp(&a, &b, sizeof(float)) == 0;
}

// Identical diagnostic fixtures, not a flight tune or EEPROM write.
template <typename PID> static void configure(PID &pid)
{
    pid.kP().set(.135f);
    pid.kI().set(.135f);
    pid.kD().set(.0036f);
    pid.ff().set(.07f);
    pid.kDff().set(.002f);
    pid.set_imax(.5f);
    pid.set_pdmax(.2f);
    pid.set_filt_T_hz(15);
    pid.set_filt_E_hz(7);
    pid.set_filt_D_hz(20);
    // Millisecond-clock slew scheduling is excluded from this deterministic paired fixture.
    pid.set_slew_limit(0);
    pid.reset_I();
    pid.reset_filter();
}

AP_UTB_NativeBench::AP_UTB_NativeBench()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_UTB_NativeBench::init(AP_AHRS_View &view)
{
    // One boot-time construction; no heap or controller construction in update().
    alignas(Session) static uint8_t storage[sizeof(Session)] {};
    AP_Motors *const live_motors = AP_Motors::get_singleton();
    AC_AttitudeControl *const live_controller = AC_AttitudeControl::get_singleton();
    if (_session != nullptr) {
        return;
    }
    _session = new (storage) Session(view);
    if (AP_Motors::get_singleton() != live_motors || AC_AttitudeControl::get_singleton() != live_controller) {
        _session = nullptr;
        return;
    }
    _run.set(0); // A saved RUN=1 never auto-starts an experiment after boot.
}

bool AP_UTB_NativeBench::compute()
{
    Session &s = *_session;
    const uint32_t now = AP_HAL::micros();
    const uint16_t sample = _seq % 500;
    uint32_t start;
    Vector3f after;
    Quaternion attitude_after;
    s.time_us = AP_HAL::micros64();
    s.wall_us = now - _last_us;
    _last_us = now;
    s.phase = _seq / 500;
    s.dt = s.phase == 3 && sample != 0 ? s.wall_us * 1.0e-6f : .0025f;
    s.flags = (sample >= 250 && sample < 350) ? 1 : 0;
    if (!s.state.finite()) {
        s.flags |= 16; // Report health independently from numerical equality.
    }
    s.reference = sample < 100 ? Vector3f() : sample < 250 ? Vector3f(2, -1, .5f) : Vector3f(-2, 1, -.5f);
    s.measured = s.phase == 3 ? s.view.get_gyro_latest() : Vector3f(.1f*sinf(sample*.03f), .2f*cosf(sample*.02f), 0);
    s.view.get_quat_body_to_ned(s.body);
    if (!isfinite(s.dt) || !is_positive(s.dt) || s.measured.is_nan() || s.measured.is_inf() ||
        !isfinite(s.body.length_squared()) || !is_positive(s.body.length_squared())) {
        return false;
    }
    if (sample == 0) {
        configure(s.ap.get_rate_roll_pid()); configure(s.utb.get_rate_roll_pid());
        configure(s.ap.get_rate_pitch_pid()); configure(s.utb.get_rate_pitch_pid());
        configure(s.ap.get_rate_yaw_pid()); configure(s.utb.get_rate_yaw_pid());
        s.ap.reset_target_and_rate(true); s.utb.reset_target_and_rate(true);
    }
    if (sample == 200) {
        s.ap.reset_rate_controller_I_terms(); s.utb.reset_rate_controller_I_terms();
        s.flags |= 2;
    }
    if (sample == 400) {
        s.ap.get_rate_roll_pid().reset_filter(); s.utb.get_rate_roll_pid().reset_filter();
        s.ap.get_rate_pitch_pid().reset_filter(); s.utb.get_rate_pitch_pid().reset_filter();
        s.ap.get_rate_yaw_pid().reset_filter(); s.utb.get_rate_yaw_pid().reset_filter();
        s.flags |= 4;
    }
    s.ap_sink.limit.roll = s.ap_sink.limit.pitch = s.ap_sink.limit.yaw = (s.flags & 1) != 0;
    s.utb_sink.limit = s.ap_sink.limit;
    s.ap.set_dt_s(s.dt); s.utb.set_dt_s(s.dt);
    start = AP_HAL::micros();
    if (s.phase == 0) {
        s.ap_out[0] = s.ap.get_rate_roll_pid().update_all(s.reference.x, s.measured.x, s.dt, s.flags & 1);
        s.ap_out[1] = s.ap.get_rate_pitch_pid().update_all(s.reference.y, s.measured.y, s.dt, s.flags & 1);
        s.ap_out[2] = s.ap.get_rate_yaw_pid().update_all(s.reference.z, s.measured.z, s.dt, s.flags & 1);
        s.ap_rates = s.reference;
    } else {
        if (s.phase == 1) {
            s.ap.input_rate_bf_roll_pitch_yaw_2_rads(s.reference.x, s.reference.y, s.reference.z);
        } else {
            s.ap.input_euler_angle_roll_pitch_euler_rate_yaw_rad(.1f*s.reference.x, .1f*s.reference.y, s.reference.z);
        }
        s.ap_rates = s.ap.rate_bf_targets();
        s.ap.rate_controller_run_dt(s.measured, s.dt);
        s.ap.rate_controller_target_reset();
        s.ap_out[0] = s.ap_sink.get_roll(); s.ap_out[1] = s.ap_sink.get_pitch(); s.ap_out[2] = s.ap_sink.get_yaw();
    }
    s.ap_us = AP_HAL::micros() - start;
    start = AP_HAL::micros();
    if (s.phase == 0) {
        s.utb_out[0] = s.utb.get_rate_roll_pid().update_all(s.reference.x, s.measured.x, s.dt, s.flags & 1);
        s.utb_out[1] = s.utb.get_rate_pitch_pid().update_all(s.reference.y, s.measured.y, s.dt, s.flags & 1);
        s.utb_out[2] = s.utb.get_rate_yaw_pid().update_all(s.reference.z, s.measured.z, s.dt, s.flags & 1);
        s.utb_rates = s.reference;
    } else {
        if (s.phase == 1) {
            s.utb.input_rate_bf_roll_pitch_yaw_2_rads(s.reference.x, s.reference.y, s.reference.z);
        } else {
            s.utb.input_euler_angle_roll_pitch_euler_rate_yaw_rad(.1f*s.reference.x, .1f*s.reference.y, s.reference.z);
        }
        s.utb_rates = s.utb.rate_bf_targets();
        s.utb.rate_controller_run_dt(s.measured, s.dt);
        s.utb.rate_controller_target_reset();
        s.utb_out[0] = s.utb_sink.get_roll(); s.utb_out[1] = s.utb_sink.get_pitch(); s.utb_out[2] = s.utb_sink.get_yaw();
    }
    s.utb_us = AP_HAL::micros() - start;
    s.ap_info[0] = s.ap.get_rate_roll_pid().get_pid_info(); s.utb_info[0] = s.utb.get_rate_roll_pid().get_pid_info();
    s.ap_info[1] = s.ap.get_rate_pitch_pid().get_pid_info(); s.utb_info[1] = s.utb.get_rate_pitch_pid().get_pid_info();
    s.ap_info[2] = s.ap.get_rate_yaw_pid().get_pid_info(); s.utb_info[2] = s.utb.get_rate_yaw_pid().get_pid_info();
    s.ap_ff[0] = s.ap.get_rate_roll_pid().get_ff(); s.utb_ff[0] = s.utb.get_rate_roll_pid().get_ff();
    s.ap_ff[1] = s.ap.get_rate_pitch_pid().get_ff(); s.utb_ff[1] = s.utb.get_rate_pitch_pid().get_ff();
    s.ap_ff[2] = s.ap.get_rate_yaw_pid().get_ff(); s.utb_ff[2] = s.utb.get_rate_yaw_pid().get_ff();
    s.ap_motor_ff[0] = s.phase == 0 ? s.ap_ff[0] : s.ap_sink.get_roll_ff();
    s.ap_motor_ff[1] = s.phase == 0 ? s.ap_ff[1] : s.ap_sink.get_pitch_ff();
    s.ap_motor_ff[2] = s.phase == 0 ? s.ap_ff[2] : s.ap_sink.get_yaw_ff();
    s.utb_motor_ff[0] = s.phase == 0 ? s.utb_ff[0] : s.utb_sink.get_roll_ff();
    s.utb_motor_ff[1] = s.phase == 0 ? s.utb_ff[1] : s.utb_sink.get_pitch_ff();
    s.utb_motor_ff[2] = s.phase == 0 ? s.utb_ff[2] : s.utb_sink.get_yaw_ff();
    s.ap_target = s.ap.get_attitude_target_quat(); s.utb_target = s.utb.get_attitude_target_quat();
    after = s.view.get_gyro_latest();
    s.view.get_quat_body_to_ned(attitude_after);
    if ((s.phase == 3 && (!same_bits(after.x, s.measured.x) || !same_bits(after.y, s.measured.y) || !same_bits(after.z, s.measured.z))) ||
        !same_bits(attitude_after.q1, s.body.q1) || !same_bits(attitude_after.q2, s.body.q2) ||
        !same_bits(attitude_after.q3, s.body.q3) || !same_bits(attitude_after.q4, s.body.q4)) {
        s.flags |= 8; // Non-identical asynchronous input: exclude, do not label as a numerical mismatch.
    }
    return true;
}

float AP_UTB_NativeBench::max_difference() const
{
    const Session &s = *_session;
    float difference = 0;
    for (uint8_t axis = 0; axis < 3; axis++) {
        const float a[] = {s.ap_rates[axis], s.ap_info[axis].target, s.ap_info[axis].P, s.ap_info[axis].I,
                           s.ap_info[axis].D, s.ap_ff[axis], s.ap_out[axis]
                          };
        const float u[] = {s.utb_rates[axis], s.utb_info[axis].target, s.utb_info[axis].P, s.utb_info[axis].I,
                           s.utb_info[axis].D, s.utb_ff[axis], s.utb_out[axis]
                          };
        for (uint8_t field = 0; field < ARRAY_SIZE(a); field++) {
            if (!isfinite(a[field]) || !isfinite(u[field])) {
                return INFINITY;
            }
            difference = MAX(difference, fabsf(a[field] - u[field]));
        }
    }
    for (uint8_t field = 0; field < 4; field++) {
        if (!isfinite(s.ap_target[field]) || !isfinite(s.utb_target[field])) {
            return INFINITY;
        }
        difference = MAX(difference, fabsf(s.ap_target[field] - s.utb_target[field]));
    }
    return difference;
}

void AP_UTB_NativeBench::emit()
{
#if HAL_LOGGING_ENABLED
    const Session &s = *_session;
    AP_Logger &logger = AP::logger();
    const uint32_t start = AP_HAL::micros();
    for (uint8_t axis = 0; axis < 3; axis++) {
        const AP_PIDInfo &a = s.ap_info[axis];
        const AP_PIDInfo &u = s.utb_info[axis];
        logger.WriteStreaming("U3BD", "TimeUS,N,Ph,Ax,Dt,Ref,Gyr,AT,UT,AP,UP,AI,UI,AD,UD", "QIBBfffffffffff",
                              s.time_us, _seq, s.phase, axis, double(s.dt), double(s.reference[axis]), double(s.measured[axis]),
                              double(a.target), double(u.target), double(a.P), double(u.P), double(a.I), double(u.I),
                              double(a.D), double(u.D));
        logger.WriteStreaming("U3BF", "TimeUS,N,Ax,AF,UF,ADF,UDF,AS,US,AO,UO,AMF,UMF,ATot,UTot", "QIBffffffffffff",
                              s.time_us, _seq, axis, double(a.FF), double(u.FF), double(a.DFF), double(u.DFF),
                              double(s.ap_ff[axis]), double(s.utb_ff[axis]), double(s.ap_out[axis]), double(s.utb_out[axis]),
                              double(s.ap_motor_ff[axis]), double(s.utb_motor_ff[axis]),
                              double(s.ap_out[axis] + s.ap_motor_ff[axis]), double(s.utb_out[axis] + s.utb_motor_ff[axis]));
    }
    logger.WriteStreaming("U3BA", "TimeUS,N,AR,AP,AY,UR,UP,UY,QW,QX,QY,QZ", "QIffffffffff",
                          s.time_us, _seq, double(s.ap_rates.x), double(s.ap_rates.y), double(s.ap_rates.z),
                          double(s.utb_rates.x), double(s.utb_rates.y), double(s.utb_rates.z),
                          double(s.body.q1), double(s.body.q2), double(s.body.q3), double(s.body.q4));
    logger.WriteStreaming("U3BQ", "TimeUS,N,AW,AX,AY,AZ,UW,UX,UY,UZ", "QIffffffff",
                          s.time_us, _seq, double(s.ap_target.q1), double(s.ap_target.q2), double(s.ap_target.q3), double(s.ap_target.q4),
                          double(s.utb_target.q1), double(s.utb_target.q2), double(s.utb_target.q3), double(s.utb_target.q4));
    logger.WriteStreaming("U3BT", "TimeUS,N,APus,UTus,Wall,LogUs,Flags,Ver", "QIIIIIBB",
                          s.time_us, _seq, s.ap_us, s.utb_us, s.wall_us, _last_log_us, s.flags, 2);
    _last_log_us = AP_HAL::micros() - start; // Export latency belongs to the preceding sample, explicitly separate from math.
#endif
}

void AP_UTB_NativeBench::update(bool armed)
{
    if (armed || _session == nullptr || _run.get() != 1) {
        _run.set(0);
        _running = false;
        return;
    }
#if HAL_LOGGING_ENABLED
    AP_Logger &logger = AP::logger();
    // An open log alone does not imply that DISARMED writes are currently permitted.
    if (!logger.logging_started() || !logger.should_log(UINT32_MAX) ||
        !logger.WritesEnabled() || logger.logging_failed()) {
        _run.set(0);
        _running = false;
        return;
    }
#else
    _run.set(0);
    return;
#endif
    if (!_running) {
        _seq = 0;
        _last_us = AP_HAL::micros();
        _last_log_us = 0;
        _running = true;
    }
    if (!compute()) {
        _run.set(0);
        _running = false;
        return;
    }
    emit();
    if (++_seq == 2000) {
        _run.set(0);
        _running = false;
    }
}
#endif // AP_UTB_NATIVE_BENCH_AVAILABLE
