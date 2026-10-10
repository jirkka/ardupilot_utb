/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
 */
#include "AP_UTB.h"
#if AP_UTB_ENABLED
constexpr uint8_t AP_UTB::Queue::CAPACITY;
static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_CHAR_LOCK_FREE == 2 && ATOMIC_BOOL_LOCK_FREE == 2, "UTB feedback atomics must not lock");
const AP_Param::GroupInfo AP_UTB::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: UTB diagnostic enable
    // @Description: Enables disarmed-only UTB mode and optional shadow diagnostics after reboot. Never grants motor authority or arming permission.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_UTB, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SHADOW
    // @DisplayName: UTB shadow experiment
    // @Description: Runs diagnostic rate PID and BF_X allocation beside AP control. Requires logging to start. No actuator output. Off resets history and time budget latch.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SHADOW", 2, AP_UTB, _shadow, 0),

    // @Param: LOG_RATE
    // @DisplayName: UTB diagnostic log rate
    // @Description: Requested diagnostic sets per second, independent of controller rate. 400 Hz is experimental. Hardware CPU and logger limits remain unverified.
    // @Range: 1 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("LOG_RATE", 3, AP_UTB, _log_rate, 200),

    // @Param: RAT_RLL_P
    // @DisplayName: UTB shadow RLL P
    // @Description: Diagnostic RLL P, units s/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_P", 4, AP_UTB, _p[0], 0),

    // @Param: RAT_RLL_I
    // @DisplayName: UTB shadow RLL I
    // @Description: Diagnostic RLL I, units 1/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_I", 5, AP_UTB, _i[0], 0),

    // @Param: RAT_RLL_D
    // @DisplayName: UTB shadow RLL D
    // @Description: Diagnostic RLL D, units s^2/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_D", 6, AP_UTB, _d[0], 0),

    // @Param: RAT_RLL_IMAX
    // @DisplayName: UTB shadow RLL IMAX
    // @Description: Diagnostic RLL IMAX, units dimensionless. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_IMAX", 7, AP_UTB, _imax[0], 0),

    // @Param: RAT_RLL_D_HZ
    // @DisplayName: UTB shadow RLL D_HZ
    // @Description: Diagnostic RLL D_HZ, units Hz. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_D_HZ", 8, AP_UTB, _d_hz[0], 0),

    // @Param: RAT_PIT_P
    // @DisplayName: UTB shadow PIT P
    // @Description: Diagnostic PIT P, units s/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_P", 9, AP_UTB, _p[1], 0),

    // @Param: RAT_PIT_I
    // @DisplayName: UTB shadow PIT I
    // @Description: Diagnostic PIT I, units 1/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_I", 10, AP_UTB, _i[1], 0),

    // @Param: RAT_PIT_D
    // @DisplayName: UTB shadow PIT D
    // @Description: Diagnostic PIT D, units s^2/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_D", 11, AP_UTB, _d[1], 0),

    // @Param: RAT_PIT_IMAX
    // @DisplayName: UTB shadow PIT IMAX
    // @Description: Diagnostic PIT IMAX, units dimensionless. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_IMAX", 12, AP_UTB, _imax[1], 0),

    // @Param: RAT_PIT_D_HZ
    // @DisplayName: UTB shadow PIT D_HZ
    // @Description: Diagnostic PIT D_HZ, units Hz. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_D_HZ", 13, AP_UTB, _d_hz[1], 0),

    // @Param: RAT_YAW_P
    // @DisplayName: UTB shadow YAW P
    // @Description: Diagnostic YAW P, units s/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_P", 14, AP_UTB, _p[2], 0),

    // @Param: RAT_YAW_I
    // @DisplayName: UTB shadow YAW I
    // @Description: Diagnostic YAW I, units 1/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_I", 15, AP_UTB, _i[2], 0),

    // @Param: RAT_YAW_D
    // @DisplayName: UTB shadow YAW D
    // @Description: Diagnostic YAW D, units s^2/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_D", 16, AP_UTB, _d[2], 0),

    // @Param: RAT_YAW_IMAX
    // @DisplayName: UTB shadow YAW IMAX
    // @Description: Diagnostic YAW IMAX, units dimensionless. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_IMAX", 17, AP_UTB, _imax[2], 0),

    // @Param: RAT_YAW_D_HZ
    // @DisplayName: UTB shadow YAW D_HZ
    // @Description: Diagnostic YAW D_HZ, units Hz. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_D_HZ", 18, AP_UTB, _d_hz[2], 0),

#if AP_UTB_BENCH_ENABLED
    // @Param: B_WAIT_US
    // @DisplayName: Benchmark diagnostic acquire threshold
    // @Description: Counts main BLOCK logger acquire durations above this threshold. Zero disables only the exceed counter. Not a flight safety limit.
    // @Range: 0 1000000
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("B_WAIT_US", 19, AP_UTB, _bench_wait, 0),
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Param: B_FAIL
    // @DisplayName: Benchmark simulated thread creation failure
    // @Description: SITL only: bypasses UTB worker creation with a false result without exhausting heap. Does not alter arming guards.
    // @Values: 0:Normal,1:Simulated failure
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("B_FAIL", 20, AP_UTB, _bench_fail, 0),
#endif
#endif
    AP_GROUPEND
};

AP_UTB::AP_UTB()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_UTB::init(const AP_AHRS_View &view)
{
    _enabled_at_boot = _enable.get() == 1;
    if (_enabled_at_boot) {
        _provider.init(view);
    }
}

void AP_UTB::update_state()
{
    if (_enabled_at_boot && !shadow_requested()) {
        _provider.sample(_state);
    }
}

bool AP_UTB::Queue::publish(const Snapshot &value, bool enqueue)
{
    // The status mailbox is best-effort and independent of SPSC data publication.
    if (_sem->take_nonblocking()) {
        _status = value;
        _sem->give();
    }
    if (!enqueue) {
        return true;
    }
    if (_objects.space() == 0) {
        return false;
    }
    // Single producer: the consumer can only increase space after this check.
    ++_enqueued;
    const uint8_t depth = MIN(uint32_t(CAPACITY), _objects.available() + 1);
    _high_water.store(MAX(_high_water.load(), depth));
    return _objects.push(value);
}

bool AP_UTB::Queue::pop(Snapshot &value)
{
    return _objects.pop(value);
}

bool AP_UTB::Queue::read_status(Snapshot &value)
{
    if (!_sem->take_nonblocking()) {
        return false;
    }
    value = _status;
    _sem->give();
    return true;
}

void AP_UTB::reset_shadow()
{
    _controller.reset();
    ++_epoch;
    _capture = {};
    _snapshot = {};
    _context_valid = false;
    _last_imu_us = 0;
    _last_gyro = 255;
    _experiment_started = false;
    _budget_latched = false;
    _consecutive_overruns = 0;
    _last_log_us = 0;
    _rate_start_us = 0;
    _rate_start_written = _written_sets.load();
}

void AP_UTB::capture_target(const Vector3f &target, uint64_t time_us, uint8_t mode)
{
    if (!shadow_requested()) {
        return;
    }
    _capture.target = target;
    _capture.time_us = time_us;
    _capture.seq = _seq + 1;
    _capture.mode = mode;
    _capture.valid = !target.is_nan() && !target.is_inf();
}

void AP_UTB::evaluate(const Inputs &input, const AP_UTB_RateController::Gains (&gains)[3], Snapshot &s)
{
    s = {};
    s.input = input;
    s.seq = ++_seq;
    s.epoch = _epoch.load();
    s.flags = ENABLED | SHADOW_REQUESTED;
    if (input.logging_available) {
        s.flags |= LOGGING_AVAILABLE | SHADOW_OBSERVABLE;
    } else {
        s.observation = ObservationReason::LOGGING_UNAVAILABLE;
    }
    const uint32_t missing = _missing.load();
    const bool backend_failed = _backend_failed.load() || missing != _last_observed_missing;
    _last_observed_missing = missing;
    if (backend_failed) {
        s.flags &= ~SHADOW_OBSERVABLE;
        s.observation = ObservationReason::WRITE_FAILED;
    }
    const bool gyro_changed = _context_valid && input.state.primary_gyro != _last_gyro;
    if (gyro_changed || !_context_valid || input.mode != _last_mode || input.armed != _last_armed ||
        input.frame_class != _last_class || input.frame_type != _last_type) {
        _controller.reset();
    }
    _context_valid = true;
    _last_mode = input.mode;
    _last_armed = input.armed;
    _last_class = input.frame_class;
    _last_type = input.frame_type;
    _last_gyro = input.state.primary_gyro;
    const bool nominal_valid = isfinite(input.nominal_dt) && input.nominal_dt > 0;
    const bool dt_valid = nominal_valid && isfinite(input.dt) && input.dt >= input.nominal_dt * 0.25f &&
                          input.dt <= input.nominal_dt * 4;
    const uint32_t age_us = nominal_valid ? uint32_t(MIN(0.1f, input.nominal_dt * 4) * 1.0e6f) : 0;
    const bool state_valid = nominal_valid && input.state.valid(uint32_t(input.time_us), age_us) &&
                             input.state.imu_updated_us != _last_imu_us;
    _last_imu_us = input.state.imu_updated_us;
    if (state_valid) {
        s.flags |= STATE_VALID;
    }
    if (_budget_latched) {
        s.policy = PolicyReason::TIME_BUDGET;
        _controller.reset();
        return;
    }
    if (!_experiment_started && !input.logging_available) {
        s.policy = PolicyReason::EXPERIMENT_SUPPRESSED;
        _controller.reset();
        return;
    }
    _experiment_started = true;
    if (input.fast_rate) {
        s.calculation = CalculationReason::FAST_RATE_UNSUPPORTED;
    } else if (!AP_UTB_MotorMixer::supports_frame(input.frame_class, input.frame_type)) {
        s.calculation = CalculationReason::FRAME_MISMATCH;
    } else if (!input.acro_available) {
        s.calculation = CalculationReason::ACRO_UNAVAILABLE;
    } else if (!dt_valid) {
        s.calculation = CalculationReason::DT_INVALID;
    } else if (!state_valid) {
        s.calculation = CalculationReason::STATE_INVALID;
    } else if (!input.reference_valid || !isfinite(input.thrust) || input.thrust < 0 || input.thrust > 1) {
        s.calculation = CalculationReason::REFERENCE_INVALID;
    } else if (!_controller.configure(gains, input.nominal_dt)) {
        s.calculation = CalculationReason::CONFIG_INVALID;
    }
    if (s.calculation != CalculationReason::NONE) {
        _controller.reset();
        return;
    }
    s.flags |= CONTROLLER_EVALUATED;
    s.control = _controller.update(input.state, input.reference, input.dt);
    if (!s.control.valid) {
        s.calculation = s.control.priming ? (gyro_changed ? CalculationReason::PRIMARY_GYRO_CHANGED : CalculationReason::PRIMING) : CalculationReason::CONTROLLER_INVALID;
        return;
    }
    s.flags |= CONTROLLER_VALID | MIXER_EVALUATED;
    s.mixer = _mixer.mix(s.control, input.thrust, input.frame_class, input.frame_type);
    if (!s.mixer.valid) {
        s.calculation = CalculationReason::MIXER_INVALID;
        _controller.reset();
        return;
    }
    s.flags |= MIXER_VALID | SHADOW_HEALTHY;
    _controller.feedback(s.control, s.mixer.achieved);
    if (input.logging_available && !backend_failed) {
        s.flags |= SHADOW_OBSERVABLE;
    }
}

void AP_UTB::update(const Inputs &input)
{
    _logging_requested.store(shadow_requested());
    if (!shadow_requested()) {
        if (_was_requested) {
            reset_shadow();
        }
        _was_requested = false;
        _snapshot.input.mode = input.mode;
        _snapshot.input.time_us = input.time_us;
        _queue.publish(_snapshot, false);
        return;
    }
    _was_requested = true;
    const uint32_t start = input.started_us;
    AP_UTB_RateController::Gains gains[3];
    for (uint8_t j = 0; j < 3; j++) {
        gains[j] = {_p[j].get(), _i[j].get(), _d[j].get(), _imax[j].get(), _d_hz[j].get()};
    }
    evaluate(input, gains, _snapshot);
    _state = input.state;
    if (!(_snapshot.flags & STATE_VALID)) {
        _snapshot.input.state.rates_rads.zero();
    }
    if (!input.reference_valid) {
        _snapshot.input.reference.zero();
    }
    if (!_snapshot.input.ap_target.valid) {
        _snapshot.input.ap_target.target.zero();
    }
    if (!isfinite(_snapshot.input.dt)) {
        _snapshot.input.dt = 0;
    }
    if (!isfinite(_snapshot.input.thrust)) {
        _snapshot.input.thrust = 0;
    }
    const int16_t rate = _log_rate.get();
    if (rate < 1 || rate > 400 || (_shadow.get() != 0 && _shadow.get() != 1)) {
        _snapshot.control = {};
        _snapshot.mixer = {};
        _snapshot.flags &= ~(CONTROLLER_VALID | MIXER_VALID | SHADOW_HEALTHY);
        _snapshot.calculation = CalculationReason::CONFIG_INVALID;
        _controller.reset();
    }
    _snapshot.requested_log_rate = rate >= 1 && rate <= 400 ? rate : 0;
    if (_rate_start_us == 0) {
        _rate_start_us = input.time_us;
    }
    const uint64_t elapsed = input.time_us - _rate_start_us;
    _snapshot.effective_log_rate = elapsed > 0 ? (_written_sets.load() - _rate_start_written) * 1.0e6f / elapsed : 0;
    _snapshot.drops = _drops;
    _snapshot.missing = _missing;
    bool selected = false;
    if (rate != _last_log_rate) {
        _last_log_us = 0;
        _last_log_rate = rate;
    }
    if (rate >= 1 && rate <= 400 && (_last_log_us == 0 || input.time_us >= _last_log_us)) {
        const uint64_t period_us = uint64_t(1.0e6f / rate);
        _last_log_us = (_last_log_us == 0 ? input.time_us : _last_log_us) + period_us;
        if (_last_log_us <= input.time_us) {
            _last_log_us = input.time_us + period_us;
        }
        selected = true;
    }
    if (_snapshot.calculation == CalculationReason::PRIMARY_GYRO_CHANGED) {
        selected = true; // Preserve the one-cycle reset event when diagnostic capacity permits.
    }
    // Commit all metadata before publication; the consumer only sees complete copies.
    _snapshot.execution_us = AP_HAL::micros() - start;
    record_execution(_snapshot.execution_us, input.nominal_dt);
    _snapshot.max_execution_us = _max_execution_us;
    _snapshot.overruns = _overruns;
    _snapshot.drops = _drops.load();
    _snapshot.missing = _missing.load();
    publish_diagnostics(_snapshot, selected);

}

bool AP_UTB::publish_diagnostics(Snapshot &snapshot, bool selected)
{
    if (selected) {
        ++_requested_sets;
    }
    if (!_queue.publish(snapshot, selected)) {
        if (selected) {
            ++_drops;
            snapshot.flags &= ~SHADOW_OBSERVABLE;
            snapshot.observation = ObservationReason::QUEUE_FULL;
            snapshot.drops = _drops.load();
            // One best-effort status publication, never retry the dropped data set.
            _queue.publish(snapshot, false);
        }
        return false;
    }
    return true;
}

void AP_UTB::record_execution(uint32_t elapsed_us, float nominal_dt)
{
    _max_execution_us = MAX(_max_execution_us, elapsed_us);
    if (isfinite(nominal_dt) && nominal_dt > 0 && elapsed_us > nominal_dt * 5.0e4f) {
        _overruns++;
        if (_consecutive_overruns < 3 && ++_consecutive_overruns >= 3) {
            _budget_latched = true;
        }
    } else {
        _consecutive_overruns = 0;
    }
}

void AP_UTB::log_result(bool success)
{
    _backend_failed.store(!success);
    if (success) {
        ++_written_sets;
    } else {
        ++_missing;
    }
}
#endif // AP_UTB_ENABLED
