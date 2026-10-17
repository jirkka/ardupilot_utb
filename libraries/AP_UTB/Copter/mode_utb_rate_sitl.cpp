//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       mode_utb_rate_sitl.cpp
///  @brief      Aktivní mode41: vlastní/AP control selection, frozen config, reference, throttle/envelope, fault-
///  @author     Jiri Kovar
///  @version    2026-10-17
///  @date       2026-10-17
//--------------------------------------------------------------------------------------------------------------------
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
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "../../../ArduCopter/Copter.h"
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
#include "utb_sitl_log.h"
#include <chrono>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Report whether this component is enabled in the current configuration.
///
///  @param[in]  None
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool ModeUTBRateSITL::enabled().
//--------------------------------------------------------------------------------------------------------------------
bool ModeUTBRateSITL::enabled() const
{
    return copter.utb.enabled() && (copter.utb.sitl_owner() == 1 || copter.utb.sitl_owner() == 2);
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Init.
///
///  @param[in]  ignore_checks Ignore checks.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool ModeUTBRateSITL::init(bool ignore_checks).
//--------------------------------------------------------------------------------------------------------------------
bool ModeUTBRateSITL::init(bool ignore_checks)
{
    // AP poloha vůči počátku, kladně dolů, v metrech.
    float down;
    // Rychlost NED pro bezpečnostní vertikální obálku.
    Vector3f velocity;
    // Tři sady PID parametrů; hodnoty se čtou až po úvodních kontrolách.
    AP_UTB_RateController::Gains gains[3];

    if (!enabled() || copter._last_reason != ModeReason::GCS_COMMAND || copter.using_rate_thread ||
        copter.g2.att_enable.get() != 0 || copter.g2.frame_class.get() != 1 || copter.g.frame_type.get() != 18 ||
        copter.utb.shadow_requested() || fabsf(copter.scheduler.get_loop_period_s() - 0.0025f) > 1.0e-7f) {
        return false;
    }
#if AP_UTB_BENCH_ENABLED
    if (copter.utb.synthetic_requested()) {
        return false;
    }
#endif
    _angle_source = copter.utb.sitl_attitude();
    _angle_config = {copter.utb.sitl_angle_roll_p(), copter.utb.sitl_angle_pitch_p(), copter.utb.sitl_angle_rate_max()};
    _angle_amplitude = copter.utb.sitl_angle();
    _yaw_rate = copter.utb.sitl_yaw_rate();
    _throttle_offset = copter.utb.sitl_throttle_offset();
    _attitude.reset();
    if (_angle_source < 0 || _angle_source > 2 || (_angle_source == 1 && copter.utb.sitl_owner() != 1)) {
        return false;
    }
    if (_angle_source != 0 && (!_attitude.configure(_angle_config) || !isfinite(_angle_amplitude) ||
                               _angle_amplitude < 0 || _angle_amplitude > 5 || !isfinite(_yaw_rate) || fabsf(_yaw_rate) > 0.1f ||
                               !isfinite(_throttle_offset) || fabsf(_throttle_offset) > 0.001f ||
                               (copter.utb.sitl_profile() != 0 && copter.utb.sitl_profile() != 1 && copter.utb.sitl_profile() != 2 &&
                                copter.utb.sitl_profile() != 4 && copter.utb.sitl_profile() != 5 && copter.utb.sitl_profile() != 6) ||
                               (_angle_amplitude * (copter.utb.sitl_profile() == 4 ? sqrtf(2) : 1)) > 5)) {
        return false;
    }
    _angle_reference.zero(); _angle_result = {};
    _ap_angle_calls = 0;
    if (_angle_source != 0) {

        if (!copter.ahrs.get_relative_position_D_origin_float(down) || !copter.ahrs.get_velocity_NED(velocity)) {
            return false;
        }
        _entry_height = _height = -down;
        _vertical_speed = -velocity.z;
    }
    _owner = copter.utb.sitl_owner();
    _profile = copter.utb.sitl_profile();
    _amplitude = copter.utb.sitl_amplitude();
    _delay = copter.utb.sitl_delay();
    _antiwindup = copter.utb.sitl_antiwindup();
#if AP_SCRIPTING_ENABLED
    if (_antiwindup == 1 && AP::scripting() != nullptr && AP::scripting()->enabled()) {
        return false;
    }
#endif
    if (_profile < 0 || _profile > 7 || _delay < 0 || _delay > 1 ||
        _antiwindup < 0 || _antiwindup > 1 || (_antiwindup == 1 && _owner != 1) ||
        !isfinite(_amplitude) || _amplitude < 0 || _amplitude > 0.3f) {
        return false;
    }
    copter.utb.sitl_gains(gains);
    for (const auto &gain : gains) {
        if (_owner == 1 && gain.p <= 0) {
            return false;
        }
    }
    if (!_rate.configure(gains, _antiwindup == 1, _delay == 1)) {
        return false;
    }
    copter.utb.sample_state(_sample);
    if (!_sample.valid(AP_HAL::micros(), 10000)) {
        return false;
    }
    _seq = _cycle = _ap_calls = _motor_calls = 0;
    _running = _complete = false;
    _reference.zero();
    _raw_applied.zero();
    _raw_previous.zero();
    _raw_valid = _previous_raw_valid = false;
    _command_seq = _previous_raw_seq = 0;
    _allocation = {};
    _target.zero();
    _p.zero(); _i.zero(); _d.zero();
    _hover = motors->armed() ? motors->get_throttle_in() : motors->get_throttle_hover();
    if (!isfinite(_hover) || _hover <= 0 || _hover >= 1) {
        return false;
    }
    attitude_control->reset_target_and_rate(true);
    attitude_control->reset_rate_controller_I_terms();
    attitude_control->get_rate_roll_pid().reset_filter();
    attitude_control->get_rate_pitch_pid().reset_filter();
    attitude_control->get_rate_yaw_pid().reset_filter();
    zero_rpy();
    return true;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Allows arming.
///
///  @param[in]  method Method.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool ModeUTBRateSITL::allows_arming(AP_Arming::Method method).
//--------------------------------------------------------------------------------------------------------------------
bool ModeUTBRateSITL::allows_arming(AP_Arming::Method method) const
{
    return method == AP_Arming::Method::MAVLINK && enabled() &&
           copter.utb.sitl_owner() == _owner && _rate.fault() == AP_UTB_SITLRate::Fault::NONE &&
           copter.g2.att_enable.get() == 0 && !copter.using_rate_thread;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Clear normalized roll/pitch/yaw commands and feed-forward terms.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void ModeUTBRateSITL::zero_rpy().
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBRateSITL::zero_rpy()
{
    motors->set_roll(0); motors->set_roll_ff(0);
    motors->set_pitch(0); motors->set_pitch_ff(0);
    motors->set_yaw(0); motors->set_yaw_ff(0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Exit.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void ModeUTBRateSITL::exit().
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBRateSITL::exit()
{
    _rate.reset();
    _attitude.reset();
    _angle_reference.zero(); _angle_result = {};
    _running = false;
    _complete = false;
    _reference.zero();
    _raw_applied.zero();
    _raw_previous.zero();
    _raw_valid = _previous_raw_valid = false;
    zero_rpy();
    attitude_control->reset_target_and_rate(true);
    attitude_control->reset_rate_controller_I_terms();
    attitude_control->get_rate_roll_pid().reset_filter();
    attitude_control->get_rate_pitch_pid().reset_filter();
    attitude_control->get_rate_yaw_pid().reset_filter();
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Run.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void ModeUTBRateSITL::run().
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBRateSITL::run()
{
    // Normalizovaný povel plynu převzatý od pilota.
    float throttle;
    // Příznak plného spool stavu původních AP motorů.
    bool full;

    if (_rate.fault() != AP_UTB_SITLRate::Fault::NONE) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->set_throttle_out(0, false, copter.g.throttle_filt);
        return;
    }
    motors->set_desired_spool_state(copter.ap.throttle_zero ? AP_Motors::DesiredSpoolState::GROUND_IDLE :
                                    AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    throttle = get_pilot_desired_throttle() + (_angle_source != 0 ? _throttle_offset : 0);
    full = motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;
    if (motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN ||
        motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        throttle = 0;
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
    } else if (full && !motors->limit.throttle_lower) {
        set_land_complete(false);
    }
    if (!isfinite(throttle) || throttle < 0 || throttle > 1) {
        _rate.invalidate(AP_UTB_SITLRate::Fault::STATE);
        throttle = 0;
    }
    if (!_complete && copter.utb.sitl_run() && motors->armed() && full) {
        _running = true;
    }
    if (_angle_source != 0) {
        _angle_reference = AP_UTB_SITLAttitude::reference(_profile, _running ? _cycle : 0,
                           radians(_angle_amplitude), _running ? _yaw_rate : 0);
    } else {
        _reference = AP_UTB_SITLRate::reference(_profile, _running ? _cycle : 0, _amplitude);
    }
    if (_running && ++_cycle >= (_angle_source != 0 ? AP_UTB_SITLAttitude::DURATION_CYCLES :
                                 AP_UTB_SITLRate::duration_cycles(_profile))) {
        _complete = true;
        _running = false;
    }
    // Identical rate shaping for both owners; this function does not execute the AP rate PID.
    if (_angle_source == 0) {
        attitude_control->input_rate_bf_roll_pitch_yaw_2_rads(_reference.x, _reference.y, _reference.z);
    }
    attitude_control->set_throttle_out(throttle, false, copter.g.throttle_filt);
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Execute the selected SITL controller and apply its guarded normalized R/P/Y request.
///
///  @param[in]  dt Sample interval in seconds.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void ModeUTBRateSITL::rate_control(float dt).
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBRateSITL::rate_control(float dt)
{
    // Norma požadované roll/pitch úhlové rychlosti v rad/s.
    float rp;
    // AP poloha vůči počátku, kladně dolů, v metrech.
    float down;
    // Bitová mapa původních AP motorových limitů.
    uint8_t limits;
    // Index průchodu pevnou kolekcí.
    uint8_t a;

    // Čas začátku zůstává před konstrukcí pracovních objektů a odečty vstupů.
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    // Předchozí latched chyba pro jednorázové ohlášení nové chyby.
    const AP_UTB_SITLRate::Fault previous_fault = _rate.fault();
    // PID konfigurace tohoto cyklu; přiřazení zůstává na původním místě.
    AP_UTB_RateController::Gains gains[3];
    // Bezpečnostní rychlost NED získaná během kontroly vertikální obálky.
    Vector3f velocity;
    // Snapshot vstupu pro právě jeden UTB/AP rate krok.
    AP_UTB_SITLRate::Inputs in;
    // Rozklad AP PID výsledků, použitý pouze pro AP owner větev.
    AP_PIDInfo info[3];
    // Povel aktuálního vlastníka před případným zpožděním.
    Vector3f applied;
    // Povel po experimentálním jednovzorkovém transportu.
    Vector3f transported;
    // Nesaturovaný povel pro následující alokační zpětnou vazbu.
    Vector3f raw;
    ++_seq;
    _time_us = AP_HAL::micros64();
    _dt = dt;
    copter.utb.sitl_gains(gains);
    if (!enabled() || copter.utb.sitl_owner() != _owner || copter.utb.sitl_profile() != _profile ||
        !(copter.utb.sitl_amplitude() <= _amplitude && copter.utb.sitl_amplitude() >= _amplitude) ||
        !_rate.configuration_matches(gains) || copter.utb.sitl_delay() != _delay ||
        copter.utb.sitl_antiwindup() != _antiwindup || copter.utb.sitl_attitude() != _angle_source ||
        (_angle_source != 0 && (!_attitude.configuration_matches({copter.utb.sitl_angle_roll_p(),
                                copter.utb.sitl_angle_pitch_p(), copter.utb.sitl_angle_rate_max()}) ||
                                !(copter.utb.sitl_angle() <= _angle_amplitude && copter.utb.sitl_angle() >= _angle_amplitude) || !(copter.utb.sitl_yaw_rate() <= _yaw_rate && copter.utb.sitl_yaw_rate() >= _yaw_rate) ||
                                !(copter.utb.sitl_throttle_offset() <= _throttle_offset && copter.utb.sitl_throttle_offset() >= _throttle_offset))) ||
        copter.utb.shadow_requested() || copter.any_failsafe_triggered()) {
        _rate.invalidate(AP_UTB_SITLRate::Fault::CONFIG);
    }
#if AP_SCRIPTING_ENABLED
    if (_antiwindup == 1 && AP::scripting() != nullptr && AP::scripting()->enabled()) {
        _rate.invalidate(AP_UTB_SITLRate::Fault::CONFIG);
    }
#endif
    copter.utb.sample_state(_sample);
    _target = attitude_control->rate_bf_targets();
    if (_angle_source != 0 && _rate.fault() == AP_UTB_SITLRate::Fault::NONE) {
        _angle_result = {};
        if (_angle_source == 1) {
            _angle_result = _attitude.update(_sample, _angle_reference, dt, AP_HAL::micros());
            _target = _angle_result.rates;
            _reference = _target;
            if (!_angle_result.valid) {
                _rate.invalidate(AP_UTB_SITLRate::Fault::ATTITUDE);
            }
        } else if (_sample.valid(AP_HAL::micros(), 10000) && isfinite(dt) && dt >= 0.000625f && dt <= 0.01f) {
            // Comparison source only; never executed when UTB owns the outer loop.
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
                _angle_reference.x, _angle_reference.y, _angle_reference.z);
            ++_ap_angle_calls;
            _target = attitude_control->rate_bf_targets();
            rp = Vector2f(_target.x, _target.y).length();
            if (rp > _angle_config.rate_max) {
                _target.x *= _angle_config.rate_max / rp;
                _target.y *= _angle_config.rate_max / rp;
                _angle_result.limited = true;
                if (_owner == 2) {
                    _rate.invalidate(AP_UTB_SITLRate::Fault::ENVELOPE);
                }
            }
            _reference = _target;
            _sample.attitude_body_to_ned.to_euler(_angle_result.euler);
            _angle_result.valid = true;
        }

        if (!copter.ahrs.get_relative_position_D_origin_float(down) || !copter.ahrs.get_velocity_NED(velocity)) {
            _rate.invalidate(AP_UTB_SITLRate::Fault::VERTICAL);
        } else {
            _height = -down; _vertical_speed = -velocity.z;
            if (!isfinite(_height) || !isfinite(_vertical_speed) || fabsf(_height - _entry_height) > 2 ||
                fabsf(_vertical_speed) > 0.5f) {
                _rate.invalidate(AP_UTB_SITLRate::Fault::VERTICAL);
            }
        }
    }
    in.state = _sample;
    in.target = _target;
    in.now_us = AP_HAL::micros();
    in.seq = _seq;
    in.dt = dt;
    in.nominal_dt = copter.scheduler.get_loop_period_s();
    in.frame_class = copter.g2.frame_class.get();
    in.frame_type = copter.g.frame_type.get();
    in.fast_rate = copter.using_rate_thread || copter.g2.att_enable.get() != 0;
    in.armed = motors->armed();
    in.full_spool = motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;
    in.allocation_limited = motors->limit.roll || motors->limit.pitch || motors->limit.yaw || motors->limit.throttle_upper;
    _result = _rate.step(in, _owner == 1);
    _p.zero(); _i.zero(); _d.zero();
    if (_owner == 1) {
        for (a = 0; a < 3; a++) {
            _p[a] = _result.axis[a].p; _i[a] = _result.axis[a].i; _d[a] = _result.axis[a].d;
        }
    } else if (_result.valid) {
        ++_ap_calls;
        attitude_control->rate_controller_run();
        info[0] = attitude_control->get_rate_roll_pid().get_pid_info();
        info[1] = attitude_control->get_rate_pitch_pid().get_pid_info();
        info[2] = attitude_control->get_rate_yaw_pid().get_pid_info();
        for (a = 0; a < 3; a++) {
            _p[a] = info[a].P; _i[a] = info[a].I; _d[a] = info[a].D;
        }
        if (!in.armed || !in.full_spool) {
            zero_rpy();
        }
    } else {
        zero_rpy();
    }
    applied = _owner == 1 ? _result.command :
              Vector3f(motors->get_roll() + motors->get_roll_ff(),
                       motors->get_pitch() + motors->get_pitch_ff(), motors->get_yaw() + motors->get_yaw_ff());
    if (applied.is_nan() || applied.is_inf()) {
        _rate.invalidate(AP_UTB_SITLRate::Fault::STATE);
    } else if (_result.valid && (_antiwindup == 0 || _owner != 1) &&
               (fabsf(applied.x) >= 1 || fabsf(applied.y) >= 1 || fabsf(applied.z) >= 1)) {
        _rate.invalidate(AP_UTB_SITLRate::Fault::SATURATION);
    }
    transported = _rate.transport(applied, _delay == 1, _result.valid && in.armed && in.full_spool);
    if (_result.valid && in.armed && in.full_spool) {
        raw = applied;
        if (_owner == 1) {
            for (a = 0; a < 3; a++) {
                raw[a] = _result.axis[a].raw;
            }
        }
        _raw_applied = _delay == 1 ? _raw_previous : raw;
        _command_seq = _delay == 1 ? _previous_raw_seq : _seq;
        _raw_valid = _delay == 1 ? _previous_raw_valid : true;
        _raw_previous = raw;
        _previous_raw_seq = _seq;
        _previous_raw_valid = true;
    } else {
        _raw_applied.zero(); _raw_previous.zero();
        _command_seq = _previous_raw_seq = 0;
        _raw_valid = _previous_raw_valid = false;
    }
    motors->set_roll(transported.x); motors->set_roll_ff(0);
    motors->set_pitch(transported.y); motors->set_pitch_ff(0);
    motors->set_yaw(transported.z); motors->set_yaw_ff(0);
    if (_rate.fault() != AP_UTB_SITLRate::Fault::NONE) {
        zero_rpy();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->set_throttle_out(0, false, copter.g.throttle_filt);
        if (previous_fault == AP_UTB_SITLRate::Fault::NONE) {
            limits = motors->limit.roll | (motors->limit.pitch << 1) | (motors->limit.yaw << 2) |
                     (motors->limit.throttle_lower << 3) | (motors->limit.throttle_upper << 4);
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UTB SITL fault=%u limits=%u", unsigned(_rate.fault()), unsigned(limits));
        }
    }
    _execution_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Read the completed AP motor allocation for feedback on the next rate cycle.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void ModeUTBRateSITL::capture_allocation().
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBRateSITL::capture_allocation()
{
    // Pořadí motoru vrácené read-only AP getterem.
    uint8_t order;
    // Index průchodu pevnou kolekcí.
    uint8_t m;

    // Pevný snapshot skutečné AP alokace pro následující simulovaný cyklus.
    AP_UTB_SITLAllocation::Capture c;
    // Volitelný AP singleton; přiřadí se před explicitní kontrolou nullptr.
    const AP_MotorsMatrix *matrix;
    c.output_seq = _seq; c.command_seq = _command_seq; c.sampled_us = AP_HAL::micros();
    c.raw = _raw_applied; c.pid_valid = _raw_valid;
    c.command = {motors->get_roll(), motors->get_pitch(), motors->get_yaw()};
    c.gain = motors->thr_lin.get_compensation_gain();
    c.armed = motors->armed();
    c.full_spool = motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;
    c.boost = motors->get_thrust_boost();
    c.limits = motors->limit.roll | (motors->limit.pitch << 1) | (motors->limit.yaw << 2) |
               (motors->limit.throttle_lower << 3) | (motors->limit.throttle_upper << 4);
    // AP singleton může chybět; platnost se před každým použitím ověřuje.
    matrix = AP_MotorsMatrix::get_singleton();
    c.factors_available = matrix != nullptr;
    if (matrix != nullptr) {
        for (m = 0; m < 4; m++) {

            c.factors_available &= matrix->get_factors(m, c.factors[m].x, c.factors[m].y, c.factors[m].z,
                                   c.collective[m], order) && motors->get_thrust(m, c.post_thrust[m]);
            c.thrust[m] = matrix->get_thrust_rpyt_out(m);
        }
    }
    _allocation = AP_UTB_SITLAllocation::reconstruct(c);
    _rate.capture_allocation(_allocation);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Capture the completed SITL output and write the enabled diagnostic packets.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void ModeUTBRateSITL::log_cycle().
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBRateSITL::log_cycle()
{
#if HAL_LOGGING_ENABLED
    // Normalizovaný aktuátorový výstup přečtený pouze pro diagnostiku.
    float actuator;
    // Bitová mapa blokované integrace po jednotlivých osách.
    uint8_t blocked;
    // Index průchodu pevnou kolekcí.
    uint8_t a;
    // Index průchodu pevnou kolekcí.
    uint8_t m;
#endif

    ++_motor_calls;
    capture_allocation();
#if HAL_LOGGING_ENABLED
    const float output[] = {motors->get_roll(), motors->get_pitch(), motors->get_yaw()};
    for (a = 0; a < 3; a++) {
        const log_UARX row = {LOG_PACKET_HEADER_INIT(LOG_UARX_MSG), _time_us, _seq, a, uint8_t(_owner),
                              uint8_t(_rate.fault()), uint8_t(motors->armed()), uint8_t(motors->get_spool_state()),
                              _dt, _reference[a], _target[a], _sample.rates_rads[a], _p[a], _i[a], _d[a], output[a]
                             };
        copter.logger.WriteBlock_first_succeed(&row, sizeof(row));
    }
    log_UAMT row = {LOG_PACKET_HEADER_INIT(LOG_UAMT_MSG), _time_us, _seq,
                    uint8_t(_rate.fault() != AP_UTB_SITLRate::Fault::NONE ? 3 : _complete ? 2 : _running ? 1 : 0),
                    uint8_t(_rate.fault()), {}, {}, _ap_calls, _rate.calls(), _motor_calls, _execution_us
                   };
    for (m = 0; m < 4; m++) {
        row.pwm[m] = hal.rcout->read(m);
        actuator = 0;
        motors->get_raw_motor_throttle(m, actuator);
        row.actuator[m] = actuator;
    }
    copter.logger.WriteBlock_first_succeed(&row, sizeof(row));
    blocked = 0;
    for (a = 0; a < 3; a++) {
        blocked |= _result.axis[a].integration_blocked << a;
    }
    // Čtení zachycené alokace bez kopie nebo nullable ukazatele.
    const AP_UTB_SITLAllocation::Capture &c = _allocation.capture;
    for (a = 0; a < 3; a++) {
        const log_UAFB feedback = {LOG_PACKET_HEADER_INIT(LOG_UAFB_MSG), _time_us, _seq, c.command_seq, c.sampled_us,
                                   a, uint8_t(_allocation.valid), uint8_t(c.pid_valid), c.limits, blocked,
                                   c.gain, c.raw[a], c.command[a], _allocation.allocated[a], _allocation.residual[a]
                                  };
        copter.logger.WriteBlock_first_succeed(&feedback, sizeof(feedback));
    }
    log_UALT thrust = {LOG_PACKET_HEADER_INIT(LOG_UALT_MSG), _time_us, _seq, c.gain, {}, {},
                       uint8_t(_allocation.valid), c.limits
                      };
    for (m = 0; m < 4; m++) {
        thrust.thrust[m] = c.thrust[m]; thrust.post_thrust[m] = c.post_thrust[m];
    }
    copter.logger.WriteBlock_first_succeed(&thrust, sizeof(thrust));
    if (_angle_source != 0) {
        // Attitude výsledek použitý pouze při zapnutém úhlovém experimentu.
        const AP_UTB_SITLAttitude::Result &attitude = _angle_result;
        const log_UAAT angle = {LOG_PACKET_HEADER_INIT(LOG_UAAT_MSG), _time_us, _seq,
                                uint8_t(_angle_source), uint8_t(_attitude.fault()),
                                _angle_reference.x, _angle_reference.y, attitude.euler.x, attitude.euler.y, attitude.euler.z,
                                attitude.error.x, attitude.error.y, _target.x, _target.y, _target.z, uint8_t(attitude.limited)
                               };
        const log_UAST status = {LOG_PACKET_HEADER_INIT(LOG_UAST_MSG), _time_us, _seq,
                                 _attitude.calls(), _ap_angle_calls, _height, _vertical_speed, motors->get_throttle_in()
                                };
        copter.logger.WriteBlock_first_succeed(&angle, sizeof(angle));
        copter.logger.WriteBlock_first_succeed(&status, sizeof(status));
    }
#endif
}
#endif // AP_UTB_RATE_SITL_AVAILABLE

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private function prototypes
