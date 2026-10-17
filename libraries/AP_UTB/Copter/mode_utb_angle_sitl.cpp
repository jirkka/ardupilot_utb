//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file mode_utb_angle_sitl.cpp
/// @brief Pilotní RC ANGLE integrace původní UTB kaskády a AP motorové infrastruktury, pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-17
/// @date 2026-10-17
//--------------------------------------------------------------------------------------------------------------------
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "../../../ArduCopter/Copter.h"
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
#include "utb_sitl_log.h"
#include "utb_angle_log.h"
#include <chrono>
// End Include Files

//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Ověřit vlastní explicitní opt-in.
/// @param[in] None
/// @return True při validním kontraktu.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
bool ModeUTBAngleSITL::enabled() const
{
    return copter.utb.enabled() && copter.utb.angle_enable() == 1;
}

// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Ověřit zmrazenou konfiguraci a konflikty.
/// @param[in] None
/// @return True při validním kontraktu.
/// @details Soukromá pomocná metoda. Pouze HAL_BOARD_SITL.
bool ModeUTBAngleSITL::configuration_valid() const
{
    if (!enabled() || copter.using_rate_thread || copter.g2.att_enable.get() != 0 ||
        copter.g2.frame_class.get() != 1 || copter.g.frame_type.get() != 18 || copter.utb.shadow_requested() ||
        !(_config.angle_deg <= copter.utb.angle_max() && _config.angle_deg >= copter.utb.angle_max()) ||
        !(_config.yaw_rate <= copter.utb.angle_yaw_max() && _config.yaw_rate >= copter.utb.angle_yaw_max()) ||
        _antiwindup != copter.utb.angle_aw() ||
        fabsf(copter.scheduler.get_loop_period_s() - 0.0025f) > 1.0e-7f) {
        return false;
    }
#if AP_UTB_BENCH_ENABLED
    if (copter.utb.synthetic_requested()) {
        return false;
    }
#endif
#if AP_SCRIPTING_ENABLED
    if (AP::scripting() != nullptr && AP::scripting()->enabled()) {
        return false;
    }
#endif
    return true;
}

// End Private functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Odebrat kalibrované RCMAP kanály přes skutečné AP RC rozhraní.
/// @param[in] None
/// @return True při validním kontraktu.
/// @details Soukromá pomocná metoda. Pouze HAL_BOARD_SITL.
bool ModeUTBAngleSITL::read_pilot()
{
    // Validita a časová značka původní AP RC infrastruktury.
    AP_UTB_AngleRC::Signal signal;
    // Střed AP throttle mapy s neprázdnými intervaly na obě strany.
    int16_t midpoint;

    _pilot = {};
    signal = {AP_HAL::millis(), copter.g2.rc_channels.last_input_ms(), copter.g2.rc_channels.get_fs_timeout_ms(),
              copter.g2.rc_channels.has_valid_input(), copter.failsafe.radio
             };
    _pilot.rc_ms = signal.updated_ms;
    _pilot.fault = AP_UTB_AngleRC::signal_fault(signal);
    if (_pilot.fault != AP_UTB_AngleRC::Fault::NONE) {
        return false;
    }
    // Nullable ukazatele vyžaduje původní Copter RC rozhraní.
    if (channel_roll == nullptr || channel_pitch == nullptr || channel_yaw == nullptr || channel_throttle == nullptr) {
        _pilot.fault = AP_UTB_AngleRC::Fault::SIGNAL;
        return false;
    }
    if (!AP_UTB_AngleRC::calibration_valid(*channel_roll, false) ||
        !AP_UTB_AngleRC::calibration_valid(*channel_pitch, false) ||
        !AP_UTB_AngleRC::calibration_valid(*channel_yaw, false) ||
        !AP_UTB_AngleRC::calibration_valid(*channel_throttle, true)) {
        _pilot.fault = AP_UTB_AngleRC::Fault::CALIBRATION;
        return false;
    }
    midpoint = copter.get_throttle_mid();
    if (midpoint <= 0 || midpoint >= 1000) {
        _pilot.fault = AP_UTB_AngleRC::Fault::CALIBRATION;
        return false;
    }
    _pilot = AP_UTB_AngleRC::read(*channel_roll, *channel_pitch, *channel_yaw, *channel_throttle,
                                  get_pilot_desired_throttle(), _config, signal);
    return _pilot.valid;
}

// End Private functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Resetovat přechodové AP historie bez spuštění AP PID.
/// @param[in] None
/// @return None.
/// @details Soukromá pomocná metoda. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::reset_ap_history()
{
    attitude_control->reset_target_and_rate(true);
    attitude_control->reset_rate_controller_I_terms();
    attitude_control->get_rate_roll_pid().reset_filter();
    attitude_control->get_rate_pitch_pid().reset_filter();
    attitude_control->get_rate_yaw_pid().reset_filter();
}

// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Inicializovat čistý pilotní kontext až po všech kontrolách.
/// @param[in] ignore_checks Původní AP argument, skutečné dt nebo důvod chyby.
/// @return True při validním kontraktu.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
bool ModeUTBAngleSITL::init(bool ignore_checks)
{
    // Již ověřený SITL preset; původní RAT/S_* parametry se nemění.
    AP_UTB_RateController::Gains gains[3];
    // Read-only svislá obálka a vstupní orientace.
    float down;
    Vector3f velocity;
    Vector3f euler;
    // Původní throttle kontext a AP hover parametr; nemění se jimi RC mapa.
    float entry_throttle;
    float ap_hover;

    _config = {copter.utb.angle_max(), copter.utb.angle_yaw_max()};
    _antiwindup = copter.utb.angle_aw();
    if (!configuration_valid() || !AP_UTB_AngleRC::valid_config(_config) ||
        _antiwindup < 0 || _antiwindup > 1 || copter.any_failsafe_triggered()) {
        return false;
    }
    entry_throttle = motors->get_throttle_in();
    ap_hover = motors->get_throttle_hover();
    _saved_mix = attitude_control->get_throttle_mix();
    if (!isfinite(entry_throttle) || entry_throttle < 0 || entry_throttle > 1 ||
        !isfinite(ap_hover) || ap_hover <= 0 || ap_hover >= 1 ||
        !isfinite(_saved_mix) || _saved_mix < 0 || !read_pilot()) {
        return false;
    }
    copter.utb.sample_state(_sample);
    if (!_sample.valid(AP_HAL::micros(), 10000) ||
        !ahrs.get_relative_position_D_origin_float(down) || !ahrs.get_velocity_NED(velocity) ||
        !isfinite(down) || velocity.is_nan() || velocity.is_inf() || fabsf(velocity.z) > 0.5f) {
        return false;
    }
    _sample.attitude_body_to_ned.to_euler(euler);
    if (MAX(fabsf(euler.x), fabsf(euler.y)) > radians(35.0f)) {
        return false;
    }
    AP_UTB_AngleRC::gains(gains);
    if (!_rate.configure(gains, _antiwindup == 1, false) || !_attitude.configure({3, 3, 0.3f})) {
        return false;
    }
    _entry_height = _height = -down;
    _vertical_speed = -velocity.z;
    _seq = _motor_calls = _command_seq = _execution_us = _log_execution_us = _run_execution_us = 0;
    _time_us = 0;
    _angle_reference = _pilot.reference;
    _reference.zero(); _target.zero();
    _p.zero(); _i.zero(); _d.zero(); _raw_applied.zero();
    _raw_valid = _running = false;
    _result = {}; _angle_result = {}; _allocation = {};
    _stop = Stop::NONE;
    _throttle = 0;
    _have_mix = true;
    // Výslovný AP manual mix; žádný angle boost nebo další AP PID.
    attitude_control->set_throttle_mix_value(0.1f);
    reset_ap_history();
    zero_rpy();
    return true;
}

// End Public functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Ponechat standardní AP RC/MAVLink/AUX arming cestu.
/// @param[in] method Původní AP argument, skutečné dt nebo důvod chyby.
/// @return True při validním kontraktu.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
bool ModeUTBAngleSITL::allows_arming(AP_Arming::Method method) const
{
    return (method == AP_Arming::Method::RUDDER || method == AP_Arming::Method::MAVLINK ||
            method == AP_Arming::Method::AUXSWITCH) && configuration_valid() && _pilot.valid &&
           _stop == Stop::NONE && !copter.any_failsafe_triggered();
}

// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Uchovat chybu a vyžádat nulový povel bez AP fallbacku.
/// @param[in] reason Původní AP argument, skutečné dt nebo důvod chyby.
/// @return None.
/// @details Soukromá pomocná metoda. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::stop(Stop reason)
{
    if (_stop == Stop::NONE) {
        _stop = reason;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UTB ANGLE stop=%u rc=%u rate=%u", unsigned(_stop),
                      unsigned(_pilot.fault), unsigned(_rate.fault()));
    }
    _rate.invalidate(AP_UTB_SITLRate::Fault::STATE);
    _pilot.valid = false;
    _angle_reference.zero(); _reference.zero(); _target.zero(); _raw_applied.zero();
    _p.zero(); _i.zero(); _d.zero();
    _raw_valid = _running = false;
    _result = {}; _angle_result = {};
    _throttle = 0;
    zero_rpy();
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    attitude_control->set_throttle_out(0, false, copter.g.throttle_filt);
}

// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Resetovat historii a obnovit zachycený AP mix.
/// @param[in] None
/// @return None.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::exit()
{
    _rate.reset(); _attitude.reset();
    _pilot = {}; _result = {}; _angle_result = {}; _allocation = {};
    _angle_reference.zero(); _reference.zero(); _target.zero(); _raw_applied.zero();
    _p.zero(); _i.zero(); _d.zero();
    _running = _raw_valid = false;
    _stop = Stop::NONE;
    _throttle = 0;
    zero_rpy();
    reset_ap_history();
    if (_have_mix) {
        attitude_control->set_throttle_mix_value(_saved_mix);
        _have_mix = false;
    }
}

// End Public functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Zpracovat skutečný RC snapshot a ruční throttle/spool.
/// @param[in] None
/// @return None.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::run()
{
    // Host wall time mimo logger a rate slot, nikoli virtuální scheduler dt.
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    if (_stop != Stop::NONE) {
        stop(_stop);
    } else if (!configuration_valid()) {
        stop(Stop::CONFIG);
    } else if (!read_pilot()) {
        stop(Stop::RC);
    } else if (copter.any_failsafe_triggered()) {
        stop(Stop::FAILSAFE);
    } else {
        _angle_reference = _pilot.reference;
        _throttle = _pilot.throttle;
        motors->set_desired_spool_state(!motors->armed() ? AP_Motors::DesiredSpoolState::SHUT_DOWN :
                                        copter.ap.throttle_zero ? AP_Motors::DesiredSpoolState::GROUND_IDLE :
                                        AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (!motors->armed() || motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN ||
            motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            _throttle = 0;
        } else if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED && !motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        attitude_control->set_throttle_mix_value(0.1f);
        attitude_control->set_throttle_out(_throttle, false, copter.g.throttle_filt);
    }
    _run_execution_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}

// End Public functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Vykonat právě jednu vlastní původní UTB kaskádu.
/// @param[in] dt Původní AP argument, skutečné dt nebo důvod chyby.
/// @return None.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::rate_control(float dt)
{
    // Host wall time jediného skutečného průchodu kaskádou.
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    // Explicitní validace timestampu předchozího mode run snapshotu.
    AP_UTB_AngleRC::Signal signal;
    // Skutečná výšková obálka, bez polohového nebo výškového regulátoru.
    float down;
    Vector3f velocity;
    // Pevný vstup původního lifecycle/rate helperu.
    AP_UTB_SITLRate::Inputs in;
    // Index osy a povel předaný původnímu AP_Motors.
    uint8_t axis;
    Vector3f transported;

    ++_seq;
    _time_us = AP_HAL::micros64();
    _dt = dt;
    signal = {AP_HAL::millis(), _pilot.rc_ms, copter.g2.rc_channels.get_fs_timeout_ms(),
              copter.g2.rc_channels.has_valid_input(), copter.failsafe.radio
             };
    if (_stop != Stop::NONE) {
        stop(_stop);
    } else if (!configuration_valid()) {
        stop(Stop::CONFIG);
    } else if (AP_UTB_AngleRC::signal_fault(signal) != AP_UTB_AngleRC::Fault::NONE || !_pilot.valid) {
        _pilot.fault = AP_UTB_AngleRC::signal_fault(signal);
        stop(Stop::RC);
    } else if (copter.any_failsafe_triggered()) {
        stop(Stop::FAILSAFE);
    } else {
        copter.utb.sample_state(_sample);
        if (!_sample.valid(AP_HAL::micros(), 10000)) {
            stop(Stop::STATE);
        } else if (!ahrs.get_relative_position_D_origin_float(down) || !ahrs.get_velocity_NED(velocity) ||
                   !isfinite(down) || velocity.is_nan() || velocity.is_inf() ||
                   fabsf(-down - _entry_height) > 2 || fabsf(velocity.z) > 0.5f) {
            stop(Stop::VERTICAL);
        } else {
            _height = -down; _vertical_speed = -velocity.z;
            _angle_result = _attitude.update(_sample, _angle_reference, dt, AP_HAL::micros());
            if (!_angle_result.valid) {
                stop(Stop::ATTITUDE);
            } else {
                _reference = _target = _angle_result.rates;
                in.state = _sample; in.target = _target;
                in.now_us = AP_HAL::micros(); in.seq = _seq; in.dt = dt;
                in.nominal_dt = copter.scheduler.get_loop_period_s();
                in.frame_class = copter.g2.frame_class.get(); in.frame_type = copter.g.frame_type.get();
                in.fast_rate = copter.using_rate_thread || copter.g2.att_enable.get() != 0;
                in.armed = motors->armed();
                in.full_spool = motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED;
                in.allocation_limited = motors->limit.roll || motors->limit.pitch || motors->limit.yaw || motors->limit.throttle_upper;
                _result = _rate.step(in, true);
                if (_rate.fault() != AP_UTB_SITLRate::Fault::NONE || (!_result.valid && !_result.priming)) {
                    stop(Stop::RATE);
                } else {
                    for (axis = 0; axis < 3; axis++) {
                        _p[axis] = _result.axis[axis].p; _i[axis] = _result.axis[axis].i; _d[axis] = _result.axis[axis].d;
                        _raw_applied[axis] = _result.axis[axis].raw;
                    }
                    _raw_valid = _result.valid && in.armed && in.full_spool;
                    _command_seq = _raw_valid ? _seq : 0;
                    _running = in.armed && in.full_spool;
                    transported = _rate.transport(_result.command, false, _raw_valid);
                    motors->set_roll(transported.x); motors->set_roll_ff(0);
                    motors->set_pitch(transported.y); motors->set_pitch_ff(0);
                    motors->set_yaw(transported.z); motors->set_yaw_ff(0);
                }
            }
        }
    }
    _execution_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}

//--------------------------------------------------------------------------------------------------------------------
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Zapsat nulový R/P/Y a FF do původního AP_Motors.
/// @param[in] None
/// @return None.
/// @details Soukromá pomocná metoda. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::zero_rpy()
{
    motors->set_roll(0); motors->set_roll_ff(0);
    motors->set_pitch(0); motors->set_pitch_ff(0);
    motors->set_yaw(0); motors->set_yaw_ff(0);
}

//--------------------------------------------------------------------------------------------------------------------
// End Private functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Zachytit skutečnou AP alokaci po jediném outputu.
/// @param[in] None
/// @return None.
/// @details Soukromá pomocná metoda. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::capture_allocation()
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

//--------------------------------------------------------------------------------------------------------------------
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Uložit skutečnou RC/controller/motor evidenci se společným TimeUS/Seq.
/// @param[in] None
/// @return None.
/// @details Veřejný mode callback. Pouze HAL_BOARD_SITL.
void ModeUTBAngleSITL::log_cycle()
{
    // Host wall time aktuálního post-output capture/logger callbacku.
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
#if HAL_LOGGING_ENABLED
    // Aktuální R/P/Y povely původního AP motorového adaptéru.
    float output[3];
    // Pevné záznamy; naplní se až po zachycení příslušného cyklu.
    log_UARX axis_row;
    log_UAMT motor_row;
    log_UAFB feedback;
    log_UALT thrust;
    log_UAAT angle;
    log_UAST status;
    log_UARC pilot;
    // Read-only reference na členy; zůstávají platné i po aktualizaci snapshotu.
    const AP_UTB_SITLAllocation::Capture &c = _allocation.capture;
    const AP_UTB_SITLAttitude::Result &attitude = _angle_result;
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
    output[0] = motors->get_roll();
    output[1] = motors->get_pitch();
    output[2] = motors->get_yaw();
    for (a = 0; a < 3; a++) {
        axis_row = {LOG_PACKET_HEADER_INIT(LOG_UARX_MSG), _time_us, _seq, a, uint8_t(_owner),
                    uint8_t(_rate.fault()), uint8_t(motors->armed()), uint8_t(motors->get_spool_state()),
                    _dt, _reference[a], _target[a], _sample.rates_rads[a], _p[a], _i[a], _d[a], output[a]
                   };
        copter.logger.WriteBlock_first_succeed(&axis_row, sizeof(axis_row));
    }
    motor_row = {LOG_PACKET_HEADER_INIT(LOG_UAMT_MSG), _time_us, _seq,
                 uint8_t(_rate.fault() != AP_UTB_SITLRate::Fault::NONE ? 3 : _complete ? 2 : _running ? 1 : 0),
                 uint8_t(_rate.fault()), {}, {}, _ap_calls, _rate.calls(), _motor_calls, _execution_us
                };
    for (m = 0; m < 4; m++) {
        motor_row.pwm[m] = hal.rcout->read(m);
        actuator = 0;
        motors->get_raw_motor_throttle(m, actuator);
        motor_row.actuator[m] = actuator;
    }
    copter.logger.WriteBlock_first_succeed(&motor_row, sizeof(motor_row));
    blocked = 0;
    for (a = 0; a < 3; a++) {
        blocked |= _result.axis[a].integration_blocked << a;
    }
    for (a = 0; a < 3; a++) {
        feedback = {LOG_PACKET_HEADER_INIT(LOG_UAFB_MSG), _time_us, _seq, c.command_seq, c.sampled_us,
                    a, uint8_t(_allocation.valid), uint8_t(c.pid_valid), c.limits, blocked,
                    c.gain, c.raw[a], c.command[a], _allocation.allocated[a], _allocation.residual[a]
                   };
        copter.logger.WriteBlock_first_succeed(&feedback, sizeof(feedback));
    }
    thrust = {LOG_PACKET_HEADER_INIT(LOG_UALT_MSG), _time_us, _seq, c.gain, {}, {},
              uint8_t(_allocation.valid), c.limits
             };
    for (m = 0; m < 4; m++) {
        thrust.thrust[m] = c.thrust[m]; thrust.post_thrust[m] = c.post_thrust[m];
    }
    copter.logger.WriteBlock_first_succeed(&thrust, sizeof(thrust));
    if (_angle_source != 0) {
        angle = {LOG_PACKET_HEADER_INIT(LOG_UAAT_MSG), _time_us, _seq,
                 uint8_t(_angle_source), uint8_t(_attitude.fault()),
                 _angle_reference.x, _angle_reference.y, attitude.euler.x, attitude.euler.y, attitude.euler.z,
                 attitude.error.x, attitude.error.y, _target.x, _target.y, _target.z, uint8_t(attitude.limited)
                };
        status = {LOG_PACKET_HEADER_INIT(LOG_UAST_MSG), _time_us, _seq,
                  _attitude.calls(), _ap_angle_calls, _height, _vertical_speed, motors->get_throttle_in()
                 };
        copter.logger.WriteBlock_first_succeed(&angle, sizeof(angle));
        copter.logger.WriteBlock_first_succeed(&status, sizeof(status));
    }
    pilot = { LOG_PACKET_HEADER_INIT(LOG_UARC_MSG), _time_us, _seq, _pilot.rc_ms,
              _pilot.normalized.x, _pilot.normalized.y, _pilot.normalized.z,
              _pilot.throttle, _throttle, uint8_t(_pilot.valid), uint8_t(_pilot.fault),
              uint8_t(_stop), uint8_t(mode_number()), uint32_t(AP_HAL::millis() - _pilot.rc_ms),
              attitude_control->get_throttle_mix(), _log_execution_us, _run_execution_us
            };
    copter.logger.WriteBlock_first_succeed(&pilot, sizeof(pilot));
#endif
    _log_execution_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
// End Data type, constant, macro definition
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
// End Static data declarations
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
// End Private function prototypes
#endif // AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
