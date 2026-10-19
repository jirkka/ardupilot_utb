/// @file UTB_Core.cpp
/// @author Jiri Kovar; AP algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL version 3 or later. Derived from Copter 4.7.1, b25e828b; see COPYING.txt.
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "UTB_Core.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
bool UTB_Core::enter(uint8_t mode, AC_AttitudeControl &native)
{
    if (!state.finite()) {
        return false;
    }
    if (_mode == 0) {
        rate.reset_target_and_rate(true);
        rate.reset_rate_controller_I_terms();
        rate.set_throttle_mix_value(native.get_throttle_mix());
        // Separate parameter banks can have different IMAX; native setters clamp transferred I.
        rate.get_rate_roll_pid().set_integrator(isfinite(native.get_rate_roll_pid().get_i()) ? native.get_rate_roll_pid().get_i() : 0);
        rate.get_rate_pitch_pid().set_integrator(isfinite(native.get_rate_pitch_pid().get_i()) ? native.get_rate_pitch_pid().get_i() : 0);
        rate.get_rate_yaw_pid().set_integrator(isfinite(native.get_rate_yaw_pid().get_i()) ? native.get_rate_yaw_pid().get_i() : 0);
        rate.get_rate_roll_pid().reset_filter();
        rate.get_rate_pitch_pid().reset_filter();
        rate.get_rate_yaw_pid().reset_filter();
    }
    _seq = _motor_calls = 0;
    _mode = mode;
    _healthy = true;
    return true;
}
void UTB_Core::leave(uint8_t mode)
{
    // A destination UTB mode enters before the source exits in the original mode manager.
    if (_mode == mode) {
        _mode = 0;
    }
}
bool UTB_Core::update_rate(float dt)
{
    ++_seq;
    _time_us = AP_HAL::micros64();
    _dt = dt;
    _healthy = state.finite() && isfinite(dt) && is_positive(dt);
    if (!isfinite(dt) || !is_positive(dt)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return false;
    }
    // Native PID input guards are evaluated per axis. Estimator/INS health policy stays in AP.

    rate.set_dt_s(dt);
    rate.rate_controller_run_dt(state.get_gyro_latest(), dt);
    rate.rate_controller_target_reset();
    _healthy = _healthy && rate.output_valid();
    return _healthy;
}
void UTB_Core::prepare_native(AC_AttitudeControl &native)
{
    // No native PID is executed here. This seeds a later AP mode's normal initialization.
    native.set_throttle_input_for_handoff(isfinite(rate.get_throttle_in()) ? rate.get_throttle_in() : _motors.get_throttle_in());
    const float mix = rate.get_throttle_mix();
    if (isfinite(mix)) {
        native.set_throttle_mix_value(mix);
    }
    native.reset_target_and_rate(true);
    native.get_rate_roll_pid().set_integrator(isfinite(rate.get_rate_roll_pid().get_i()) ? rate.get_rate_roll_pid().get_i() : 0);
    native.get_rate_pitch_pid().set_integrator(isfinite(rate.get_rate_pitch_pid().get_i()) ? rate.get_rate_pitch_pid().get_i() : 0);
    native.get_rate_yaw_pid().set_integrator(isfinite(rate.get_rate_yaw_pid().get_i()) ? rate.get_rate_yaw_pid().get_i() : 0);
    native.get_rate_roll_pid().reset_filter();
    native.get_rate_pitch_pid().reset_filter();
    native.get_rate_yaw_pid().reset_filter();
}

// Original update_throttle_mix manual-mode policy; called by the original 50 Hz task.
void UTB_Core::update_mix(bool armed, bool landed, bool throttle_zero, bool air_mode)
{
    if (!armed || landed || (throttle_zero && !air_mode)) {
        rate.set_throttle_mix_min();
    } else {
        rate.set_throttle_mix_man();
    }
}
#endif // AP_UTB_RATE_SITL_AVAILABLE
