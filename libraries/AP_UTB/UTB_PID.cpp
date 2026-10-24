/// @file UTB_PID.cpp
/// @brief AP-derived flight control; independent UTB computation, source: libraries/AC_PID/AC_PID.cpp.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
/*
   Derived from ArduPilot Copter 4.7.1, reference b25e828b86f945996489b26fb0357ce05f72afcb.
   Source files and hashes are recorded in docs/UTB_PHASE_3A_CS.md.
   Copyright remains with the original ArduPilot contributors.
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version. See COPYING.txt.
   This program is distributed WITHOUT ANY WARRANTY; without even the
   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "AP_UTB_config.h"
#if AP_UTB_MATH_AVAILABLE
/// @file	UTB_PID.cpp
/// @brief	General-purpose PID controller with input, error, and derivative filtering, plus slew rate limiting and EEPROM gain storage.

#include <AP_Math/AP_Math.h>
#include "UTB_PID.h"

#define AC_PID_DEFAULT_NOTCH_ATTENUATION 40

const AP_Param::GroupInfo UTB_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, UTB_PID, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 1, UTB_PID, _ki, default_ki),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 2, UTB_PID, _kd, default_kd),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 4, UTB_PID, _kff, default_kff),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 5, UTB_PID, _kimax, default_kimax),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the target input (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 9, UTB_PID, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the error (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 10, UTB_PID, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the derivative (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 11, UTB_PID, _filt_D_hz, default_filt_D_hz),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SMAX", 12, UTB_PID, _slew_rate_max, default_slew_rate_max),

    // @Param: PDMX
    // @DisplayName: PD sum maximum
    // @Description: The maximum/minimum value that the sum of the P and D term can output
    // @User: Advanced
    AP_GROUPINFO("PDMX", 13, UTB_PID, _kpdmax, 0),

    // @Param: D_FF
    // @DisplayName: PID Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_FF", 14, UTB_PID, _kdff, default_kdff),

#if AP_FILTER_ENABLED
    // @Param: NTF
    // @DisplayName: PID Target notch filter index
    // @Description: PID Target notch filter index
    // @Range: 0 8
    // @User: Advanced
    AP_GROUPINFO("NTF", 15, UTB_PID, _notch_T_filter, 0),

    // @Param: NEF
    // @DisplayName: PID Error notch filter index
    // @Description: PID Error notch filter index
    // @Range: 0 8
    // @User: Advanced
    AP_GROUPINFO("NEF", 16, UTB_PID, _notch_E_filter, 0),
#endif

    AP_GROUPEND
};

// Constructor
UTB_PID::UTB_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
                 float initial_srmax, float initial_srtau, float initial_dff) :
    default_kp(initial_p),
    default_ki(initial_i),
    default_kd(initial_d),
    default_kff(initial_ff),
    default_kdff(initial_dff),
    default_kimax(initial_imax),
    default_filt_T_hz(initial_filt_T_hz),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz),
    default_slew_rate_max(initial_srmax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // this param is not in the table, so its default is no loaded in the call above
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// Sets the target low-pass filter frequency (Hz)
void UTB_PID::set_filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// Sets the error low-pass filter frequency (Hz)
void UTB_PID::set_filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// Sets the derivative low-pass filter frequency (Hz)
void UTB_PID::set_filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void UTB_PID::set_slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

// Configures optional notch filters for target and error signals using the given sample rate.
// Filters are dynamically allocated and validated via the AP_Filter API.
void UTB_PID::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_notch_T_filter == 0 && _notch_E_filter == 0) {
        return;
    }

    if (_notch_T_filter != 0) {
        if (_target_notch == nullptr) {
            _target_notch = NEW_NOTHROW NotchFilterFloat();
        }
        // Lookup filter definition and initialize if valid
        AP_Filter* filter = AP::filters().get_filter(_notch_T_filter);
        if (filter != nullptr && !filter->setup_notch_filter(*_target_notch, sample_rate)) {
            delete _target_notch;
            _target_notch = nullptr;
            _notch_T_filter.set(0);  // disable filter if setup fails
        }
    }

    if (_notch_E_filter != 0) {
        if (_error_notch == nullptr) {
            _error_notch = NEW_NOTHROW NotchFilterFloat();
        }
        // Lookup filter definition and initialize if valid
        AP_Filter* filter = AP::filters().get_filter(_notch_E_filter);
        if (filter != nullptr && !filter->setup_notch_filter(*_error_notch, sample_rate)) {
            delete _error_notch;
            _error_notch = nullptr;
            _notch_E_filter.set(0);  // disable filter if setup fails
        }
    }
#endif
}

// Computes the PID output using a target and measurement input.
// Applies filters to the target and error, calculates the derivative and updates the integrator.
// If `limit` is true, the integrator is allowed to shrink but not grow.
float UTB_PID::update_all(float target, float measurement, float dt, bool limit, float pd_scale, float i_scale)
{
    // Return zero if input is invalid (NaN or infinite)
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // Flag used for logging to indicate a filter reset occurred
    _pid_info.reset = _flags._reset_filter;
    // Reset filters to match the current input (first sample or after reset)
    if (_flags._reset_filter) {
        // Reset filters to match the current inputs
        _flags._reset_filter = false;

        // Reset target filter
        _target = target;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            _target_notch->reset();
            _target = _target_notch->apply(_target);
        }
#endif

        // Calculate error and reset error filter
        _error = _target - measurement;
#if AP_FILTER_ENABLED
        if (_error_notch != nullptr) {
            _error_notch->reset();
            _error = _error_notch->apply(_error);
        }
#endif
        // Clear derivative history to avoid spikes after reset
        _derivative = 0.0f;
        _target_derivative = 0.0f;

    } else {

        // Apply target filters
        const float target_last = _target;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            // Allocate and set up target notch filter
            target = _target_notch->apply(target);
        }
#endif
        // Apply first-order low-pass filter to target value
        _target += get_filt_T_alpha(dt) * (target - _target);

        // Calculate error and apply error filter
        const float error_last = _error;
        float error = _target - measurement;
#if AP_FILTER_ENABLED
        if (_error_notch != nullptr) {
            // Allocate and set up error notch filter
            error = _error_notch->apply(error);
        }
#endif
        // apply notch filters before FTLD/FLTE to minimize shot noise
        _error += get_filt_E_alpha(dt) * (error - _error);

        if (is_positive(dt)) {
            // Compute and low-pass filter the error derivative (D term)
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
            // Calculate target derivative for D_FF contribution
            _target_derivative = (_target - target_last) / dt;
        }
    }

    // Integrate error (with wind-up protection if limit is active)
    // If limit is active, allow I-term to shrink but not grow
    update_i(dt, limit, i_scale);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);
    float I_out = _integrator;

    // Calculate dynamic modifier to reduce P+D output based on slew rate limiter
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    // This modifier is used to reduce control effort under fast transients
    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    // scale pd output if required
    P_out *= pd_scale;
    D_out *= pd_scale;

    _pid_info.PD_limit = false;
    // Apply PD sum limit if enabled
    if (is_positive(_kpdmax)) {
        const float PD_sum_abs = fabsf(P_out + D_out);
        if (PD_sum_abs > _kpdmax) {
            const float pd_limit_scale = _kpdmax / PD_sum_abs;
            P_out *= pd_limit_scale;
            D_out *= pd_limit_scale;
            _pid_info.PD_limit = true;
        }
    }

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;
    _pid_info.I = I_out;
    _pid_info.limit = limit;
    // Set I set flag for logging and clear
    _pid_info.I_term_set = _flags._I_set;
    _flags._I_set = false;
    _pid_info.FF = _target * _kff;
    _pid_info.DFF = _target_derivative * _kdff;

    return P_out + D_out + I_out;
}

// Computes the PID output from an error input only (target assumed to be zero).
// Applies error filtering and updates the derivative and integrator.
// Target and measurement must be set separately for logging.
// todo: remove function when it is no longer used.
float UTB_PID::update_error(float error, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    // Reuse update all code path, zero target and pass negative error as measurement
    // Pass negative error as "measurement" so that error = target - measurement evaluates correctly
    // Bypasses target filtering for legacy compatibility
    _target = 0.0;
    const float output = update_all(0.0, -error, dt, limit);

    // Make sure logged target and actual are still 0 to maintain behaviour
    _pid_info.target = 0.0;
    _pid_info.actual = 0.0;

    return output;
}

// Updates the integrator based on current error and dt.
// If `limit` is true, the integrator is only allowed to shrink to avoid wind-up.
// i_scale can be used to temporarily scale the updated I-term, by default is 1 - should not be set to 0
void UTB_PID::update_i(float dt, bool limit, float i_scale)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Allow integrator growth only if not limited, or if error opposes the integrator direction
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * i_scale * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
}

float UTB_PID::get_p() const
{
    return _pid_info.P;
}

float UTB_PID::get_i() const
{
    return _integrator;
}

float UTB_PID::get_d() const
{
    return _pid_info.D;
}

float UTB_PID::get_ff() const
{
    return  _pid_info.FF + _pid_info.DFF;
}

float UTB_PID::get_ff_component() const
{
    return  _pid_info.FF;
}

float UTB_PID::get_dff_component() const
{
    return  _pid_info.DFF;
}

// Used to fully zero the I term between mode changes or initialization
void UTB_PID::reset_I()
{
    _flags._I_set = true;
    _integrator = 0.0;
}

// Loads controller configuration from EEPROM, including gains and filter frequencies. (not used)
void UTB_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// Saves controller configuration from EEPROM, including gains and filter frequencies. Used by autotune to save gains before tuning.
void UTB_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

// Returns alpha value for the target low-pass filter (based on filter frequency and dt)
float UTB_PID::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// Returns alpha value for the error low-pass filter (based on filter frequency and dt)
float UTB_PID::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// Returns alpha value for the derivative low-pass filter (based on filter frequency and dt)
float UTB_PID::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

// Sets the integrator directly, clamped to the IMAX bounds. Also flags I-term as externally set.
void UTB_PID::set_integrator(float integrator)
{
    _flags._I_set = true;
    _integrator = constrain_float(integrator, -_kimax, _kimax);
}

// Gradually adjust the integrator toward a desired value using a time constant.
// Typically used to "relax" the I-term in dynamic conditions.
void UTB_PID::relax_integrator(float integrator, float dt, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        _flags._I_set = true;
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
}

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  The UTB_PID_SlewLimiter filter provides an actuator slew rate limiter for
  PID controllers. It is used to reduce the P and D gains when the
  filter detects that the P+D components are pushing the actuator
  beyond the configured actuator rate limit. This can prevent
  oscillations that are caused by the output actuation rate going
  beyond the actuator maximum physical rate, which causes the
  actuation demand and achieved rate to get out of phase.

  this filter was originally written by Paul Riseborough for fixed
  wing use. It was adapted for wider use in AC_PID by Andrew Tridgell
 */
#include "UTB_PID.h"

#define WINDOW_MS 300                   // time in msec required for a half cycle of the slowest oscillation frequency expected
#define MODIFIER_GAIN 1.5f              // ratio of modifier reduction to slew rate exceedance ratio
#define DERIVATIVE_CUTOFF_FREQ 25.0f

UTB_PID_SlewLimiter::UTB_PID_SlewLimiter(const float &_slew_rate_max, const float &_slew_rate_tau) :
    slew_rate_max(_slew_rate_max),
    slew_rate_tau(_slew_rate_tau)
{
    slew_filter.set_cutoff_frequency(DERIVATIVE_CUTOFF_FREQ);
    slew_filter.reset(0.0);
}

/*
  apply filter to sample, returning multiplier between 0 and 1 to keep
  output within slew rate
 */
float UTB_PID_SlewLimiter::modifier(float sample, float dt)
{
    if (!is_positive(dt)) {
        return 1.0;
    }

    // Calculate a low pass filtered slew rate
    const float slew_rate = slew_filter.apply((sample - last_sample) / dt, dt);
    last_sample = sample;

    uint32_t now_ms = AP_HAL::millis();

    // Apply a filter to decay maximum seen slew rate once the value had left the window period
    const float decay_alpha = fminf(dt, slew_rate_tau) / slew_rate_tau;

    // Apply a filter to increases in slew rate only to reduce the effect of gusts and large controller setpoint changes
    const float attack_alpha = fminf(2.0f * decay_alpha, 1.0f);

    // Decay the peak positive and negative slew rate if they are outside the window
    // Never drop PID gains below 10% of configured value
    if (slew_rate > _max_pos_slew_rate) {
        _max_pos_slew_rate = slew_rate;
        _max_pos_slew_event_ms = now_ms;
    } else if (now_ms - _max_pos_slew_event_ms > WINDOW_MS) {
        _max_pos_slew_rate *= (1.0f - decay_alpha);
    }

    if (-slew_rate > _max_neg_slew_rate) {
        _max_neg_slew_rate = -slew_rate;
        _max_neg_slew_event_ms = now_ms;
    } else if (now_ms - _max_neg_slew_event_ms > WINDOW_MS) {
        _max_neg_slew_rate *= (1.0f - decay_alpha);
    }

    const float raw_slew_rate = 0.5f*(_max_pos_slew_rate + _max_neg_slew_rate);
    _output_slew_rate = (1.0f - attack_alpha) * _output_slew_rate + attack_alpha * raw_slew_rate;
    _output_slew_rate = fminf(_output_slew_rate, raw_slew_rate);

    if (slew_rate_max <= 0) {
        return 1.0;
    }

    // Constrain slew rate used for calculation
    const float limited_raw_slew_rate = 0.5f*(fminf(_max_pos_slew_rate, 10.0f * slew_rate_max) + fminf(_max_neg_slew_rate, 10.0f * slew_rate_max));

    // Store a series of positive slew rate exceedance events
    if (!_pos_event_stored && slew_rate > slew_rate_max) {
        if (_pos_event_index >= SLEWLIMITER_N_EVENTS) {
            _pos_event_index = 0;
        }
        _pos_event_ms[_pos_event_index] = now_ms;
        _pos_event_index++;
        _pos_event_stored = true;
        _neg_event_stored = false;
    }

    // Store a series of negative slew rate exceedance events
    if (!_neg_event_stored && -slew_rate > slew_rate_max) {
        if (_neg_event_index >= SLEWLIMITER_N_EVENTS) {
            _neg_event_index = 0;
        }
        _neg_event_ms[_neg_event_index] = now_ms;
        _neg_event_index++;
        _neg_event_stored = true;
        _pos_event_stored = false;
    }

    // Find the oldest event time
    uint32_t oldest_ms = now_ms;
    for (uint8_t index = 0; index < SLEWLIMITER_N_EVENTS; index++) {
        oldest_ms = MIN(oldest_ms, _pos_event_ms[index]);
        oldest_ms = MIN(oldest_ms, _neg_event_ms[index]);
    }

    // Apply a further reduction when the oldest exceedance event falls outside the window required for the
    // specified number of exceedance events. This prevents spikes due to control mode changed, etc causing
    // unwanted gain reduction and is only applied to the slew rate used for gain reduction
    float modifier_input = limited_raw_slew_rate;
    if (now_ms - oldest_ms > (SLEWLIMITER_N_EVENTS + 1) * WINDOW_MS) {
        const float oldest_time_from_window = 0.001f*(float)(now_ms - oldest_ms - (SLEWLIMITER_N_EVENTS + 1) * WINDOW_MS);
        modifier_input *= expf(-oldest_time_from_window / slew_rate_tau);
    }

    _modifier_slew_rate = (1.0f - attack_alpha) * _modifier_slew_rate + attack_alpha * modifier_input;
    _modifier_slew_rate = fminf(_modifier_slew_rate, modifier_input);

    // Calculate the gain adjustment
    float mod = 1.0f;
    if (_modifier_slew_rate > slew_rate_max) {
        mod = slew_rate_max / (slew_rate_max + MODIFIER_GAIN * (_modifier_slew_rate - slew_rate_max));
    }

    return mod;
}


#if AP_UTB_NATIVE_SHADOW_ENABLED
AC_PID_Shadow UTB_PID::shadow_snapshot() const
{
    AC_PID_Shadow state;
    const float config[] = {_kp.get(), _ki.get(), _kd.get(), _kff.get(), _kimax.get(), _kpdmax.get(),
                            _filt_T_hz.get(), _filt_E_hz.get(), _filt_D_hz.get(), _slew_rate_max.get(),
                            _kdff.get(), _slew_rate_tau.get()};
    for (uint8_t i = 0; i < AC_PID_Shadow::CONFIG_COUNT; i++) {
        state.config[i] = config[i];
    }
    state.integrator = _integrator;
    state.target = _target;
    state.error = _error;
    state.derivative = _derivative;
    state.target_derivative = _target_derivative;
    state.reset_filter = _flags._reset_filter;
    state.i_set = _flags._I_set;
    state.slew_scale = _slew_limit_scale;
    state.info = _pid_info;
#if AP_FILTER_ENABLED
    state.notch = _notch_T_filter.get() != 0 || _notch_E_filter.get() != 0 ||
                  _target_notch != nullptr || _error_notch != nullptr;
#endif
    return state;
}
void UTB_PID::shadow_restore(const AC_PID_Shadow &state)
{
    // Assignment of plain values only; no gain loading or notch setup.
    _kp.set(state.config[AC_PID_Shadow::P]);
    _ki.set(state.config[AC_PID_Shadow::I]);
    _kd.set(state.config[AC_PID_Shadow::D]);
    _kff.set(state.config[AC_PID_Shadow::FF]);
    _kimax.set(state.config[AC_PID_Shadow::IMAX]);
    _kpdmax.set(state.config[AC_PID_Shadow::PDMAX]);
    _filt_T_hz.set(state.config[AC_PID_Shadow::FLTT]);
    _filt_E_hz.set(state.config[AC_PID_Shadow::FLTE]);
    _filt_D_hz.set(state.config[AC_PID_Shadow::FLTD]);
    _slew_rate_max.set(state.config[AC_PID_Shadow::SMAX]);
    _kdff.set(state.config[AC_PID_Shadow::DFF]);
    _slew_rate_tau.set(state.config[AC_PID_Shadow::SRT]);
    _slew_limit_scale = state.slew_scale;
    _integrator = state.integrator;
    _target = state.target;
    _error = state.error;
    _derivative = state.derivative;
    _target_derivative = state.target_derivative;
    _flags._reset_filter = state.reset_filter;
    _flags._I_set = state.i_set;
    _pid_info = state.info;
}
#endif

#endif // AP_UTB_RATE_SITL_AVAILABLE
