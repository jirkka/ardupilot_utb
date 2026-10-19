#pragma once
/// @file UTB_PID.h
/// @brief AP-derived flight control; independent UTB computation, source: libraries/AC_PID/AC_PID.h.
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

/// @file	UTB_PID.h
/// @brief	General-purpose PID controller with input, error, and derivative filtering, plus slew rate limiting and EEPROM gain storage.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>


/* slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum
 * slew rate
*/

#include <stdint.h>
#include <Filter/LowPassFilter.h>

#define SLEWLIMITER_N_EVENTS 2  // number of positive and negative consecutive slew rate exceedance events recorded where a value of 2 corresponds to a complete cycle

class UTB_PID_SlewLimiter
{
public:
    UTB_PID_SlewLimiter(const float &slew_rate_max, const float &slew_rate_tau);

    CLASS_NO_COPY(UTB_PID_SlewLimiter);

    /*
      apply filter to sample, returning multiplier between 0 and 1 to keep
      output within slew rate
    */
    float modifier(float sample, float dt);

    /*
      get last oscillation slew rate
     */
    float get_slew_rate(void) const
    {
        return _output_slew_rate;
    }

private:
    const float &slew_rate_max;
    const float &slew_rate_tau;
    LowPassFilterFloat slew_filter;
    float _output_slew_rate;
    float _modifier_slew_rate;
    float last_sample;
    float _max_pos_slew_rate;
    float _max_neg_slew_rate;
    uint32_t _max_pos_slew_event_ms;
    uint32_t _max_neg_slew_event_ms;
    uint8_t _pos_event_index;
    uint8_t _neg_event_index;
    uint32_t _pos_event_ms[SLEWLIMITER_N_EVENTS];
    uint32_t _neg_event_ms[SLEWLIMITER_N_EVENTS];
    bool _pos_event_stored;
    bool _neg_event_stored;
};

#include <Filter/NotchFilter.h>
#include <Filter/AP_Filter.h>

#define AC_PID_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_DFILT_HZ_DEFAULT  20.0f  // default input filter frequency
#define AC_PID_RESET_TC          0.16f  // Time constant for integrator reset decay to zero

#include <AC_PID/AP_PIDInfo.h>

/// @class	UTB_PID
/// @brief	Copter PID control class
class UTB_PID
{
public:

    struct Defaults {
        float p;
        float i;
        float d;
        float ff;
        float imax;
        float filt_T_hz;
        float filt_E_hz;
        float filt_D_hz;
        float srmax;
        float srtau;
        float dff;
    };

    /// Constructor for PID controller with EEPROM-backed gain.
    /// Parameters are initialized from defaults or EEPROM at runtime.
    UTB_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
            float initial_srmax=0, float initial_srtau=1.0, float initial_dff=0);
    UTB_PID(const UTB_PID::Defaults &defaults) :
        UTB_PID(
            defaults.p,
            defaults.i,
            defaults.d,
            defaults.ff,
            defaults.imax,
            defaults.filt_T_hz,
            defaults.filt_E_hz,
            defaults.filt_D_hz,
            defaults.srmax,
            defaults.srtau,
            defaults.dff
        )
    { }

    CLASS_NO_COPY(UTB_PID);

    // Computes the PID output using a target and measurement input.
    // Applies filters to the target and error, calculates the derivative and updates the integrator.
    // If `limit` is true, the integrator is allowed to shrink but not grow.
    float update_all(float target, float measurement, float dt, bool limit = false, float pd_scale = 1.0f, float i_scale = 1.0f);

    // Computes the PID output from an error input only (target assumed to be zero).
    // Applies error filtering and updates the derivative and integrator.
    // Target and measurement must be set separately for logging.
    // todo: remove function when it is no longer used.
    float update_error(float error, float dt, bool limit = false);

    // get_pid - get results from pid controller
    float get_p() const;
    float get_i() const;
    float get_d() const;
    float get_ff() const;
    float get_ff_component() const;
    float get_dff_component() const;

    // Used to fully zero the I term between mode changes or initialization
    void reset_I();

    // Flags the input filter for reset. The next call to `update_all()` will reinitialize the filter using the next input.
    void reset_filter()
    {
        _flags._reset_filter = true;
    }

    // Loads controller configuration from EEPROM, including gains and filter frequencies. (not used)
    void load_gains();

    // Saves controller configuration from EEPROM, including gains and filter frequencies. Used by autotune to save gains before tuning.
    void save_gains();

    // get accessors
    const AP_Float &kP() const
    {
        return _kp;
    }
    AP_Float &kP()
    {
        return _kp;
    }
    AP_Float &kI()
    {
        return _ki;
    }
    AP_Float &kD()
    {
        return _kd;
    }
    AP_Float &kIMAX()
    {
        return _kimax;
    }
    AP_Float &kPDMAX()
    {
        return _kpdmax;
    }
    AP_Float &ff()
    {
        return _kff;
    }
    AP_Float &filt_T_hz()
    {
        return _filt_T_hz;
    }
    AP_Float &filt_E_hz()
    {
        return _filt_E_hz;
    }
    AP_Float &filt_D_hz()
    {
        return _filt_D_hz;
    }
    AP_Float &slew_limit()
    {
        return _slew_rate_max;
    }
    AP_Float &kDff()
    {
        return _kdff;
    }

    float imax() const
    {
        return _kimax.get();
    }
    float pdmax() const
    {
        return _kpdmax.get();
    }

    // Returns alpha value for the target low-pass filter (based on filter frequency and dt)
    float get_filt_T_alpha(float dt) const;
    // Returns alpha value for the error low-pass filter (based on filter frequency and dt)
    float get_filt_E_alpha(float dt) const;
    // Returns alpha value for the derivative low-pass filter (based on filter frequency and dt)
    float get_filt_D_alpha(float dt) const;

    // set accessors
    void set_kP(const float v)
    {
        _kp.set(v);
    }
    void set_kI(const float v)
    {
        _ki.set(v);
    }
    void set_kD(const float v)
    {
        _kd.set(v);
    }
    void set_ff(const float v)
    {
        _kff.set(v);
    }
    void set_imax(const float v)
    {
        _kimax.set(fabsf(v));
    }
    void set_pdmax(const float v)
    {
        _kpdmax.set(fabsf(v));
    }
    void set_filt_T_hz(const float v);
    void set_filt_E_hz(const float v);
    void set_filt_D_hz(const float v);
    void set_slew_limit(const float v);
    void set_kDff(const float v)
    {
        _kdff.set(v);
    }

    // Sets target and actual rate values for external logging (optional).
    void set_target_rate(float target)
    {
        _pid_info.target = target;
    }
    void set_actual_rate(float actual)
    {
        _pid_info.actual = actual;
    }

    // Sets the integrator directly, clamped to the IMAX bounds. Also flags I-term as externally set.
    void set_integrator(float i);

    // Gradually adjust the integrator toward a desired value using a time constant.
    // Typically used to "relax" the I-term in dynamic conditions.
    void relax_integrator(float integrator, float dt, float time_constant);

    // set slew limiter scale factor
    void set_slew_limit_scale(int8_t scale)
    {
        _slew_limit_scale = scale;
    }

    // Returns current slew rate from the limiter. Returns 0 if SMAX is zero (disabled).
    float get_slew_rate(void) const
    {
        return _slew_limiter.get_slew_rate();
    }

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    // Configures optional notch filters for target and error signals using the given sample rate.
    // Filters are dynamically allocated and validated via the AP_Filter API.
    void set_notch_sample_rate(float);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Updates the integrator based on current error and dt.
    // If `limit` is true, the integrator is only allowed to shrink to avoid wind-up.
    void update_i(float dt, bool limit, float i_scale = 1.0f);

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _kff;
    AP_Float _kimax;
    AP_Float _kpdmax;
    AP_Float _filt_T_hz;         // PID target filter frequency in Hz
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz
    AP_Float _slew_rate_max;
    AP_Float _kdff;
#if AP_FILTER_ENABLED
    AP_Int8 _notch_T_filter;
    AP_Int8 _notch_E_filter;
#endif

    // Slew rate time constant (tau). Not exposed in this class by default, but defined as an AP_Float so parent classes can make it configurable via param table.
    AP_Float _slew_rate_tau;

    UTB_PID_SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};

    // flags
    struct ac_pid_flags {
        bool _reset_filter :1;  // true if the input filter should be reset on the next call to update_all()
        bool _I_set :1;         // true if the I term has been set externally, including zeroing
    } _flags;

    // internal variables
    float _integrator;        // integrator value
    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _derivative;        // derivative value to enable filtering
    int8_t _slew_limit_scale;
    float _target_derivative; // target derivative value to enable dff
#if AP_FILTER_ENABLED
    NotchFilterFloat* _target_notch;
    NotchFilterFloat* _error_notch;
#endif

    AP_PIDInfo _pid_info;

    const float default_kp;
    const float default_ki;
    const float default_kd;
    const float default_kff;
    const float default_kdff;
    const float default_kimax;
    const float default_filt_T_hz;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
    const float default_slew_rate_max;
};

#endif // AP_UTB_RATE_SITL_AVAILABLE
