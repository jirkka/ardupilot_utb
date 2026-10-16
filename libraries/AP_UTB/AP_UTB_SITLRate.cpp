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
#include "AP_UTB_SITLRate.h"
#if AP_UTB_RATE_SITL_AVAILABLE
bool AP_UTB_SITLRate::configure(const AP_UTB_RateController::Gains (&gains)[3], bool allocation_feedback, bool delayed)
{
    reset();
    _allocation_feedback = allocation_feedback;
    _delayed = delayed;
    for (uint8_t i = 0; i < 3; i++) {
        _gains[i] = gains[i];
    }
    _configured = _controller.configure(gains, 0.0025f);
    if (!_configured) {
        invalidate(Fault::CONFIG);
    }
    return _configured;
}

bool AP_UTB_SITLRate::configuration_matches(const AP_UTB_RateController::Gains (&gains)[3]) const
{
    for (uint8_t i = 0; i < 3; i++) {
        if (!(gains[i] == _gains[i])) {
            return false;
        }
    }
    return true;
}

void AP_UTB_SITLRate::reset()
{
    _controller.reset();
    _fault = Fault::NONE;
    _calls = 0;
    _last_imu = 0;
    _gyro = 255;
    _context = false;
    _configured = false;
    _previous_command.zero();
    _feedback = {};
    _allocation_feedback = _delayed = false;
}

void AP_UTB_SITLRate::invalidate(Fault reason)
{
    if (_fault == Fault::NONE) {
        _fault = reason;
    }
    _controller.reset();
}

Vector3f AP_UTB_SITLRate::transport(const Vector3f &command, bool delayed, bool enabled)
{
    if (!enabled || _fault != Fault::NONE || command.is_nan() || command.is_inf()) {
        _previous_command.zero();
        return {};
    }
    const Vector3f output = delayed ? _previous_command : command;
    _previous_command = command;
    return output;
}

AP_UTB_RateController::Result AP_UTB_SITLRate::step(const Inputs &in, bool utb_owner)
{
    if (_fault != Fault::NONE) {
        return {};
    }
    if (!_configured) {
        invalidate(Fault::CONFIG);
    } else if (in.frame_class != 1 || in.frame_type != 18) {
        invalidate(Fault::FRAME);
    } else if (in.fast_rate) {
        invalidate(Fault::FAST_RATE);
    } else if (!isfinite(in.nominal_dt) || fabsf(in.nominal_dt - 0.0025f) > 1.0e-7f ||
               !isfinite(in.dt) || in.dt < 0.000625f || in.dt > 0.01f) {
        invalidate(Fault::DT);
    } else if (!in.state.valid(in.now_us, 10000) || in.target.is_nan() || in.target.is_inf()) {
        invalidate(Fault::STATE);
    } else if (_context && (in.state.imu_updated_us == _last_imu || in.state.primary_gyro != _gyro)) {
        invalidate(Fault::IMU);
    } else if (in.target.length() > 0.6f || in.state.rates_rads.length() > 2.0f) {
        invalidate(Fault::ENVELOPE);
    } else if (!_allocation_feedback && _context && _armed && _full_spool && in.armed && in.full_spool && in.allocation_limited) {
        invalidate(Fault::SATURATION);
    }
    if (_fault != Fault::NONE) {
        return {};
    }
    const bool changed = !_context || in.armed != _armed || in.full_spool != _full_spool;
    if (changed) {
        _controller.reset();
    } else if (_allocation_feedback && utb_owner && in.armed && in.full_spool) {
        const auto &c = _feedback.capture;
        if (!_feedback.valid || c.output_seq != in.seq - 1 || uint32_t(in.now_us - c.sampled_us) > 10000 ||
            (c.pid_valid && c.command_seq != c.output_seq - uint32_t(_delayed))) {
            invalidate(Fault::FEEDBACK);
            return {};
        }
        if (c.pid_valid) {
            AP_UTB_RateController::Result previous;
            previous.valid = true;
            for (uint8_t a = 0; a < 3; a++) {
                previous.axis[a].raw = c.raw[a];
            }
            _controller.feedback(previous, _feedback.allocated);
        }
    }
    _context = true;
    _armed = in.armed;
    _full_spool = in.full_spool;
    _last_imu = in.state.imu_updated_us;
    _gyro = in.state.primary_gyro;
    if (!utb_owner) {
        // AP baseline validates the same input, without running a second PID.
        AP_UTB_RateController::Result ready;
        ready.valid = true;
        return ready;
    }
    ++_calls;
    auto result = _controller.update(in.state, in.target, in.dt);
    if (!result.valid && !result.priming) {
        invalidate(Fault::STATE);
    }
    for (const auto &axis : result.axis) {
        if (!_allocation_feedback && result.valid && fabsf(axis.raw) >= 1.0f) {
            invalidate(Fault::SATURATION);
            return {};
        }
    }
    return result;
}

Vector3f AP_UTB_SITLRate::reference(uint8_t profile, uint32_t cycle, float amplitude)
{
    Vector3f ref;
    if (cycle >= duration_cycles(profile) || !isfinite(amplitude) || amplitude < 0 || amplitude > 0.3f) {
        return ref;
    }
    const float value = cycle < 400 ? 0 : cycle < 1200 ? amplitude :
                        cycle < 1600 ? 0 : cycle < 2400 ? -amplitude : 0;
    if (profile >= 1 && profile <= 3) {
        ref[profile - 1] = value;
    } else if (profile == 4) {
        ref = Vector3f(value, -value, value);
    } else if (profile == 5 && cycle >= 400 && cycle < 2400) {
        ref.x = amplitude * sinf(2 * M_PI * 0.5f * (cycle - 400) * 0.0025f);
    } else if (profile == 6 && cycle >= 400 && cycle < 2400) {
        ref.x = ((cycle - 400) / 400) % 2 == 0 ? amplitude : -amplitude;
    } else if (profile == 7) {
        ref.z = cycle < 400 ? 0 : cycle < 3600 ? amplitude : cycle < 4000 ? 0 : cycle < 7200 ? -amplitude : 0;
    }
    return ref;
}
#endif // AP_UTB_RATE_SITL_AVAILABLE
