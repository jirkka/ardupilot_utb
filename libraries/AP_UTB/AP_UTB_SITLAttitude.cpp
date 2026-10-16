/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#include "AP_UTB_SITLAttitude.h"
#if AP_UTB_RATE_SITL_AVAILABLE
bool AP_UTB_SITLAttitude::Config::operator==(const Config &other) const
{
    return roll_p <= other.roll_p && roll_p >= other.roll_p &&
           pitch_p <= other.pitch_p && pitch_p >= other.pitch_p &&
           rate_max <= other.rate_max && rate_max >= other.rate_max;
}

void AP_UTB_SITLAttitude::reset()
{
    _configured = _context = false;
    _fault = Fault::NONE;
    _last_imu = _calls = 0;
    _gyro = 255;
}

void AP_UTB_SITLAttitude::invalidate(Fault fault)
{
    if (_fault == Fault::NONE) {
        _fault = fault;
    }
}

bool AP_UTB_SITLAttitude::configure(const Config &config)
{
    reset();
    if (!isfinite(config.roll_p) || !isfinite(config.pitch_p) || !isfinite(config.rate_max) ||
        config.roll_p < 1 || config.roll_p > 6 || config.pitch_p < 1 || config.pitch_p > 6 ||
        config.rate_max < 0.05f || config.rate_max > 0.3f) {
        invalidate(Fault::CONFIG);
        return false;
    }
    _config = config;
    _configured = true;
    return true;
}

AP_UTB_SITLAttitude::Result AP_UTB_SITLAttitude::update(const AP_UTB_State &state, const Vector3f &ref,
        float dt, uint32_t now_us)
{
    Result out;
    if (_fault != Fault::NONE) {
        return out;
    }
    if (!_configured) {
        invalidate(Fault::CONFIG);
    } else if (!isfinite(dt) || dt < 0.000625f || dt > 0.01f) {
        invalidate(Fault::DT);
    } else if (!state.valid(now_us, 10000)) {
        invalidate(Fault::STATE);
    } else if (_context && (state.imu_updated_us == _last_imu || state.primary_gyro != _gyro)) {
        invalidate(Fault::IMU);
    } else if (ref.is_nan() || ref.is_inf() || Vector2f(ref.x, ref.y).length() > radians(5.0f) + 1.0e-7f ||
               fabsf(ref.z) > 0.1f) {
        invalidate(Fault::REFERENCE);
    }
    if (_fault != Fault::NONE) {
        return out;
    }
    state.attitude_body_to_ned.to_euler(out.euler);
    if (MAX(fabsf(out.euler.x), fabsf(out.euler.y)) > radians(35.0f)) {
        invalidate(Fault::ENVELOPE);
        return {};
    }
    Quaternion desired;
    desired.from_euler(ref.x, ref.y, out.euler.z);
    // Target body-down direction expressed in the current body frame.
    const Vector3f direction = state.attitude_body_to_ned.inverse() * (desired * Vector3f(0, 0, 1));
    const float sine = Vector2f(direction.x, direction.y).length();
    const float angle = atan2f(sine, direction.z);
    const float scale = sine > 1.0e-7f ? angle / sine : 1.0f;
    out.error = Vector3f(-direction.y, direction.x, 0) * scale;
    out.rates = {out.error.x * _config.roll_p, out.error.y * _config.pitch_p, ref.z};
    const float rp = Vector2f(out.rates.x, out.rates.y).length();
    if (rp > _config.rate_max) {
        out.rates.x *= _config.rate_max / rp;
        out.rates.y *= _config.rate_max / rp;
        out.limited = true;
    }
    _last_imu = state.imu_updated_us;
    _gyro = state.primary_gyro;
    _context = true;
    ++_calls;
    out.valid = true;
    return out;
}

Vector3f AP_UTB_SITLAttitude::reference(uint8_t profile, uint32_t cycle, float angle_rad, float yaw_rate)
{
    if (cycle >= DURATION_CYCLES) {
        return {};
    }
    Vector3f ref(0, 0, yaw_rate);
    const float value = cycle < 400 ? 0 : cycle < 1600 ? angle_rad : cycle < 2800 ? 0 :
                        cycle < 4000 ? -angle_rad : 0;
    if (profile == 1 || profile == 2) {
        ref[profile - 1] = value;
    } else if (profile == 4) {
        ref.x = value; ref.y = -value;
    } else if (profile == 5 && cycle >= 400 && cycle < 4400) {
        ref.x = angle_rad * sinf(2 * M_PI * 0.2f * (cycle - 400) * 0.0025f);
    } else if (profile == 6 && cycle >= 400 && cycle < 4000) {
        ref.x = ((cycle - 400) / 1200) % 2 == 0 ? angle_rad : -angle_rad;
    }
    return ref;
}
#endif // AP_UTB_RATE_SITL_AVAILABLE
