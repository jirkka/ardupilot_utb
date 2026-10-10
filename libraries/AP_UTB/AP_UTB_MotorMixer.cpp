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
#include "AP_UTB_MotorMixer.h"
#if AP_UTB_ENABLED
AP_UTB_MotorMixer::Result AP_UTB_MotorMixer::mix(
    const AP_UTB_RateController::Result &control, float thrust, uint8_t frame_class, uint8_t frame_type) const
{
    Result result;
    const Vector3f &c = control.command;
    if (!supports_frame(frame_class, frame_type) || !control.valid || !isfinite(thrust) || thrust < 0 || thrust > 1 ||
        c.is_nan() || c.is_inf() || fabsf(c.x) > 1 || fabsf(c.y) > 1 || fabsf(c.z) > 1) {
        return result;
    }
    const float moment[4] = {(-c.x - c.y - c.z) * 0.5f, (-c.x + c.y + c.z) * 0.5f,
                             (c.x - c.y + c.z) * 0.5f, (c.x + c.y - c.z) * 0.5f
                            };
    float low = moment[0];
    float high = moment[0];
    for (uint8_t i = 1; i < 4; i++) {
        low = MIN(low, moment[i]);
        high = MAX(high, moment[i]);
    }
    const float span = high - low;
    result.scale = span > 1 ? 1 / span : 1;
    const float collective = constrain_float(thrust, -result.scale * low, 1 - result.scale * high);
    result.collective_shift = collective - thrust;
    for (uint8_t i = 0; i < 4; i++) {
        const float value = collective + result.scale * moment[i];
        if (!isfinite(value) || value < -1.0e-5f || value > 1 + 1.0e-5f) {
            return {};
        }
        result.thrust[i] = constrain_float(value, 0, 1);
        if (result.thrust[i] <= 1.0e-5f) {
            result.lower_mask |= 1U << i;
        }
        if (result.thrust[i] >= 1 - 1.0e-5f) {
            result.upper_mask |= 1U << i;
        }
    }
    const float *m = result.thrust;
    result.achieved_thrust = (m[0] + m[1] + m[2] + m[3]) * 0.25f;
    result.achieved = Vector3f((-m[0] - m[1] + m[2] + m[3]) * 0.5f,
                               (-m[0] + m[1] - m[2] + m[3]) * 0.5f,
                               (-m[0] + m[1] + m[2] - m[3]) * 0.5f);
    for (uint8_t i = 0; i < 4; i++) {
        const float residual = i == 3 ? thrust - result.achieved_thrust : c[i] - result.achieved[i];
        if (residual > 1.0e-5f) {
            result.positive_limits |= 1U << i;
        }
        if (residual < -1.0e-5f) {
            result.negative_limits |= 1U << i;
        }
    }
    result.moments_scaled = result.scale < 1 - 1.0e-5f;
    result.collective_shifted = fabsf(result.collective_shift) > 1.0e-5f;
    result.valid = true;
    return result;
}
#endif // AP_UTB_ENABLED
