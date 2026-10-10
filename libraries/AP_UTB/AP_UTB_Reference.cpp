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
#include "AP_UTB_Reference.h"
#if AP_UTB_ENABLED
bool AP_UTB_Reference::rates(const Vector3f &sticks, float rp_rate_deg, float y_rate_deg,
                             float rp_expo, float y_expo, Vector3f &target)
{
    target.zero();
    if (sticks.is_nan() || sticks.is_inf() || fabsf(sticks.x) > 1 || fabsf(sticks.y) > 1 || fabsf(sticks.z) > 1 ||
        !isfinite(rp_rate_deg) || rp_rate_deg < 1 || rp_rate_deg > 1080 ||
        !isfinite(y_rate_deg) || y_rate_deg < 1 || y_rate_deg > 360 ||
        !isfinite(rp_expo) || rp_expo < -0.5f || rp_expo > 0.95f ||
        !isfinite(y_expo) || y_expo < -1 || y_expo > 0.95f) {
        return false;
    }
    const float length = MAX(1.0f, norm(sticks.x, sticks.y));
    target = Vector3f(radians(rp_rate_deg) * input_expo(sticks.x / length, rp_expo),
                      radians(rp_rate_deg) * input_expo(sticks.y / length, rp_expo),
                      radians(y_rate_deg) * input_expo(sticks.z, y_expo));
    return !target.is_nan() && !target.is_inf();
}
#endif // AP_UTB_ENABLED
