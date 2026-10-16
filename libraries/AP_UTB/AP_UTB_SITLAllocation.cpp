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
#include "AP_UTB_SITLAllocation.h"
#if AP_UTB_RATE_SITL_AVAILABLE
AP_UTB_SITLAllocation::Result AP_UTB_SITLAllocation::reconstruct(const Capture &c)
{
    Result result;
    result.capture = c;
    if (!c.factors_available || c.boost || !c.armed || !c.full_spool || !isfinite(c.gain) || c.gain <= 0 ||
        c.raw.is_nan() || c.raw.is_inf() || c.command.is_nan() || c.command.is_inf()) {
        return result;
    }
    const Vector3f expected[] = {{-.5f, -.5f, .5f}, {-.5f, .5f, -.5f},
        {.5f, -.5f, -.5f}, {.5f, .5f, .5f}
    };
    for (uint8_t m = 0; m < 4; m++) {
        if (c.factors[m].is_nan() || c.factors[m].is_inf() || (c.factors[m] - expected[m]).length() > 1.0e-6f ||
            !isfinite(c.collective[m]) || fabsf(c.collective[m] - 1) > 1.0e-6f ||
            !isfinite(c.thrust[m]) || c.thrust[m] < -1.0e-6f || c.thrust[m] > 1 + 1.0e-6f ||
            !isfinite(c.post_thrust[m]) || fabsf(c.post_thrust[m] - c.thrust[m] / c.gain) > 1.0e-5f) {
            return result;
        }
        result.allocated += c.factors[m] * (c.thrust[m] / c.gain);
    }
    for (uint8_t a = 0; a < 3; a++) {
        if (fabsf(c.command[a]) > 1 || fabsf(result.allocated[a]) > 1 + 1.0e-5f ||
            (c.pid_valid && fabsf(constrain_float(c.raw[a], -1, 1) - c.command[a]) > 1.0e-6f)) {
            return result;
        }
    }
    result.residual = c.raw - result.allocated;
    result.valid = !result.allocated.is_nan() && !result.allocated.is_inf() &&
                   !result.residual.is_nan() && !result.residual.is_inf();
    return result;
}
#endif // AP_UTB_RATE_SITL_AVAILABLE
