/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#include "AP_UTB_LogPacket.h"

#if AP_UTB_ENABLED
#include <string.h>

void UTBLogPacket::encode(const AP_UTB::Snapshot &s, void *words)
{
    Payload payload {};
    for (uint8_t axis = 0; axis < 3; axis++) {
        const auto &a = s.control.axis[axis];
        const float residual = a.raw - s.mixer.achieved[axis];
        payload.axis[axis] = {
            s.input.reference[axis], s.input.state.rates_rads[axis], a.error,
            a.p, a.i, a.d, a.raw, a.output, s.input.ap_target.target[axis],
            uint8_t(s.control.valid),
            uint8_t((residual > 1.0e-5f ? 1 : 0) | (residual < -1.0e-5f ? 2 : 0) |
            (a.integration_blocked ? 4 : 0))
        };
        payload.command[axis] = s.control.command[axis];
        payload.achieved[axis] = s.mixer.achieved[axis];
    }
    payload.collective = s.input.thrust;
    for (uint8_t i = 0; i < 4; i++) {
        payload.motor[i] = s.mixer.thrust[i];
    }
    payload.scale = s.mixer.scale;
    payload.shift = s.mixer.collective_shift;
    payload.mixer_valid = s.mixer.valid;
    payload.lower = s.mixer.lower_mask;
    payload.upper = s.mixer.upper_mask;
    payload.achieved_collective = s.mixer.achieved_thrust;
    payload.positive = s.mixer.positive_limits;
    payload.negative = s.mixer.negative_limits;
    payload.scaled = s.mixer.moments_scaled;
    payload.shifted = s.mixer.collective_shifted;
    payload.dt = s.input.dt;
    payload.execution_us = s.execution_us;
    payload.requested = s.requested_log_rate;
    payload.max_execution_us = s.max_execution_us;
    payload.version = VERSION;
    memcpy(words, &payload, sizeof(payload));
}
#endif // AP_UTB_ENABLED
