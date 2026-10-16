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
#pragma once
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include <AP_Math/AP_Math.h>

// Reconstruction of actual normalized allocation, never a physical torque estimate.
class AP_UTB_SITLAllocation
{
public:
    struct Capture {
        Vector3f factors[4];
        float collective[4] {};
        float thrust[4] {};
        float post_thrust[4] {};
        float gain = 0;
        Vector3f raw;
        Vector3f command;
        uint32_t output_seq = 0;
        uint32_t command_seq = 0;
        uint32_t sampled_us = 0;
        uint8_t limits = 0;
        bool factors_available = false;
        bool boost = false;
        bool full_spool = false;
        bool armed = false;
        bool pid_valid = false;
    };
    struct Result {
        Capture capture;
        Vector3f allocated;
        Vector3f residual;
        bool valid = false;
    };
    static Result reconstruct(const Capture &capture);
};
#endif // AP_UTB_RATE_SITL_AVAILABLE
