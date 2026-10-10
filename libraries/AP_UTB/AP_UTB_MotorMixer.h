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
#include "AP_UTB_RateController.h"
#if AP_UTB_ENABLED
// Diagnostic allocation only. No actuator, HAL or logger dependencies.
class AP_UTB_MotorMixer
{
public:
    struct Result {
        float thrust[4] {};
        Vector3f achieved;
        float achieved_thrust = 0;
        float scale = 0;
        float collective_shift = 0;
        uint8_t positive_limits = 0;
        uint8_t negative_limits = 0;
        uint8_t lower_mask = 0;
        uint8_t upper_mask = 0;
        bool moments_scaled = false;
        bool collective_shifted = false;
        bool valid = false;
    };
    static bool supports_frame(uint8_t frame_class, uint8_t frame_type)
    {
        return frame_class == 1 && frame_type == 12;
    }
    Result mix(const AP_UTB_RateController::Result &control, float thrust,
               uint8_t frame_class, uint8_t frame_type) const;
};
#endif // AP_UTB_ENABLED
