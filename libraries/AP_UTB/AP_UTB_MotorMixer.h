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
class AP_UTB_MotorMixer
{
public:
    struct Result {
        float thrust[4] {};
        bool valid = false;
    };

    // No AP_Motors or HAL output access is available to this skeleton.
    Result mix(const AP_UTB_RateController::Result &control, float thrust) const;
};
#endif // AP_UTB_ENABLED
