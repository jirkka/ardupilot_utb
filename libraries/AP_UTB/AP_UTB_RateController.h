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

#include "AP_UTB_State.h"

#if AP_UTB_ENABLED
class AP_UTB_RateController
{
public:
    struct Result {
        Vector3f command;
        bool valid = false;
    };

    // Phase 0 never produces a usable control command.
    Result update(const AP_UTB_State &state, const Vector3f &target_rads, float dt_s) const;
};
#endif // AP_UTB_ENABLED
