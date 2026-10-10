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
#if AP_UTB_ENABLED
#include <AP_Math/AP_Math.h>
// AP-derived UTB shadow reference: static map only, no trainer/RATE_TC.
class AP_UTB_Reference
{
public:
    static bool rates(const Vector3f &sticks, float rp_rate_deg, float y_rate_deg,
                      float rp_expo, float y_expo, Vector3f &target);
};
#endif // AP_UTB_ENABLED
