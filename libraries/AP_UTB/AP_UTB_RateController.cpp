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
#include "AP_UTB_RateController.h"

#if AP_UTB_ENABLED
AP_UTB_RateController::Result AP_UTB_RateController::update(
    const AP_UTB_State &state, const Vector3f &target_rads, float dt_s) const
{
    return {};
}
#endif // AP_UTB_ENABLED
