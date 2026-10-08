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

class AP_AHRS_View;

// Read-only diagnostic snapshot. Units are radians and radians/second, body FRD.
struct AP_UTB_State {
    Quaternion attitude_body_to_ned;
    Vector3f rates_rads;
    uint32_t sampled_us = 0;
    uint32_t imu_updated_us = 0;
    bool attitude_valid = false;
    bool rates_valid = false;

    bool valid(uint32_t now_us, uint32_t max_age_us) const;
};

class AP_UTB_StateProvider
{
public:
    virtual ~AP_UTB_StateProvider() = default;
    virtual bool sample(AP_UTB_State &state) const = 0;
};

class AP_UTB_APStateProvider : public AP_UTB_StateProvider
{
public:
    void init(const AP_AHRS_View &view)
    {
        _view = &view;
    }
    bool sample(AP_UTB_State &state) const override;

private:
    const AP_AHRS_View *_view = nullptr;
};
#endif // AP_UTB_ENABLED
