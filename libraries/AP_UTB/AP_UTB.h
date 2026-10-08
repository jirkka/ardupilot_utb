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
#include <AP_Param/AP_Param.h>
#include "AP_UTB_State.h"

class AP_UTB
{
public:
    AP_UTB();
    CLASS_NO_COPY(AP_UTB);

    void init(const AP_AHRS_View &view);
    bool enabled() const
    {
        return _enabled_at_boot;
    }
    void update_state();
    const AP_UTB_State &state() const
    {
        return _state;
    }

    // A valid state snapshot does not make the unimplemented controller healthy.
    bool healthy() const
    {
        return false;
    }

    static const AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _enable;
    bool _enabled_at_boot = false;
    AP_UTB_APStateProvider _provider;
    AP_UTB_State _state;
};
#endif // AP_UTB_ENABLED
