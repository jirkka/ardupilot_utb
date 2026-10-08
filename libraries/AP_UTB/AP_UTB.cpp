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
#include "AP_UTB.h"

#if AP_UTB_ENABLED
const AP_Param::GroupInfo AP_UTB::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: UTB diagnostic skeleton enable
    // @Description: Enables the disarmed-only UTB diagnostic mode after reboot. Phase 0 does not implement flight control and cannot arm.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_UTB, _enable, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_UTB::AP_UTB()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_UTB::init(const AP_AHRS_View &view)
{
    _enabled_at_boot = _enable.get() == 1;
    if (_enabled_at_boot) {
        _provider.init(view);
    }
}

void AP_UTB::update_state()
{
    if (_enabled_at_boot) {
        _provider.sample(_state);
    }
}
#endif // AP_UTB_ENABLED
