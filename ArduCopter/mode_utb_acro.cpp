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
#include "Copter.h"

#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
bool ModeUTBAcro::init(bool ignore_checks)
{
    // This restriction also applies when ordinary mode-entry checks are skipped.
    if (!enabled() || motors->armed() ||
        motors->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN) {
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UTB phase 0: disarmed diagnostics only");
    return true;
}

bool ModeUTBAcro::enabled() const
{
    return copter.utb.enabled();
}

void ModeUTBAcro::run()
{
    copter.utb.update_state();
}
#endif
