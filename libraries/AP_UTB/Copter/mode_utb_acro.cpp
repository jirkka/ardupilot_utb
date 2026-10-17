//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       mode_utb_acro.cpp
///  @brief      Neaktivní DISARMED skeleton; init vyžaduje SHUT_DOWN, run pouze update_state.
///  @author     Jiri Kovar
///  @version    2026-10-16
///  @date       2026-10-16
//--------------------------------------------------------------------------------------------------------------------
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
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "../../../ArduCopter/Copter.h"

#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Init.
///
///  @param[in]  ignore_checks Ignore checks.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool ModeUTBAcro::init(bool ignore_checks).
//--------------------------------------------------------------------------------------------------------------------
bool ModeUTBAcro::init(bool ignore_checks)
{
    // This restriction also applies when ordinary mode-entry checks are skipped.
    if (!enabled() || motors->armed() ||
        motors->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN) {
        return false;
    }
    return true;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Report whether this component is enabled in the current configuration.
///
///  @param[in]  None
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool ModeUTBAcro::enabled().
//--------------------------------------------------------------------------------------------------------------------
bool ModeUTBAcro::enabled() const
{
    return copter.utb.enabled();
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Run.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void ModeUTBAcro::run().
//--------------------------------------------------------------------------------------------------------------------
void ModeUTBAcro::run()
{
    copter.utb.update_state();
}
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private function prototypes
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
