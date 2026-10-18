//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file mode_utb_acro_sitl.h
/// @brief Samostatný pilotní rate režim 43 se sdíleným pilotním adaptérem, pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-17
/// @date 2026-10-17
//--------------------------------------------------------------------------------------------------------------------
#pragma once
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "mode_utb_angle_sitl.h"
// End Include Files
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Sdílí lifecycle/RC/throttle/rate adaptér; větev 43 nikdy nevolá attitude update.
class ModeUTBAcroSITL : public ModeUTBAngleSITL
{
public:
    //================================================================================================================
    // Public functions
    //----------------------------------------------------------------------------------------------------------------
    using ModeUTBAngleSITL::ModeUTBAngleSITL;
    /// @brief Vrátit samostatné volné číslo pilotního ACRO.
    /// @param[in] None
    /// @return UTB_ACRO_SITL=43.
    /// @details Historický nearmovatelný mode 40 zůstává beze změny.
    Number mode_number() const override
    {
        return Number::UTB_ACRO_SITL;
    }
    // End Public functions
protected:
    //================================================================================================================
    // Private functions
    //----------------------------------------------------------------------------------------------------------------
    /// @brief Pojmenovat režim pro standardní GCS registraci.
    /// @param[in] None
    /// @return SITL pilotní ACRO název.
    /// @details Žádný nový fyzický hardware mode.
    const char *name() const override
    {
        return "UTB Acro SITL";
    }
    /// @brief Poskytnout krátké mode označení.
    /// @param[in] None
    /// @return UACR.
    /// @details Používá původní AP notify.
    const char *name4() const override
    {
        return "UACR";
    }
    // End Private functions
};
// End Data type, constant, macro definition
#endif // AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME

//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
// End Static data declarations

//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
// End Private function prototypes

//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
// None
// End Private functions
