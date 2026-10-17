//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_SyntheticReference.h
///  @brief      Konečný DISARMED no-RC diagnostický generátor s eligibility, PREROLL/RUNNING/COMPLETE/ABORTED.
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
#pragma once
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "AP_UTB_config.h"
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_ENABLED && AP_UTB_BENCH_ENABLED
#include <AP_Math/AP_Math.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Reference only. No sensor substitution, actuator or logger interfaces.
/// @brief UTB component; Konečný DISARMED no-RC diagnostický generátor s eligibility,
///        PREROLL/RUNNING/COMPLETE/ABORTED.
class AP_UTB_SyntheticReference
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    enum class Status : uint8_t { OFF, WAITING, RUNNING, COMPLETE, ABORTED, PREROLL };
    struct Sample {
        Vector3f rates;
        uint32_t cycle = 0;
        uint8_t phase = 0;
        Status status = Status::OFF;
        bool valid = false;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Update.
    ///
    ///  @param[in]  request Request.
    ///  @param[in]  eligible Eligible.
    ///  @param[in]  armed Armed.
    ///  @param[in]  watchdog Watchdog.
    ///  @param[in]  mode Mode.
    ///  @return     Sample.
    ///
    ///  @details    Public class member. Declaration/definition: Sample update(uint8_t request, bool eligible, bool armed,
    ///              bool watchdog, uint8_t mode).
    //--------------------------------------------------------------------------------------------------------------------
    Sample update(uint8_t request, bool eligible, bool armed, bool watchdog, uint8_t mode);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Abort.
    ///
    ///  @param[in]  None
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void abort().
    //--------------------------------------------------------------------------------------------------------------------
    void abort();
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Signal.
    ///
    ///  @param[in]  cycle Reference cycle index.
    ///  @param[in]  profile Explicit finite reference profile identifier.
    ///  @return     Vector3f.
    ///
    ///  @details    Public class member. Declaration/definition: static Vector3f signal(uint32_t cycle, uint8_t profile).
    //--------------------------------------------------------------------------------------------------------------------
    static Vector3f signal(uint32_t cycle, uint8_t profile);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Phase.
    ///
    ///  @param[in]  cycle Reference cycle index.
    ///  @return     uint8_t.
    ///
    ///  @details    Public class member. Declaration/definition: static uint8_t phase(uint32_t cycle).
    //--------------------------------------------------------------------------------------------------------------------
    static uint8_t phase(uint32_t cycle);
    static constexpr uint32_t CYCLES = 8000;
private:
    uint32_t _cycle = 0;
    uint16_t _preroll = 0;
    uint8_t _mode = 0;
    uint8_t _profile = 0;
    Status _status = Status::OFF;
};
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
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
