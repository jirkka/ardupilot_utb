//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_SyntheticReference.cpp
///  @brief      Konečný DISARMED no-RC diagnostický generátor s eligibility, PREROLL/RUNNING/COMPLETE/ABORTED.
///  @author     Jiri Kovar
///  @version    2026-10-17
///  @date       2026-10-17
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
#include "AP_UTB_SyntheticReference.h"
#if AP_UTB_ENABLED && AP_UTB_BENCH_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// Konce jednotlivých fází syntetické reference v počtech 400Hz cyklů.
static constexpr uint16_t phase_end[] = {200, 1800, 3400, 4200, 5000, 5800, 6600, 7400, 7800, 8000};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Phase.
///
///  @param[in]  cycle Reference cycle index.
///  @return     uint8_t.
///
///  @details    Public class member. Declaration/definition: uint8_t AP_UTB_SyntheticReference::phase(uint32_t cycle).
//--------------------------------------------------------------------------------------------------------------------
uint8_t AP_UTB_SyntheticReference::phase(uint32_t cycle)
{
    // Index průchodu pevnou kolekcí.
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(phase_end); i++) {
        if (cycle < phase_end[i]) {
            return i;
        }
    }
    return 9;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Signal.
///
///  @param[in]  cycle Reference cycle index.
///  @param[in]  profile Explicit finite reference profile identifier.
///  @return     Vector3f.
///
///  @details    Public class member. Declaration/definition: Vector3f AP_UTB_SyntheticReference::signal(uint32_t cycle,
///              uint8_t profile).
//--------------------------------------------------------------------------------------------------------------------
Vector3f AP_UTB_SyntheticReference::signal(uint32_t cycle, uint8_t profile)
{
    // Číslo právě aktivní fáze konečné interní reference.
    uint8_t section;
    // Počáteční čas nebo index vyhodnocovaného intervalu.
    uint32_t start;
    // Pomocný úhel v radiánech.
    float angle;
    // Amplituda diagnostického úhlového buzení v rad/s.
    float amplitude;

    if (cycle >= CYCLES) {
        return {};
    }
    section = phase(cycle);
    start = section == 0 ? 0 : phase_end[section - 1];
    angle = 2.0f * M_PI * float(cycle - start) / 800.0f;
    amplitude = 0.3f;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    amplitude = profile == 2 ? 8.0f : amplitude;
#else
    if (profile != 1) {
        return {};
    }
#endif
    switch (section) {
    case 1:
        return {amplitude, 0, 0};
    case 2:
        return {-amplitude, 0, 0};
    case 3:
        return {0, amplitude, 0};
    case 4:
        return {0, -amplitude, 0};
    case 5:
        return {amplitude * sinf(angle), 0, 0};
    case 6:
        return {0, 0, amplitude * sinf(angle)};
    case 7:
        return {amplitude * sinf(angle), amplitude * cosf(angle), amplitude * sinf(2 * angle)};
    default:
        return {};
    }
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Abort.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB_SyntheticReference::abort().
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB_SyntheticReference::abort()
{
    _status = Status::ABORTED;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Update.
///
///  @param[in]  request Request.
///  @param[in]  eligible Eligible.
///  @param[in]  armed Armed.
///  @param[in]  watchdog Watchdog.
///  @param[in]  mode Mode.
///  @return     AP_UTB_SyntheticReference::Sample.
///
///  @details    Public class member. Declaration/definition: AP_UTB_SyntheticReference::Sample
///              AP_UTB_SyntheticReference::update( uint8_t request, bool eligible, bool armed, bool watchdog, uint8_t mode).
//--------------------------------------------------------------------------------------------------------------------
AP_UTB_SyntheticReference::Sample AP_UTB_SyntheticReference::update(
    uint8_t request, bool eligible, bool armed, bool watchdog, uint8_t mode)
{
    // Příznak explicitně podporovaného referenčního profilu.
    bool supported;
    // Výstupní vzorek syntetické reference a jeho stav.
    Sample result;

    if (request == 0) {
        _cycle = 0;
        _preroll = 0;
        _profile = 0;
        _status = Status::OFF;
        return {};
    }
    supported = request == 1;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    supported |= request == 2;
#endif
    if (!supported || armed || watchdog || (_profile != 0 && _profile != request) ||
        ((_status == Status::RUNNING || _status == Status::PREROLL) && (!eligible || mode != _mode))) {
        abort();
    }
    if (_status == Status::OFF) {
        _status = Status::WAITING;
    }
    if (_status == Status::WAITING && eligible) {
        _status = Status::PREROLL;
        _preroll = 400;
        _mode = mode;
        _profile = request;
    }
    result.cycle = _cycle;
    result.status = _status;
    if (_status == Status::PREROLL) {
        result.valid = true;
        if (--_preroll == 0) {
            _status = Status::RUNNING;
        }
        return result;
    }
    if (_status != Status::RUNNING) {
        return result;
    }
    if (_cycle >= CYCLES) {
        _status = result.status = Status::COMPLETE;
        return result;
    }
    result.phase = phase(_cycle);
    result.rates = signal(_cycle++, request);
    result.valid = true;
    return result;
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
