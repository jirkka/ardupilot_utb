//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file AP_UTB_AngleRC.h
/// @brief Validace skutečných AP RC kanálů a pilotní ANGLE reference, pouze SITL.
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
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "AP_UTB_RateController.h"
#include <RC_Channel/RC_Channel.h>
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class AP_UTB_AngleRC
{
public:
    enum class Fault : uint8_t { NONE, CONFIG, SIGNAL, STALE, CALIBRATION, RANGE, THROTTLE };
    struct Config {
        // Maximum normového náklonu a tělové yaw rychlosti.
        float angle_deg;
        float yaw_rate;
        /// @brief Inicializovat explicitní omezenou konfiguraci.
        /// @param[in] angle Maximální náklon v °.
        /// @param[in] yaw Maximální yaw rate v rad/s.
        /// @return None.
        /// @details Výchozí hodnoty odpovídají ověřené SITL obálce.
        Config(float angle = 5, float yaw = 0.1f) : angle_deg(angle), yaw_rate(yaw) {}
    };
    struct Signal {
        // Explicitní monotónní časy a původní AP stav signálu.
        uint32_t now_ms;
        uint32_t updated_ms;
        uint32_t timeout_ms;
        bool valid;
        bool failsafe;
        /// @brief Zachytit AP signal metadata bez skrytého čtení hodin.
        /// @param[in] now Čas kontroly v ms.
        /// @param[in] updated Čas posledního RC vstupu v ms.
        /// @param[in] timeout Původní AP timeout v ms.
        /// @param[in] input_valid Platnost původního AP RC vstupu.
        /// @param[in] radio_failsafe Původní AP radio failsafe.
        /// @return None.
        /// @details Výchozí prázdný signál je neplatný.
        Signal(uint32_t now = 0, uint32_t updated = 0, uint32_t timeout = 0,
               bool input_valid = false, bool radio_failsafe = false) :
            now_ms(now), updated_ms(updated), timeout_ms(timeout), valid(input_valid), failsafe(radio_failsafe) {}
    };
    struct Result {
        Vector3f normalized;
        Vector3f reference;
        float throttle = 0;
        uint32_t rc_ms = 0;
        Fault fault = Fault::NONE;
        bool valid = false;
    };
    // End Data type, constant, macro definition
    //================================================================================================================
    // Public functions
    //----------------------------------------------------------------------------------------------------------------
    /// @brief Ověřit nastavený limit uvnitř dosud validované úhlové/rate obálky.
    /// @param[in] config Maximální normový náklon v ° a yaw rate v rad/s.
    /// @return True pouze pro konečné podporované hodnoty.
    /// @details Nezvětšuje hard limit existujícího attitude controlleru.
    static bool valid_config(const Config &config);
    /// @brief Ověřit čerstvost skutečného AP RC timestampu včetně přetečení hodin.
    /// @param[in] signal Stav signálu a existující AP timeout.
    /// @return Důvod odmítnutí nebo NONE.
    /// @details Nepoužívá skryté hodiny a neprodlužuje původní RC timeout.
    static Fault signal_fault(const Signal &signal);
    /// @brief Ověřit kalibraci před použitím AP normalizace.
    /// @param[in] channel Skutečný RCMAP kanál.
    /// @param[in] throttle True pro RANGE throttle; jinak ANGLE vstup.
    /// @return True pro neprázdné kalibrované intervaly a očekávaný typ.
    /// @details Endpointy ani deadband neopravuje.
    static bool calibration_valid(const RC_Channel &channel, bool throttle);
    /// @brief Převést platné AP RC kanály na omezený pilotní ANGLE snapshot.
    /// @param[in] roll Roll kanál po skutečném RCMAP.
    /// @param[in] pitch Pitch kanál po skutečném RCMAP.
    /// @param[in] yaw Yaw kanál po skutečném RCMAP.
    /// @param[in] throttle Throttle kanál po skutečném RCMAP.
    /// @param[in] mapped_throttle Výstup původní AP manual throttle mapy.
    /// @param[in] config Limity reference.
    /// @param[in] signal Platnost a čerstvost RC.
    /// @return Snapshot, při chybě neplatný s nulovými povely.
    /// @details Normalizaci, reverse a deadband provádí RC_Channel, nikoli vlastní PID.
    static Result read(const RC_Channel &roll, const RC_Channel &pitch, const RC_Channel &yaw,
                       const RC_Channel &throttle, float mapped_throttle, const Config &config, const Signal &signal);
    /// @brief Sestavit explicitní již ověřený SITL rate preset pro nový pilotní mód.
    /// @param[out] gains Nastavení R/P/Y používané původním UTB PID.
    /// @return None.
    /// @details Nezmění globální RAT/S_* parametry, původní matematiku ani staré experimenty.
    static void gains(AP_UTB_RateController::Gains (&gains)[3]);
    // End Public functions
};
// End Data type, constant, macro definition
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
#endif // AP_UTB_RATE_SITL_AVAILABLE
