//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file AP_UTB_AcroRC.h
/// @brief Lineární pilotní body-rate reference; sdílená AP RC validace, pouze SITL.
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
#include "AP_UTB_AngleRC.h"
// End Include Files
#if AP_UTB_RATE_SITL_AVAILABLE
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class AP_UTB_AcroRC
{
public:
    // Hard caps omezují parametrickou pilotní obálku; zvýšení vyžaduje novou validaci.
    static constexpr float ROLL_MAX = 0.15f;
    static constexpr float PITCH_MAX = 0.15f;
    static constexpr float YAW_MAX = 0.05f;
    // End Data type, constant, macro definition
    //================================================================================================================
    // Public functions
    //----------------------------------------------------------------------------------------------------------------
    /// @brief Ověřit parametry proti explicitním hard caps, bez tichého clippingu konfigurace.
    /// @param[in] limits Maximální kladné R/P/Y body rates v rad/s.
    /// @return True pouze pro konečný podporovaný rozsah.
    /// @details Stejné limity se zmrazí při mode entry a kontrolují za běhu.
    static bool valid_config(const Vector3f &limits)
    {
        return !limits.is_nan() && !limits.is_inf() && limits.x > 0 && limits.x <= ROLL_MAX &&
               limits.y > 0 && limits.y <= PITCH_MAX && limits.z > 0 && limits.z <= YAW_MAX;
    }
    /// @brief Rozlišit neaplikovaný DISARMED AP kontext od aktivního throttle požadavku.
    /// @param[in] raw Původní AP_Motors raw throttle před jeho omezením.
    /// @param[in] armed Skutečný AP arming stav.
    /// @return True pro konečný neaktivní kontext; při ARMED navíc pouze rozsah 0..1.
    /// @details LAND ground relax může vracet záporný raw PID povel při nulovém fyzickém outputu.
    ///          Tato hodnota se nepřebírá do UTB; nový manual throttle vzniká jen z validovaného RC.
    static bool entry_throttle_valid(float raw, bool armed)
    {
        return isfinite(raw) && (!armed || (raw >= 0 && raw <= 1));
    }
    /// @brief Ověřit jedinou aktuálně podporovanou původní AP failsafe politiku.
    /// @param[in] gcs FS_GCS_ENABLE.
    /// @param[in] radio FS_THR_ENABLE.
    /// @param[in] options FS_OPTIONS.
    /// @return True pro RTL/RTL bez pokračování pilotního řízení.
    /// @details Nenahrazuje runtime AP failsafe handlery ani neobnovuje řízení po faultu.
    static bool failsafe_policy(int8_t gcs, int8_t radio, int32_t options)
    {
        return gcs == 1 && radio == 1 && options == 0;
    }
    /// @brief Znovu použít skutečnou AP RC validaci a vytvořit lineární body rates bez attitude smyčky.
    /// @param[in] roll Kalibrovaný RCMAP roll.
    /// @param[in] pitch Kalibrovaný RCMAP pitch.
    /// @param[in] yaw Kalibrovaný RCMAP yaw.
    /// @param[in] throttle Kalibrovaný RCMAP throttle.
    /// @param[in] mapped_throttle Původní AP manual throttle mapa, 0..1.
    /// @param[in] limits Samostatná maxima body rates v rad/s.
    /// @param[in] signal Explicitní AP validita a monotónní časy.
    /// @return Validovaný snapshot nebo nulová neplatná reference.
    /// @details Bez kruhového RP omezení, expo, traineru a dynamic shaping; střed je zero rate.
    static AP_UTB_AngleRC::Result read(const RC_Channel &roll, const RC_Channel &pitch, const RC_Channel &yaw,
                                       const RC_Channel &throttle, float mapped_throttle,
                                       const Vector3f &limits, const AP_UTB_AngleRC::Signal &signal)
    {
        // Jediný snapshot sdílené ověřené RC infrastruktury; attitude reference se nepoužije.
        AP_UTB_AngleRC::Result result;

        if (!valid_config(limits)) {
            result.fault = AP_UTB_AngleRC::Fault::CONFIG;
            return result;
        }
        result = AP_UTB_AngleRC::read(roll, pitch, yaw, throttle, mapped_throttle, {5, limits.z}, signal);
        if (result.valid) {
            result.reference = {result.normalized.x * limits.x, result.normalized.y * limits.y,
                                result.normalized.z * limits.z
                               };
        }
        return result;
    }
    // End Public functions
};
// End Data type, constant, macro definition
#endif // AP_UTB_RATE_SITL_AVAILABLE

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
