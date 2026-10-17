//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file mode_utb_angle_sitl.h
/// @brief Pilotní RC ANGLE režim s původními UTB regulátory, pouze skutečný SITL.
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
#include <AP_UTB/AP_UTB_AngleRC.h>
// End Include Files
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Included by mode.h after the original Mode base definition.
class ModeUTBAngleSITL : public Mode
{
public:
    enum class Stop : uint8_t { NONE, CONFIG, RC, STATE, ATTITUDE, RATE, VERTICAL, FAILSAFE };
    // End Data type, constant, macro definition
    //================================================================================================================
    // Public functions
    //----------------------------------------------------------------------------------------------------------------
    using Mode::Mode;
    /// @brief Vrátit ověřené volné lokální číslo režimu.
    /// @param[in] None
    /// @return UTB_ANGLE=42.
    /// @details Enum i registrace jsou pouze v SITL.
    Number mode_number() const override
    {
        return Number::UTB_ANGLE;
    }
    /// @brief Inicializovat pouze validní RC/stav a čisté controller historie.
    /// @param[in] ignore_checks AP flag; vlastní kontroly se neobcházejí.
    /// @return True pouze při splnění experimentálního kontraktu.
    /// @details Neprovádí arming ani automatický AP fallback.
    bool init(bool ignore_checks) override;
    /// @brief Uvolnit pilotní kontext a obnovit AP přechodovou historii.
    /// @param[in] None
    /// @return None.
    /// @details Volební mechanismus a následný mód zůstávají původní AP.
    void exit() override;
    /// @brief Odebrat skutečný RC snapshot a připravit ruční throttle/spool.
    /// @param[in] None
    /// @return None.
    /// @details Reference je určena následujícímu fast rate slotu.
    void run() override;
    /// @brief Zkontrolovat pouze vlastní explicitní pilotní opt-in.
    /// @param[in] None
    /// @return True při UTB boot enable a A_ENABLE=1.
    /// @details Původní S_OWNER se tím nemění.
    bool enabled() const override;
    /// @brief Oznámit absenci XY polohového řízení.
    /// @param[in] None
    /// @return False.
    /// @details Výška/vz jsou samostatné experimentální stop guards.
    bool requires_position() const override
    {
        return false;
    }
    /// @brief Zachovat standardní manual throttle klasifikaci AP.
    /// @param[in] None
    /// @return True.
    /// @details Má vliv na AP arming, přechody, hover learning a failsafe policy.
    bool has_manual_throttle() const override
    {
        return true;
    }
    /// @brief Povolit běžnou AP arming cestu pouze ve zdravém pilotním režimu.
    /// @param[in] method Původní AP metoda RC/MAVLink/AUX.
    /// @return True pouze pro běžnou metodu a zdravý kontext.
    /// @details Původní prearm/mandatory kontroly se vždy zachovávají.
    bool allows_arming(AP_Arming::Method method) const override;
    /// @brief Klasifikovat režim jako ručně řízený.
    /// @param[in] None
    /// @return False.
    /// @details Nezavádí autonomní misi ani výškové řízení.
    bool is_autopilot() const override
    {
        return false;
    }
    /// @brief Odmítnout vstup při ztraceném RC.
    /// @param[in] None
    /// @return False.
    /// @details Aktivní výpadek navíc zachytí latched stop a původní AP failsafe.
    bool allows_entry_in_rc_failsafe() const override
    {
        return false;
    }
    /// @brief Spustit právě jednu vlastní attitude/rate kaskádu před AP outputem.
    /// @param[in] dt Skutečná scheduler perioda v sekundách.
    /// @return None.
    /// @details Žádné volání původního AP attitude/rate PID.
    void rate_control(float dt);
    /// @brief Zachytit alokaci a diagnostiku po jediném původním motor outputu.
    /// @param[in] None
    /// @return None.
    /// @details Allocation capture běží i při vypnutém logování.
    void log_cycle();
    // End Public functions
protected:
    //================================================================================================================
    // Private functions
    //----------------------------------------------------------------------------------------------------------------
    /// @brief Vrátit zobrazovaný název pilotního režimu.
    /// @param[in] None
    /// @return Název pro GCS AVAILABLE_MODES.
    /// @details Veřejná registrace zůstává SITL-only.
    const char *name() const override
    {
        return "UTB Angle SITL";
    }
    /// @brief Vrátit krátké označení režimu.
    /// @param[in] None
    /// @return UANG.
    /// @details Slouží standardnímu AP mode notify.
    const char *name4() const override
    {
        return "UANG";
    }
private:
    /// @brief Odebrat kalibrované RCMAP kanály přes původní AP mapování.
    /// @param[in] None
    /// @return Platnost uloženého pilotního snapshotu.
    /// @details Neplatný vstup nikdy neponechá platnou starou referenci.
    bool read_pilot();
    /// @brief Zkontrolovat zmrazenou konfiguraci a konfliktní AP funkce.
    /// @param[in] None
    /// @return True při zachování entry konfigurace.
    /// @details Neopravuje parametry ani nepřepíná vlastníka PID.
    bool configuration_valid() const;
    /// @brief Zachytit původní AP alokaci pro další feedback cyklus.
    /// @param[in] None
    /// @return None.
    /// @details Používá již existující ověřený BF_X_REV kontrakt.
    void capture_allocation();
    /// @brief Zapsat nulové R/P/Y a FF požadavky do původního AP_Motors.
    /// @param[in] None
    /// @return None.
    /// @details Tato aktivní cesta je compile-out na hardware.
    void zero_rpy();
    /// @brief Zastavit režim s prvním uchovaným důvodem chyby.
    /// @param[in] reason Důvod chyby pilotního adaptéru.
    /// @return None.
    /// @details Obnovení RC ani parametru latch samo neodstraní.
    void stop(Stop reason);
    /// @brief Resetovat pouze AP přechodovou historii, nikoli spustit AP PID.
    /// @param[in] None
    /// @return None.
    /// @details Volá se při entry/exit, mimo vlastní aktivní výpočet.
    void reset_ap_history();
    // End Private functions
    //================================================================================================================
    // Static data declarations
    //----------------------------------------------------------------------------------------------------------------
    // Pevný výhradní vlastník a zdroj pro společné logovací schéma.
    static constexpr uint8_t _owner = 1;
    static constexpr uint8_t _angle_source = 1;
    static constexpr bool _complete = false;
    // Původní AP smyčky se v tomto režimu nevolají.
    static constexpr uint32_t _ap_calls = 0;
    static constexpr uint32_t _ap_angle_calls = 0;
    // End Static data declarations
    //================================================================================================================
    // Data type, constant, macro definition
    //----------------------------------------------------------------------------------------------------------------
    // Původní vlastní regulátory, jejich výsledky a odebraný AP stav.
    AP_UTB_SITLAttitude _attitude;
    AP_UTB_SITLRate _rate;
    AP_UTB_SITLAttitude::Result _angle_result;
    AP_UTB_RateController::Result _result;
    AP_UTB_State _sample;
    AP_UTB_SITLAllocation::Result _allocation;
    // Pilotní reference se skutečným RC timestampem a entry konfigurací.
    AP_UTB_AngleRC::Config _config;
    AP_UTB_AngleRC::Result _pilot;
    Vector3f _angle_reference, _reference, _target;
    // Skutečné PID složky a transported raw povel pro capture/log.
    Vector3f _p, _i, _d, _raw_applied;
    uint32_t _command_seq = 0;
    bool _raw_valid = false;
    // Čas a samostatné počty controller/output cyklů.
    uint64_t _time_us = 0;
    uint32_t _seq = 0, _motor_calls = 0, _execution_us = 0;
    // Host wall time posledního logger a pilot run průchodu v mikrosekundách.
    uint32_t _log_execution_us = 0, _run_execution_us = 0;
    float _dt = 0;
    // Ruční throttle a zachované AP mix/entry výškové kontexty.
    float _throttle = 0, _saved_mix = 0;
    float _entry_height = 0, _height = 0, _vertical_speed = 0;
    int8_t _antiwindup = 0;
    bool _have_mix = false, _running = false;
    Stop _stop = Stop::NONE;
    // End Data type, constant, macro definition
};
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
// End Private function prototypes
#endif // AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
