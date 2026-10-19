//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file AP_UTB_AngleRC.cpp
/// @brief Pilotní ANGLE reference ze skutečné AP RC normalizace, pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-18
/// @date 2026-10-18
//--------------------------------------------------------------------------------------------------------------------
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
#if AP_UTB_RATE_SITL_AVAILABLE
// End Include Files
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
/// @brief Ověřit požadované limity proti aktuálně podporované obálce.
/// @param[in] config Úhel v ° a yaw rate v rad/s.
/// @return True pro konečné hodnoty uvnitř podporované obálky.
/// @details Legacy reference má limit 5°, explicitní experimentální reference 60°; nejde o letovou kvalifikaci.
bool AP_UTB_AngleRC::valid_config(const Config &config)
{
    return isfinite(config.angle_deg) && config.angle_deg > 0 && config.angle_deg <= (config.extended ? 60 : 5) &&
           isfinite(config.yaw_rate) && config.yaw_rate > 0 && config.yaw_rate <= 0.1f;
}
/// @brief Rozlišit výpadek signálu od zastaralého timestampu.
/// @param[in] signal Explicitní AP stav a monotónní časy v ms.
/// @return Důvod odmítnutí nebo NONE.
/// @details Unsigned rozdíl zachovává platnost při uint32 wrap.
AP_UTB_AngleRC::Fault AP_UTB_AngleRC::signal_fault(const Signal &signal)
{
    if (!signal.valid || signal.failsafe) {
        return Fault::SIGNAL;
    }
    // Platný AP timestamp může být přesně nula při uint32 wrap; chybějící RC určuje signal.valid.
    if (signal.timeout_ms == 0 || uint32_t(signal.now_ms - signal.updated_ms) > signal.timeout_ms) {
        return Fault::STALE;
    }
    return Fault::NONE;
}
/// @brief Odmítnout neplatnou kalibraci před dělením v AP normalizaci.
/// @param[in] channel Kalibrovaný AP kanál.
/// @param[in] throttle RANGE throttle nebo ANGLE R/P/Y.
/// @return True pro správný typ a neprázdné intervaly.
/// @details Kalibrace se pouze čte; nepoužívá se set/save.
bool AP_UTB_AngleRC::calibration_valid(const RC_Channel &channel, bool throttle)
{
    // Kalibrované endpointy, trim a mrtvá zóna v mikrosekundách RC PWM.
    const int32_t minimum = channel.get_radio_min();
    const int32_t maximum = channel.get_radio_max();
    const int32_t trim = channel.get_radio_trim();
    const int32_t deadband = channel.get_dead_zone();

    if (minimum <= 0 || maximum <= minimum) {
        return false;
    }
    if (throttle) {
        return channel.get_type() == RC_Channel::ControlType::RANGE && channel.get_range() == 1000 &&
               minimum + deadband < maximum;
    }
    return channel.get_type() == RC_Channel::ControlType::ANGLE &&
           minimum < trim - deadband && trim + deadband < maximum;
}
/// @brief Vytvořit čerstvou omezenou referenci ze skutečných AP kanálů.
/// @param[in] roll RCMAP roll.
/// @param[in] pitch RCMAP pitch.
/// @param[in] yaw RCMAP yaw.
/// @param[in] throttle RCMAP throttle.
/// @param[in] mapped_throttle AP manual throttle, 0..1.
/// @param[in] config Limity úhlu a yaw rychlosti.
/// @param[in] signal Stav signálu a timestamp.
/// @return Platný snapshot nebo nulový neplatný výsledek s důvodem.
/// @details Kruh omezuje společnou RP referenci, yaw má samostatný limit.
AP_UTB_AngleRC::Result AP_UTB_AngleRC::read(
    const RC_Channel &roll, const RC_Channel &pitch, const RC_Channel &yaw, const RC_Channel &throttle,
    float mapped_throttle, const Config &config, const Signal &signal)
{
    // Nulový výstup zůstává neplatný, dokud neprojdou všechny kontroly.
    Result result;
    // Normové měřítko kombinovaného roll/pitch vstupu.
    float length;

    result.rc_ms = signal.updated_ms;
    result.fault = signal_fault(signal);
    if (result.fault != Fault::NONE) {
        return result;
    }
    if (!valid_config(config)) {
        result.fault = Fault::CONFIG;
        return result;
    }
    if (!calibration_valid(roll, false) || !calibration_valid(pitch, false) ||
        !calibration_valid(yaw, false) || !calibration_valid(throttle, true)) {
        result.fault = Fault::CALIBRATION;
        return result;
    }
    if (roll.get_radio_in() < roll.get_radio_min() || roll.get_radio_in() > roll.get_radio_max() ||
        pitch.get_radio_in() < pitch.get_radio_min() || pitch.get_radio_in() > pitch.get_radio_max() ||
        yaw.get_radio_in() < yaw.get_radio_min() || yaw.get_radio_in() > yaw.get_radio_max() ||
        throttle.get_radio_in() < throttle.get_radio_min() || throttle.get_radio_in() > throttle.get_radio_max()) {
        result.fault = Fault::RANGE;
        return result;
    }
    if (!isfinite(mapped_throttle) || mapped_throttle < 0 || mapped_throttle > 1) {
        result.fault = Fault::THROTTLE;
        return result;
    }
    result.normalized = {roll.norm_input_dz(), pitch.norm_input_dz(), yaw.norm_input_dz()};
    length = MAX(1.0f, Vector2f(result.normalized.x, result.normalized.y).length());
    result.reference = {radians(config.angle_deg) * result.normalized.x / length,
                        radians(config.angle_deg) * result.normalized.y / length,
                        config.yaw_rate * result.normalized.z
                       };
    result.throttle = mapped_throttle;
    result.valid = true;
    return result;
}
/// @brief Vrátit již ověřený SITL preset bez změny AP_Param konfigurace.
/// @param[out] gains Konstantní ověřené sady R/P/Y.
/// @return None.
/// @details Všechny vlastní matematické operace zůstávají v původním PID.
void AP_UTB_AngleRC::gains(AP_UTB_RateController::Gains (&gains)[3])
{
    gains[0] = {0.19f, 0.16f, 0.0015f, 0.19f, 15};
    gains[1] = {0.19f, 0.16f, 0.001f, 0.19f, 20};
    gains[2] = {0.30f, 0.60f, 0, 0.19f, 20};
}
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
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
