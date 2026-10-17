//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_MotorMixer.cpp
///  @brief      Diagnostická alokace class 1 BF_X=12/BF_X_REV=18, společné škálování R/P/Y a posun collective.
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
#include "AP_UTB_MotorMixer.h"
#if AP_UTB_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Allocate diagnostic motor values while preserving the common R/P/Y direction.
///
///  @param[in]  control Control.
///  @param[in]  thrust Normalized diagnostic collective request.
///  @param[in]  frame_class Configured AP frame class.
///  @param[in]  frame_type Configured AP frame type.
///  @return     AP_UTB_MotorMixer::Result.
///
///  @details    Diagnostic allocation only. These four values are never physical motor commands.
//--------------------------------------------------------------------------------------------------------------------
AP_UTB_MotorMixer::Result AP_UTB_MotorMixer::mix(
    const AP_UTB_RateController::Result &control, float thrust, uint8_t frame_class, uint8_t frame_type) const
{
    // Znaménko yaw geometrie pro explicitně zvolený frame.
    float yaw_sign;
    // Yaw složka upravená podle geometrie rámu.
    float yaw;
    // Nejnižší diferenciální motorový požadavek.
    float low;
    // Nejvyšší diferenciální motorový požadavek.
    float high;
    // Rozpětí diferenciálních motorových požadavků.
    float span;
    // Dosažitelný normalizovaný společný motorový požadavek.
    float collective;
    // Právě počítaná hodnota reference nebo výstupní složky.
    float value;
    // Rozdíl požadovaného a dosaženého normalizovaného povelu.
    float residual;
    // Index průchodu pevnou kolekcí.
    uint8_t i;

    // Kompletní diagnostický výsledek alokace, zpočátku neplatný.
    Result result;
    // Požadovaný normalizovaný roll/pitch/yaw moment.
    const Vector3f &c = control.command;
    // Diferenciální složky čtyř motorů před posunem collective.
    float moment[4];
    // Čtení výsledných čtyř hodnot bez pomocného ukazatele.
    const float (&m)[4] = result.thrust;
    if (!supports_frame(frame_class, frame_type) || !control.valid || !isfinite(thrust) || thrust < 0 || thrust > 1 ||
        c.is_nan() || c.is_inf() || fabsf(c.x) > 1 || fabsf(c.y) > 1 || fabsf(c.z) > 1) {
        return result;
    }
    // BF_X_REV reverses only the yaw column; allocation and inverse use the same sign.
    yaw_sign = frame_type == 18 ? -1.0f : 1.0f;
    yaw = yaw_sign * c.z;
    moment[0] = (-c.x - c.y - yaw) * 0.5f;
    moment[1] = (-c.x + c.y + yaw) * 0.5f;
    moment[2] = (c.x - c.y + yaw) * 0.5f;
    moment[3] = (c.x + c.y - yaw) * 0.5f;
    low = moment[0];
    high = moment[0];
    for (i = 1; i < 4; i++) {
        low = MIN(low, moment[i]);
        high = MAX(high, moment[i]);
    }
    span = high - low;
    result.scale = span > 1 ? 1 / span : 1;
    collective = constrain_float(thrust, -result.scale * low, 1 - result.scale * high);
    result.collective_shift = collective - thrust;
    for (i = 0; i < 4; i++) {
        value = collective + result.scale * moment[i];
        if (!isfinite(value) || value < -1.0e-5f || value > 1 + 1.0e-5f) {
            return {};
        }
        result.thrust[i] = constrain_float(value, 0, 1);
        if (result.thrust[i] <= 1.0e-5f) {
            result.lower_mask |= 1U << i;
        }
        if (result.thrust[i] >= 1 - 1.0e-5f) {
            result.upper_mask |= 1U << i;
        }
    }
    result.achieved_thrust = (m[0] + m[1] + m[2] + m[3]) * 0.25f;
    result.achieved = Vector3f((-m[0] - m[1] + m[2] + m[3]) * 0.5f,
                               (-m[0] + m[1] - m[2] + m[3]) * 0.5f,
                               yaw_sign * ((-m[0] + m[1] + m[2] - m[3]) * 0.5f));
    for (i = 0; i < 4; i++) {
        residual = i == 3 ? thrust - result.achieved_thrust : c[i] - result.achieved[i];
        if (residual > 1.0e-5f) {
            result.positive_limits |= 1U << i;
        }
        if (residual < -1.0e-5f) {
            result.negative_limits |= 1U << i;
        }
    }
    result.moments_scaled = result.scale < 1 - 1.0e-5f;
    result.collective_shifted = fabsf(result.collective_shift) > 1.0e-5f;
    result.valid = true;
    return result;
}
#endif // AP_UTB_ENABLED

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
