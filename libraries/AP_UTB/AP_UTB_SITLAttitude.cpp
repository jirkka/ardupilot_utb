//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_SITLAttitude.cpp
///  @brief      Vlastní reduced rotation P: error z RᵀRd·ez, common XY rate limiter, explicitní yaw body rate.
///  @author     Jiri Kovar
///  @version    2026-10-17
///  @date       2026-10-17
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
#include "AP_UTB_SITLAttitude.h"
#if AP_UTB_RATE_SITL_AVAILABLE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Compare every configuration field without a tolerance deadband.
///
///  @param[in]  other Other value used for comparison.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB_SITLAttitude::Config::operator==(const Config
///              &other).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_SITLAttitude::Config::operator==(const Config &other) const
{
    return roll_p <= other.roll_p && roll_p >= other.roll_p &&
           pitch_p <= other.pitch_p && pitch_p >= other.pitch_p &&
           rate_max <= other.rate_max && rate_max >= other.rate_max;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Clear the component history so the next valid sample starts a new context.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB_SITLAttitude::reset().
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB_SITLAttitude::reset()
{
    _configured = _context = false;
    _fault = Fault::NONE;
    _last_imu = _calls = 0;
    _gyro = 255;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Latch the first fault and clear controller history to prevent continued control.
///
///  @param[in]  fault Fault.
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void AP_UTB_SITLAttitude::invalidate(Fault fault).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB_SITLAttitude::invalidate(Fault fault)
{
    if (_fault == Fault::NONE) {
        _fault = fault;
    }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Validate and store configuration; reset dependent controller history when required.
///
///  @param[in]  config Config.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB_SITLAttitude::configure(const Config &config).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_SITLAttitude::configure(const Config &config)
{
    reset();
    if (!isfinite(config.roll_p) || !isfinite(config.pitch_p) || !isfinite(config.rate_max) ||
        config.roll_p < 1 || config.roll_p > 6 || config.pitch_p < 1 || config.pitch_p > 6 ||
        config.rate_max < 0.05f || config.rate_max > 0.3f) {
        invalidate(Fault::CONFIG);
        return false;
    }
    _config = config;
    _configured = true;
    return true;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Update.
///
///  @param[in]  state Sampled attitude, gyro and validity metadata.
///  @param[in]  ref Ref.
///  @param[in]  dt Sample interval in seconds.
///  @param[in]  now_us Monotonic validation timestamp in microseconds.
///  @return     AP_UTB_SITLAttitude::Result.
///
///  @details    Roll/pitch reduced-rotation P produces body rates; yaw input is body rate, not heading hold. SITL only.
//--------------------------------------------------------------------------------------------------------------------
AP_UTB_SITLAttitude::Result AP_UTB_SITLAttitude::update(const AP_UTB_State &state, const Vector3f &ref,
        float dt, uint32_t now_us)
{
    // Norma vodorovných složek požadovaného směru.
    float sine;
    // Pomocný úhel v radiánech.
    float angle;
    // Bezrozměrné měřítko redukované rotační chyby.
    float scale;
    // Norma požadované roll/pitch úhlové rychlosti v rad/s.
    float rp;

    // Výsledek se stane platným až po dokončení všech kontrol.
    Result out;
    // Požadovaná orientace při zachování aktuálního yaw.
    Quaternion desired;
    // Požadovaná osa dolů vyjádřená v aktuálních osách tělesa.
    Vector3f direction;

    if (_fault != Fault::NONE) {
        return out;
    }
    if (!_configured) {
        invalidate(Fault::CONFIG);
    } else if (!isfinite(dt) || dt < 0.000625f || dt > 0.01f) {
        invalidate(Fault::DT);
    } else if (!state.valid(now_us, 10000)) {
        invalidate(Fault::STATE);
    } else if (_context && (state.imu_updated_us == _last_imu || state.primary_gyro != _gyro)) {
        invalidate(Fault::IMU);
    } else if (ref.is_nan() || ref.is_inf() || Vector2f(ref.x, ref.y).length() > radians(5.0f) + 1.0e-7f ||
               fabsf(ref.z) > 0.1f) {
        invalidate(Fault::REFERENCE);
    }
    if (_fault != Fault::NONE) {
        return out;
    }
    state.attitude_body_to_ned.to_euler(out.euler);
    if (MAX(fabsf(out.euler.x), fabsf(out.euler.y)) > radians(35.0f)) {
        invalidate(Fault::ENVELOPE);
        return {};
    }
    desired.from_euler(ref.x, ref.y, out.euler.z);
    // Target body-down direction expressed in the current body frame.
    direction = state.attitude_body_to_ned.inverse() * (desired * Vector3f(0, 0, 1));
    sine = Vector2f(direction.x, direction.y).length();
    angle = atan2f(sine, direction.z);
    scale = sine > 1.0e-7f ? angle / sine : 1.0f;
    out.error = Vector3f(-direction.y, direction.x, 0) * scale;
    out.rates = {out.error.x * _config.roll_p, out.error.y * _config.pitch_p, ref.z};
    rp = Vector2f(out.rates.x, out.rates.y).length();
    if (rp > _config.rate_max) {
        out.rates.x *= _config.rate_max / rp;
        out.rates.y *= _config.rate_max / rp;
        out.limited = true;
    }
    _last_imu = state.imu_updated_us;
    _gyro = state.primary_gyro;
    _context = true;
    ++_calls;
    out.valid = true;
    return out;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Reference.
///
///  @param[in]  profile Explicit finite reference profile identifier.
///  @param[in]  cycle Reference cycle index.
///  @param[in]  angle_rad Reference tilt amplitude in radians.
///  @param[in]  yaw_rate Desired body yaw rate in rad/s.
///  @return     Vector3f.
///
///  @details    Public class member. Declaration/definition: Vector3f AP_UTB_SITLAttitude::reference(uint8_t profile,
///              uint32_t cycle, float angle_rad, float yaw_rate).
//--------------------------------------------------------------------------------------------------------------------
Vector3f AP_UTB_SITLAttitude::reference(uint8_t profile, uint32_t cycle, float angle_rad, float yaw_rate)
{
    // Právě počítaná hodnota reference nebo výstupní složky.
    float value;
    // Roll/pitch reference a předepsaná yaw rychlost.
    Vector3f ref;

    if (cycle >= DURATION_CYCLES) {
        return {};
    }
    ref = Vector3f(0, 0, yaw_rate);
    value = cycle < 400 ? 0 : cycle < 1600 ? angle_rad : cycle < 2800 ? 0 :
            cycle < 4000 ? -angle_rad : 0;
    if (profile == 1 || profile == 2) {
        ref[profile - 1] = value;
    } else if (profile == 4) {
        ref.x = value; ref.y = -value;
    } else if (profile == 5 && cycle >= 400 && cycle < 4400) {
        ref.x = angle_rad * sinf(2 * M_PI * 0.2f * (cycle - 400) * 0.0025f);
    } else if (profile == 6 && cycle >= 400 && cycle < 4000) {
        ref.x = ((cycle - 400) / 1200) % 2 == 0 ? angle_rad : -angle_rad;
    }
    return ref;
}
#endif // AP_UTB_RATE_SITL_AVAILABLE

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
