//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_RateController.cpp
///  @brief      Tříosý PID, IMAX, D z gyro derivace s filtrem a jednokrokový directional integration gate.
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
#include "AP_UTB_RateController.h"
#if AP_UTB_ENABLED
// Configuration changes require exact finite-value comparison, not an epsilon deadband.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Compare finite configuration values exactly; NaN never compares equal.
///
///  @param[in]  a First value to compare.
///  @param[in]  b Second value to compare.
///  @return     bool.
///
///  @details    File-local helper (internal linkage). Declaration/definition: static bool same_value(float a, float b).
//--------------------------------------------------------------------------------------------------------------------
static bool same_value(float a, float b)
{
    return a <= b && a >= b;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Compare every configuration field without a tolerance deadband.
///
///  @param[in]  other Other value used for comparison.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB_RateController::Gains::operator==(const Gains
///              &other).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_RateController::Gains::operator==(const Gains &other) const
{
    return same_value(p, other.p) && same_value(i, other.i) && same_value(d, other.d) &&
           same_value(imax, other.imax) && same_value(d_hz, other.d_hz);
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Validate the component contract, including finite values and the applicable bounds.
///
///  @param[in]  nominal_dt Nominal controller interval in seconds.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB_RateController::Gains::valid(float nominal_dt).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_RateController::Gains::valid(float nominal_dt) const
{
    return isfinite(nominal_dt) && nominal_dt > 0 &&
           isfinite(p) && p >= 0 && p <= 5 && isfinite(i) && i >= 0 && i <= 5 &&
           isfinite(d) && d >= 0 && d <= 1 && isfinite(imax) && imax >= 0 && imax <= 1 &&
           isfinite(d_hz) && d_hz >= 0 && d_hz <= 200 &&
           (d <= 0 || (d_hz > 0 && d_hz <= MIN(200.0f, 0.1f / nominal_dt)));
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Validate and store configuration; reset dependent controller history when required.
///
///  @param[in]  gains Per-axis controller gains.
///  @param[in]  nominal_dt Nominal controller interval in seconds.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB_RateController::configure(const Gains (&gains)[3],
///              float nominal_dt).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB_RateController::configure(const Gains (&gains)[3], float nominal_dt)
{
    // Příznak změny konfigurace nebo provozního kontextu.
    bool changed;
    // Index průchodu pevnou kolekcí.
    uint8_t j;

    changed = !_configured || !same_value(nominal_dt, _nominal_dt);
    for (j = 0; j < 3; j++) {
        if (!gains[j].valid(nominal_dt)) {
            reset();
            _configured = false;
            return false;
        }
        changed |= !(gains[j] == _gains[j]);
    }
    if (changed) {
        reset();
    }
    for (j = 0; j < 3; j++) {
        _gains[j] = gains[j];
    }
    _nominal_dt = nominal_dt;
    _configured = true;
    return true;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Clear the component history so the next valid sample starts a new context.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB_RateController::reset().
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB_RateController::reset()
{
    // Index průchodu pevnou kolekcí.
    uint8_t j;

    _primed = false;
    for (j = 0; j < 3; j++) {
        _previous[j] = 0;
        _acceleration[j] = 0;
        _integral[j] = 0;
        _residual[j] = 0;
    }
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Update.
///
///  @param[in]  state Sampled attitude, gyro and validity metadata.
///  @param[in]  target_rads Desired body rates in rad/s.
///  @param[in]  dt_s Sample interval in seconds.
///  @return     AP_UTB_RateController::Result.
///
///  @details    D differentiates the measured gyro; the first valid sample primes history. No actuator API is called.
//--------------------------------------------------------------------------------------------------------------------
AP_UTB_RateController::Result AP_UTB_RateController::update(
    const AP_UTB_State &state, const Vector3f &target_rads, float dt_s)
{
    // Nefiltrovaná derivace měřené úhlové rychlosti v rad/s².
    float raw_acceleration;
    // Časová konstanta D filtru v sekundách.
    float tau;
    // Součet časové konstanty filtru a skutečného dt.
    float denominator;
    // Bezrozměrný koeficient diskrétního D filtru.
    float beta;
    // Přírůstek normalizovaného integrálního členu za vzorek.
    float delta_i;
    // Index průchodu pevnou kolekcí.
    uint8_t j;

    // Návratový výsledek; platnost se nastaví až po úspěšném výpočtu.
    Result result;
    if (!_configured || !isfinite(dt_s) || dt_s < _nominal_dt * 0.25f || dt_s > _nominal_dt * 4 ||
        !state.valid(state.sampled_us, uint32_t(MIN(0.1f, _nominal_dt * 4) * 1.0e6f)) ||
        target_rads.is_nan() || target_rads.is_inf()) {
        reset();
        return result;
    }
    if (!_primed) {
        for (j = 0; j < 3; j++) {
            _previous[j] = state.rates_rads[j];
        }
        _primed = true;
        result.priming = true;
        return result;
    }
    for (j = 0; j < 3; j++) {
        // Výstup právě zpracovávané osy bez kopírování.
        AxisResult &out = result.axis[j];
        // Konstantní nastavení právě zpracovávané osy.
        const Gains &g = _gains[j];
        out.error = target_rads[j] - state.rates_rads[j];
        out.p = g.p * out.error;
        raw_acceleration = (state.rates_rads[j] - _previous[j]) / dt_s;
        if (g.d > 0) {
            tau = 1.0f / (2.0f * M_PI * g.d_hz);
            denominator = tau + dt_s;
            beta = dt_s / denominator;
            if (!isfinite(tau) || !isfinite(denominator) || !isfinite(beta)) {
                reset();
                return {};
            }
            out.acceleration = (1 - beta) * _acceleration[j] + beta * raw_acceleration;
            out.d = -g.d * out.acceleration;
        }
        delta_i = g.i * out.error * dt_s;
        out.integration_blocked = (_residual[j] > 1.0e-5f && delta_i > 0) ||
                                  (_residual[j] < -1.0e-5f && delta_i < 0);
        out.i = g.i <= 0 ? 0 : constrain_float(_integral[j] + (out.integration_blocked ? 0 : delta_i),
                                               -g.imax, g.imax);
        out.raw = out.p + out.i + out.d;
        if (!isfinite(out.error) || !isfinite(out.p) || !isfinite(raw_acceleration) ||
            !isfinite(out.acceleration) || !isfinite(out.d) || !isfinite(delta_i) ||
            !isfinite(out.i) || !isfinite(out.raw)) {
            reset();
            return {};
        }
        out.output = constrain_float(out.raw, -1, 1);
        result.command[j] = out.output;
    }
    for (j = 0; j < 3; j++) {
        _previous[j] = state.rates_rads[j];
        _acceleration[j] = result.axis[j].acceleration;
        _integral[j] = result.axis[j].i;
    }
    result.valid = true;
    return result;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Store caller-verified normalized allocation residuals for the next integration step.
///
///  @param[in]  result Result.
///  @param[in]  achieved Caller-verified achieved normalized R/P/Y request.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB_RateController::feedback(const Result &result,
///              const Vector3f &achieved).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB_RateController::feedback(const Result &result, const Vector3f &achieved)
{
    // Rozdíl požadovaného a dosaženého normalizovaného povelu.
    float residual;
    // Index průchodu pevnou kolekcí.
    uint8_t j;

    if (!result.valid || achieved.is_nan() || achieved.is_inf()) {
        reset();
        return;
    }
    for (j = 0; j < 3; j++) {
        residual = result.axis[j].raw - achieved[j];
        if (!isfinite(result.axis[j].raw) || !isfinite(residual)) {
            reset();
            return;
        }
        _residual[j] = residual;
    }
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
