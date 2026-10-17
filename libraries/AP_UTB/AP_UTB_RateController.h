//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_RateController.h
///  @brief      Tříosý PID, IMAX, D z gyro derivace s filtrem a jednokrokový directional integration gate.
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
#include "AP_UTB_State.h"
#if AP_UTB_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
/// @brief UTB component; Tříosý PID, IMAX, D z gyro derivace s filtrem a jednokrokový directional integration
///        gate.
class AP_UTB_RateController
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    struct Gains {
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // End Data type, constant, macro definition
        //====================================================================================================================
        // Public functions
        //--------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Gains.
        ///
        ///  @param[in]  None
        ///  @return     None (constructor/destructor).
        ///
        ///  @details    Public class member. Declaration/definition: Gains().
        //--------------------------------------------------------------------------------------------------------------------
        Gains() = default;
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Gains.
        ///
        ///  @param[in]  p_gain P gain.
        ///  @param[in]  i_gain I gain.
        ///  @param[in]  d_gain D gain.
        ///  @param[in]  i_max I max.
        ///  @param[in]  cutoff_hz Cutoff hz.
        ///  @return     None (constructor/destructor).
        ///
        ///  @details    Public class member. Declaration/definition: Gains(float p_gain, float i_gain, float d_gain, float
        ///              i_max, float cutoff_hz).
        //--------------------------------------------------------------------------------------------------------------------
        Gains(float p_gain, float i_gain, float d_gain, float i_max, float cutoff_hz) :
            p(p_gain), i(i_gain), d(d_gain), imax(i_max), d_hz(cutoff_hz) {}

        float p = 0;
        float i = 0;
        float d = 0;
        float imax = 0;
        float d_hz = 0;
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Compare every configuration field without a tolerance deadband.
        ///
        ///  @param[in]  other Other value used for comparison.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool operator==(const Gains &other).
        //--------------------------------------------------------------------------------------------------------------------
        bool operator==(const Gains &other) const;
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Validate the component contract, including finite values and the applicable bounds.
        ///
        ///  @param[in]  nominal_dt Nominal controller interval in seconds.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool valid(float nominal_dt).
        //--------------------------------------------------------------------------------------------------------------------
        bool valid(float nominal_dt) const;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Public functions
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    struct AxisResult {
        float error = 0;
        float p = 0;
        float i = 0;
        float d = 0;
        float raw = 0;
        float output = 0;
        float acceleration = 0;
        bool integration_blocked = false;
    };
    struct Result {
        AxisResult axis[3];
        Vector3f command;
        bool valid = false;
        bool priming = false;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Validate and store configuration; reset dependent controller history when required.
    ///
    ///  @param[in]  gains Per-axis controller gains.
    ///  @param[in]  nominal_dt Nominal controller interval in seconds.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool configure(const Gains (&gains)[3], float nominal_dt).
    //--------------------------------------------------------------------------------------------------------------------
    bool configure(const Gains (&gains)[3], float nominal_dt);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Clear the component history so the next valid sample starts a new context.
    ///
    ///  @param[in]  None
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void reset().
    //--------------------------------------------------------------------------------------------------------------------
    void reset();
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Update.
    ///
    ///  @param[in]  state Sampled attitude, gyro and validity metadata.
    ///  @param[in]  target_rads Desired body rates in rad/s.
    ///  @param[in]  dt_s Sample interval in seconds.
    ///  @return     Result.
    ///
    ///  @details    D differentiates the measured gyro; the first valid sample primes history. No actuator API is called.
    //--------------------------------------------------------------------------------------------------------------------
    Result update(const AP_UTB_State &state, const Vector3f &target_rads, float dt_s);
    // Caller-verified normalized allocation only; never infer a physical achieved torque from limit flags.
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Store caller-verified normalized allocation residuals for the next integration step.
    ///
    ///  @param[in]  result Result.
    ///  @param[in]  achieved Caller-verified achieved normalized R/P/Y request.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void feedback(const Result &result, const Vector3f
    ///              &achieved).
    //--------------------------------------------------------------------------------------------------------------------
    void feedback(const Result &result, const Vector3f &achieved);
private:
    Gains _gains[3];
    float _nominal_dt = 0;
    float _previous[3] {};
    float _acceleration[3] {};
    float _integral[3] {};
    float _residual[3] {};
    bool _configured = false;
    bool _primed = false;
};
#endif // AP_UTB_ENABLED

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
