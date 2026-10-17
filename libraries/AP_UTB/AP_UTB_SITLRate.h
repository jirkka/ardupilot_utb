//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_SITLRate.h
///  @brief      SITL rate adaptér: frozen config, čerstvost/IMU/frame/dt/envelope guard, fault latch, delay a
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
#if AP_UTB_RATE_SITL_AVAILABLE
#include "AP_UTB_RateController.h"
#include "AP_UTB_SITLAllocation.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Experimental unsaturated adapter policy; has no motor or logger dependencies.
/// @brief UTB component; SITL rate adaptér: frozen config, čerstvost/IMU/frame/dt/envelope guard, fault latch,
///        delay a konečné reference.
class AP_UTB_SITLRate
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    enum class Fault : uint8_t { NONE, STATE, DT, FRAME, FAST_RATE, CONFIG, SATURATION, ENVELOPE, IMU, FEEDBACK, ATTITUDE, VERTICAL };
    struct Inputs {
        AP_UTB_State state;
        Vector3f target;
        uint32_t now_us = 0;
        uint32_t seq = 0;
        float dt = 0;
        float nominal_dt = 0;
        uint8_t frame_class = 0;
        uint8_t frame_type = 0;
        bool fast_rate = false;
        bool armed = false;
        bool full_spool = false;
        bool allocation_limited = false;
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
    ///  @param[in]  allocation_feedback Allocation feedback.
    ///  @param[in]  delayed Whether to use the explicit one-cycle transport delay.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool configure(const AP_UTB_RateController::Gains
    ///              (&gains)[3], bool allocation_feedback = false, bool delayed = false).
    //--------------------------------------------------------------------------------------------------------------------
    bool configure(const AP_UTB_RateController::Gains (&gains)[3], bool allocation_feedback = false, bool delayed = false);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Read the completed AP motor allocation for feedback on the next rate cycle.
    ///
    ///  @param[in]  feedback Feedback.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void capture_allocation(const AP_UTB_SITLAllocation::Result
    ///              &feedback).
    //--------------------------------------------------------------------------------------------------------------------
    void capture_allocation(const AP_UTB_SITLAllocation::Result &feedback)
    {
        _feedback = feedback;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Compare the runtime configuration with the frozen experiment configuration.
    ///
    ///  @param[in]  gains Per-axis controller gains.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool configuration_matches(const
    ///              AP_UTB_RateController::Gains (&gains)[3]).
    //--------------------------------------------------------------------------------------------------------------------
    bool configuration_matches(const AP_UTB_RateController::Gains (&gains)[3]) const;
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Step.
    ///
    ///  @param[in]  in Explicit input snapshot.
    ///  @param[in]  utb_owner Whether UTB owns this SITL rate-control step.
    ///  @return     AP_UTB_RateController::Result.
    ///
    ///  @details    Public class member. Declaration/definition: AP_UTB_RateController::Result step(const Inputs &in, bool
    ///              utb_owner).
    //--------------------------------------------------------------------------------------------------------------------
    AP_UTB_RateController::Result step(const Inputs &in, bool utb_owner);
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
    ///  @brief      Latch the first fault and clear controller history to prevent continued control.
    ///
    ///  @param[in]  reason Reason.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void invalidate(Fault reason).
    //--------------------------------------------------------------------------------------------------------------------
    void invalidate(Fault reason);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Fault.
    ///
    ///  @param[in]  None
    ///  @return     Fault.
    ///
    ///  @details    Public class member. Declaration/definition: Fault fault().
    //--------------------------------------------------------------------------------------------------------------------
    Fault fault() const
    {
        return _fault;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Calls.
    ///
    ///  @param[in]  None
    ///  @return     uint32_t.
    ///
    ///  @details    Public class member. Declaration/definition: uint32_t calls().
    //--------------------------------------------------------------------------------------------------------------------
    uint32_t calls() const
    {
        return _calls;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Transport.
    ///
    ///  @param[in]  command Normalized R/P/Y request.
    ///  @param[in]  delayed Whether to use the explicit one-cycle transport delay.
    ///  @param[in]  enabled Enabled.
    ///  @return     Vector3f.
    ///
    ///  @details    Public class member. Declaration/definition: Vector3f transport(const Vector3f &command, bool delayed,
    ///              bool enabled).
    //--------------------------------------------------------------------------------------------------------------------
    Vector3f transport(const Vector3f &command, bool delayed, bool enabled);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Reference.
    ///
    ///  @param[in]  profile Explicit finite reference profile identifier.
    ///  @param[in]  cycle Reference cycle index.
    ///  @param[in]  amplitude Amplitude.
    ///  @return     Vector3f.
    ///
    ///  @details    Public class member. Declaration/definition: static Vector3f reference(uint8_t profile, uint32_t cycle,
    ///              float amplitude).
    //--------------------------------------------------------------------------------------------------------------------
    static Vector3f reference(uint8_t profile, uint32_t cycle, float amplitude);
    static constexpr uint32_t DURATION_CYCLES = 3200;
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Duration cycles.
    ///
    ///  @param[in]  profile Explicit finite reference profile identifier.
    ///  @return     uint32_t.
    ///
    ///  @details    Public class member. Declaration/definition: static uint32_t duration_cycles(uint8_t profile).
    //--------------------------------------------------------------------------------------------------------------------
    static uint32_t duration_cycles(uint8_t profile)
    {
        return profile == 7 ? 8000 : DURATION_CYCLES;
    }
private:
    AP_UTB_RateController _controller;
    AP_UTB_RateController::Gains _gains[3];
    Fault _fault = Fault::NONE;
    uint32_t _calls = 0;
    uint32_t _last_imu = 0;
    uint8_t _gyro = 255;
    bool _configured = false;
    bool _context = false;
    bool _armed = false;
    bool _full_spool = false;
    Vector3f _previous_command;
    AP_UTB_SITLAllocation::Result _feedback;
    bool _allocation_feedback = false;
    bool _delayed = false;
};
#endif // AP_UTB_RATE_SITL_AVAILABLE

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
