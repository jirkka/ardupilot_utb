//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB_SITLAttitude.h
///  @brief      Vlastní reduced rotation P: error z RᵀRd·ez, common XY rate limiter, explicitní yaw body rate.
///  @author     Jiri Kovar
///  @version    2026-10-18
///  @date       2026-10-18
//--------------------------------------------------------------------------------------------------------------------
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "AP_UTB_State.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// Reduced rotation P controller. No heading integrator or actuator dependencies.
/// @brief UTB component; Vlastní reduced rotation P: error z RᵀRd·ez, common XY rate limiter, explicitní yaw
///        body rate.
class AP_UTB_SITLAttitude
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    struct Config {
        // Explicit pilot-only expansion; automated mode41 keeps false.
        bool extended = false;
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // End Data type, constant, macro definition
        //====================================================================================================================
        // Public functions
        //--------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Config.
        ///
        ///  @param[in]  roll Roll.
        ///  @param[in]  pitch Pitch.
        ///  @param[in]  maximum Maximum.
        ///  @return     None (constructor/destructor).
        ///
        ///  @details    Public class member. Declaration/definition: Config(float roll = 2, float pitch = 2, float maximum =
        ///              0.3f).
        //--------------------------------------------------------------------------------------------------------------------
        Config(float roll = 2, float pitch = 2, float maximum = 0.3f) :
            roll_p(roll), pitch_p(pitch), rate_max(maximum) {}
        float roll_p;
        float pitch_p;
        float rate_max;
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Compare every configuration field without a tolerance deadband.
        ///
        ///  @param[in]  other Other value used for comparison.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool operator==(const Config &other).
        //--------------------------------------------------------------------------------------------------------------------
        bool operator==(const Config &other) const;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Public functions
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
    enum class Fault : uint8_t { NONE, CONFIG, STATE, DT, REFERENCE, ENVELOPE, IMU };
    struct Result {
        Vector3f euler;
        Vector3f error;
        Vector3f rates;
        bool limited = false;
        bool valid = false;
    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Validate and store configuration; reset dependent controller history when required.
    ///
    ///  @param[in]  config Config.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool configure(const Config &config).
    //--------------------------------------------------------------------------------------------------------------------
    bool configure(const Config &config);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Compare the runtime configuration with the frozen experiment configuration.
    ///
    ///  @param[in]  config Config.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool configuration_matches(const Config &config).
    //--------------------------------------------------------------------------------------------------------------------
    bool configuration_matches(const Config &config) const
    {
        return _configured && config == _config;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Update.
    ///
    ///  @param[in]  state Sampled attitude, gyro and validity metadata.
    ///  @param[in]  reference Reference.
    ///  @param[in]  dt Sample interval in seconds.
    ///  @param[in]  now_us Monotonic validation timestamp in microseconds.
    ///  @return     Result.
    ///
    ///  @details    Roll/pitch reduced-rotation P produces body rates; yaw input is body rate, not heading hold. SITL only.
    //--------------------------------------------------------------------------------------------------------------------
    Result update(const AP_UTB_State &state, const Vector3f &reference, float dt, uint32_t now_us);
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
    ///  @brief      Reference.
    ///
    ///  @param[in]  profile Explicit finite reference profile identifier.
    ///  @param[in]  cycle Reference cycle index.
    ///  @param[in]  angle_rad Reference tilt amplitude in radians.
    ///  @param[in]  yaw_rate Desired body yaw rate in rad/s.
    ///  @return     Vector3f.
    ///
    ///  @details    Public class member. Declaration/definition: static Vector3f reference(uint8_t profile, uint32_t cycle,
    ///              float angle_rad, float yaw_rate).
    //--------------------------------------------------------------------------------------------------------------------
    static Vector3f reference(uint8_t profile, uint32_t cycle, float angle_rad, float yaw_rate);
    static constexpr uint32_t DURATION_CYCLES = 5600;
private:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Public functions
    //====================================================================================================================
    // Private function prototypes
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Latch the first fault and clear controller history to prevent continued control.
    ///
    ///  @param[in]  fault Fault.
    ///  @return     None.
    ///
    ///  @details    Non-public class member. Declaration/definition: void invalidate(Fault fault).
    //--------------------------------------------------------------------------------------------------------------------
    void invalidate(Fault fault);
    Config _config;
    Fault _fault = Fault::NONE;
    uint32_t _last_imu = 0;
    uint32_t _calls = 0;
    uint8_t _gyro = 255;
    bool _configured = false;
    bool _context = false;
};
#endif // AP_UTB_RATE_SITL_AVAILABLE

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private function prototypes
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
