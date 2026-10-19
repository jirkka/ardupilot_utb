//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file test_utb_acro_rc.cpp
/// @brief Skutečné AP RC mapování, znaménka, validity a samonivelace existující UTB kaskády.
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
#include <AP_gtest.h>
#include <functional>
#include <AP_UTB/AP_UTB_AcroRC.h>
#include <AP_UTB/AP_UTB_PilotRates.h>
#include <AP_UTB/AP_UTB.h>
#include <AP_UTB/AP_UTB_SITLAttitude.h>
#include <AP_UTB/AP_UTB_SITLRate.h>

// Původní HAL singleton vyžadovaný skutečnými AP RC kanály v GTest executable.
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
#if AP_UTB_RATE_SITL_AVAILABLE
class UTBAcroRC : public ::testing::Test
{
protected:
    RC_Channel roll, pitch, yaw, throttle;
    Vector3f config{0.15f, 0.15f, 0.05f};
    AP_UTB_AngleRC::Signal signal{1000, 999, 1000, true, false};

    // Private functions
    /// @brief Připravit skutečné AP kanály s definovanou kalibrací.
    /// @param[in] None
    /// @return None.
    /// @details Test fixture, bez hardwarového RC vstupu.
    void SetUp() override
    {
        for (RC_Channel &channel : {
                 std::ref(roll), std::ref(pitch), std::ref(yaw), std::ref(throttle)
             }) {
            ASSERT_TRUE(AP_Param::set_object_value(&channel, RC_Channel::var_info, "MIN", 1000));
            ASSERT_TRUE(AP_Param::set_object_value(&channel, RC_Channel::var_info, "MAX", 2000));
            ASSERT_TRUE(AP_Param::set_object_value(&channel, RC_Channel::var_info, "TRIM", 1500));
            ASSERT_TRUE(AP_Param::set_object_value(&channel, RC_Channel::var_info, "DZ", 30));
            channel.set_angle(4500);
            channel.set_radio_in(1500);
        }
        throttle.set_range(1000);
        throttle.set_radio_in(1000);
    }

    /// @brief Předat skutečné kalibrované AP kanály do pilotního adaptéru.
    /// @param[in] collective Explicitní výstup AP manual throttle mapy.
    /// @return Pilotní výsledek včetně důvodu odmítnutí.
    /// @details Parametr throttle ověřují navíc skutečné SITL experimenty.
    AP_UTB_AngleRC::Result read(float collective = 0.35f)
    {
        return AP_UTB_AcroRC::read(roll, pitch, yaw, throttle, collective, config, signal);
    }
};
// End Private functions
// End Data type, constant, macro definition
// Public functions

/// @brief Ověřit ZeroRateAtCenter skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, ZeroRateAtCenter)
{
    const auto result = read();
    ASSERT_TRUE(result.valid);
    EXPECT_FLOAT_EQ(result.reference.length(), 0);
}

/// @brief Ověřit BothDirectionsAndAxes skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, BothDirectionsAndAxes)
{
    AP_UTB_AngleRC::Result result;
    for (int sign : {
             -1, 1
         }) {
        roll.set_radio_in(1500 + sign * 500); pitch.set_radio_in(1500 - sign * 500);
        yaw.set_radio_in(1500 + sign * 500);
        result = read(); ASSERT_TRUE(result.valid);
        EXPECT_FLOAT_EQ(result.reference.x, sign * 0.15f);
        EXPECT_FLOAT_EQ(result.reference.y, -sign * 0.15f);
        EXPECT_FLOAT_EQ(result.reference.z, sign * 0.05f);
    }
}

/// @brief Ověřit ActualDeadband skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, ActualDeadband)
{
    for (int16_t pwm = 1470; pwm <= 1530; pwm++) {
        roll.set_radio_in(pwm); pitch.set_radio_in(pwm); yaw.set_radio_in(pwm);
        EXPECT_FLOAT_EQ(read().reference.length(), 0);
    }
}

/// @brief Ověřit ActualReverse skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, ActualReverse)
{
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "REVERSED", 1));
    roll.set_radio_in(2000);
    EXPECT_FLOAT_EQ(read().reference.x, -0.15f);
}

/// @brief Ověřit IndependentLimitsNoCircularClamp skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, IndependentLimitsNoCircularClamp)
{
    roll.set_radio_in(2000); pitch.set_radio_in(2000); yaw.set_radio_in(1000);
    config = {0.1f, 0.05f, 0.02f};
    EXPECT_FLOAT_EQ(read().reference.x, 0.1f);
    EXPECT_FLOAT_EQ(read().reference.y, 0.05f);
    EXPECT_FLOAT_EQ(read().reference.z, -0.02f);
}

/// @brief Ověřit HardCapsAndNonfinite skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, HardCapsAndNonfinite)
{
    Vector3f limits;
    for (uint8_t axis = 0; axis < 3; axis++) {
        for (float value : {
                 0.0f, -0.1f, 0.151f, NAN, INFINITY
             }) {
            limits = config; limits[axis] = value;
            EXPECT_FALSE(AP_UTB_AcroRC::valid_config(limits));
        }
    }
    config.z = 0.051f;
    EXPECT_FALSE(read().valid);
}

/// @brief Ověřit InvalidAndStaleInput skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, InvalidAndStaleInput)
{
    signal.valid = false; EXPECT_FALSE(read().valid);
    signal.valid = true; signal.failsafe = true; EXPECT_FALSE(read().valid);
    signal.failsafe = false; signal.now_ms = 3000; EXPECT_FALSE(read().valid);
    signal.now_ms = 1000; EXPECT_TRUE(read().valid);
    roll.set_radio_in(2100); EXPECT_FALSE(read().valid);
}

/// @brief Ověřit InvalidCalibration skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, InvalidCalibration)
{
    ASSERT_TRUE(AP_Param::set_object_value(&pitch, RC_Channel::var_info, "DZ", 600));
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::CALIBRATION);
}

/// @brief Ověřit ManualThrottleUnchanged skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, ManualThrottleUnchanged)
{
    for (float value : {
             0.0f, 0.35f, 1.0f
         }) {
        EXPECT_FLOAT_EQ(read(value).throttle, value);
    }
    EXPECT_FALSE(read(NAN).valid);
    EXPECT_FALSE(read(1.01f).valid);
}

/// @brief Ověřit FailsafeContract skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, FailsafeContract)
{
    EXPECT_TRUE(AP_UTB_AcroRC::failsafe_policy(1, 1, 0));
    EXPECT_FALSE(AP_UTB_AcroRC::failsafe_policy(1, 1, 16));
    EXPECT_FALSE(AP_UTB_AcroRC::failsafe_policy(0, 1, 0));
    EXPECT_FALSE(AP_UTB_AcroRC::failsafe_policy(1, 0, 0));
}

/// @brief Ověřit ZeroRateDoesNotLevelAndReentryPrimes skutečným AP RC/UTB adaptérem.
/// @param[in] None
/// @return None; nesoulad zachytí GTest.
/// @details Izolovaný test bez fyzického motorového výstupu.
TEST_F(UTBAcroRC, ZeroRateDoesNotLevelAndReentryPrimes)
{
    AP_UTB_SITLRate rate;
    AP_UTB_RateController::Gains gains[3];
    AP_UTB_SITLRate::Inputs input;
    AP_UTB_RateController::Result result;
    AP_UTB_AngleRC::gains(gains);
    input.state.attitude_body_to_ned.from_euler(radians(5.0f), -radians(5.0f), 0);
    input.state.attitude_valid = input.state.rates_valid = true;
    input.state.sampled_us = input.state.imu_updated_us = input.now_us = 100000;
    input.dt = input.nominal_dt = 0.0025f;
    input.frame_class = 1; input.frame_type = 18;
    input.target = read().reference;
    ASSERT_TRUE(rate.configure(gains));
    EXPECT_TRUE(rate.step(input, true).priming);
    input.state.sampled_us = input.state.imu_updated_us = input.now_us = 102500;
    result = rate.step(input, true); ASSERT_TRUE(result.valid);
    EXPECT_FLOAT_EQ(result.command.length(), 0);
    // Deliberately accumulate nonzero I, then configure a new mode context.
    input.target.x = 0.045f;
    for (uint32_t i = 0; i < 100; i++) {
        input.now_us += 2500; input.state.sampled_us = input.state.imu_updated_us = input.now_us;
        result = rate.step(input, true);
    }
    EXPECT_GT(result.axis[0].i, 0);
    rate.reset(); ASSERT_TRUE(rate.configure(gains));
    input.now_us += 2500; input.state.sampled_us = input.state.imu_updated_us = input.now_us;
    result = rate.step(input, true); EXPECT_TRUE(result.priming);
    EXPECT_FLOAT_EQ(result.axis[0].i, 0); EXPECT_FLOAT_EQ(result.axis[0].d, 0);
    EXPECT_FLOAT_EQ(result.command.length(), 0);
}

/// @brief Povolit neaplikovaný LAND raw kontext pouze při DISARMED, bez oslabení aktivních mezí.
/// @param[in] None
/// @return None; assertions ověřují skutečnou vstupní predicate obou pilotních modes.
/// @details AP motorový výstup a nový RC throttle se tímto helperem nenastavují.
TEST_F(UTBAcroRC, NativeLandDisarmedContextAndArmedBounds)
{
    EXPECT_TRUE(AP_UTB_AcroRC::entry_throttle_valid(-0.000001f, false));
    EXPECT_FALSE(AP_UTB_AcroRC::entry_throttle_valid(-0.000001f, true));
    EXPECT_FALSE(AP_UTB_AcroRC::entry_throttle_valid(1.001f, true));
    EXPECT_TRUE(AP_UTB_AcroRC::entry_throttle_valid(0, true));
    EXPECT_TRUE(AP_UTB_AcroRC::entry_throttle_valid(1, true));
    EXPECT_FALSE(AP_UTB_AcroRC::entry_throttle_valid(NAN, false));
    EXPECT_FALSE(AP_UTB_AcroRC::entry_throttle_valid(NAN, true));
    EXPECT_FALSE(AP_UTB_AcroRC::entry_throttle_valid(INFINITY, false));
    EXPECT_FALSE(AP_UTB_AcroRC::entry_throttle_valid(-INFINITY, false));
}
/// @brief Ověřit pilotní meze a původní validaci D filtru bez změny PID matematiky.
/// @param[in] None
/// @return None; chybná konfigurace se odmítá.
/// @details Jednotkový test pouze SITL parametrického kontraktu.
TEST_F(UTBAcroRC, PilotGainBounds)
{
    // Kopie gains pro nezávislé odmítnutí jednotlivých vadných hodnot.
    AP_UTB_RateController::Gains gains[3];

    AP_UTB_AngleRC::gains(gains);
    ASSERT_TRUE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    gains[0].p = NAN;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    AP_UTB_AngleRC::gains(gains); gains[1].p = 0.301f;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    AP_UTB_AngleRC::gains(gains); gains[0].i = 0.601f;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    AP_UTB_AngleRC::gains(gains); gains[1].d = 0.0041f;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    AP_UTB_AngleRC::gains(gains); gains[0].imax = 0.191f;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
}

/// @brief Ověřit cutoff meze při pevné 400Hz nominální periodě.
/// @param[in] None
/// @return None.
/// @details Při D>0 zůstává pozitivní stabilní filtr povinný.
TEST_F(UTBAcroRC, PilotFilterBounds)
{
    // Samostatná experimentální konfigurace.
    AP_UTB_RateController::Gains gains[3];

    AP_UTB_AngleRC::gains(gains); gains[0].d_hz = 40;
    EXPECT_TRUE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    gains[0].d_hz = 40.1f;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    gains[0].d_hz = 0;
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
    gains[0].d = 0;
    EXPECT_TRUE(AP_UTB_AngleRC::pilot_gains_valid(gains));
}

/// @brief Prokázat skutečný AP_Param zápis bez ovlivnění starého mode41/SHADOW gains.
/// @param[in] None
/// @return None.
/// @details Oba směry izolace se ověřují nad skutečnou var_info tabulkou.
TEST_F(UTBAcroRC, PilotParameterIsolation)
{
    // Manager a snapshots nezávislých parametrických větví.
    AP_UTB manager;
    AP_UTB_RateController::Gains legacy_before[3], legacy_after[3], pilot_before[3], pilot_after[3];
    uint8_t axis;

    manager.sitl_gains(legacy_before);
    ASSERT_TRUE(AP_Param::set_object_value(&manager, AP_UTB::var_info, "P_RLL_P", 0.12f));
    manager.pilot_gains(pilot_before);
    EXPECT_FLOAT_EQ(pilot_before[0].p, 0.12f);
    manager.sitl_gains(legacy_after);
    for (axis = 0; axis < 3; axis++) {
        EXPECT_TRUE(legacy_before[axis] == legacy_after[axis]);
    }
    ASSERT_TRUE(AP_Param::set_object_value(&manager, AP_UTB::var_info, "RAT_RLL_P", 0.27f));
    manager.pilot_gains(pilot_after);
    for (axis = 0; axis < 3; axis++) {
        EXPECT_TRUE(pilot_before[axis] == pilot_after[axis]);
    }
}

/// @brief Neplatný uložený AP parametr nesmí projít do pilotního PID.
/// @param[in] None
/// @return None.
/// @details Číselné UI rozsahy nenahrazují vlastní mode-entry validaci.
TEST_F(UTBAcroRC, InvalidStoredPilotGain)
{
    // Skutečný manager a jeho načtená pilotní konfigurace.
    AP_UTB manager;
    AP_UTB_RateController::Gains gains[3];

    ASSERT_TRUE(AP_Param::set_object_value(&manager, AP_UTB::var_info, "P_PIT_D", 0.01f));
    manager.pilot_gains(gains);
    EXPECT_FALSE(AP_UTB_AngleRC::pilot_gains_valid(gains));
}


/// @brief Check Actual endpoints, signs, center slope and expo independently.
/// @param[in] None
/// @return None; GTest reports discrepancies.
/// @details No plant capability is inferred from the reference curve.
TEST_F(UTBAcroRC, ActualRatesCurve)
{
    // Sign and normalized stick sweep; endpoint independent of expo.
    int sign;
    float x;
    float expected;
    for (sign = -1; sign <= 1; sign += 2) {
        EXPECT_NEAR(AP_UTB_PilotRates::actual(sign, 160, 670, 0.3f), sign * radians(670.0f), 2e-6);
        for (x = 0; x <= 1; x += 0.01f) {
            expected = 160 * x + 510 * (0.7f * x * x + 0.3f * powf(x, 6));
            EXPECT_NEAR(AP_UTB_PilotRates::actual(sign * x, 160, 670, 0.3f), sign * radians(expected), 3e-6);
        }
    }
    EXPECT_NEAR(degrees(AP_UTB_PilotRates::actual(1e-5f, 160, 670, 0.3f)) / 1e-5f, 160, 0.01);
    EXPECT_FLOAT_EQ(AP_UTB_PilotRates::actual(0, 160, 670, 1), 0);
    EXPECT_NEAR(degrees(AP_UTB_PilotRates::actual(0.5f, 160, 670, 0.3f)), 171.640625f, 2e-5);
}
/// @brief Reject invalid Actual domains and frozen-profile edits.
/// @param[in] None
/// @return None; GTest reports discrepancies.
/// @details Legacy fields remain opt-in and no gains are modified.
TEST_F(UTBAcroRC, ActualConfiguration)
{
    // Independent fixed snapshots and parameter manager defaults.
    AP_UTB_PilotRates::Config a;
    AP_UTB_PilotRates::Config b;
    EXPECT_TRUE(a.valid());
    a.type = b.type = 1;
    EXPECT_TRUE(a.valid());
    EXPECT_TRUE(a == b);
    b.expo.x += 0.001f;
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(isnan(AP_UTB_PilotRates::actual(NAN, 160, 670, 0.3f)));
    EXPECT_TRUE(isnan(AP_UTB_PilotRates::actual(1.01f, 160, 670, 0.3f)));
    EXPECT_TRUE(isnan(AP_UTB_PilotRates::actual(0, 671, 670, 0.3f)));
    EXPECT_TRUE(isnan(AP_UTB_PilotRates::actual(0, 160, 671, 0.3f)));
    EXPECT_TRUE(isnan(AP_UTB_PilotRates::actual(0, 160, 670, -0.1f)));
    a.angle = 61;
    EXPECT_FALSE(a.valid());
}
/// @brief Keep legacy ANGLE bounds and validate only explicit expanded configuration.
/// @param[in] None
/// @return None; GTest reports discrepancies.
/// @details Pure input-domain test, not a 60 degree flight qualification.
TEST_F(UTBAcroRC, ExtendedAngleOptIn)
{
    // Shared RC configuration, expanded only by an explicit bit.
    AP_UTB_AngleRC::Config cfg{60, 0.1f};
    EXPECT_FALSE(AP_UTB_AngleRC::valid_config(cfg));
    cfg.extended = true;
    EXPECT_TRUE(AP_UTB_AngleRC::valid_config(cfg));
    cfg.angle_deg = 61;
    EXPECT_FALSE(AP_UTB_AngleRC::valid_config(cfg));
}

/// @brief Verify expanded ANGLE reference, self-level sign and retained attitude guard.
/// @param[in] None
/// @return None; GTest reports mismatches.
/// @details Kinematic validation only; no claim that the SITL airframe can hold 60 degrees.
TEST_F(UTBAcroRC, ExtendedAngleKinematicsAndRateLimit)
{
    // Explicit healthy identity sample and frozen experimental outer-loop configuration.
    AP_UTB_State sample;
    AP_UTB_SITLAttitude attitude;
    AP_UTB_SITLAttitude::Config cfg{3.25f, 3.25f, radians(90.0f)};
    AP_UTB_SITLAttitude::Result result;
    sample.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
    sample.attitude_valid = sample.rates_valid = true;
    sample.sampled_us = sample.imu_updated_us = 100000;
    sample.primary_gyro = 0;
    EXPECT_FALSE(attitude.configure(cfg));
    cfg.extended = true;
    ASSERT_TRUE(attitude.configure(cfg));
    result = attitude.update(sample, {radians(60.0f), 0, 0}, .0025f, 100010);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.limited);
    EXPECT_NEAR(result.rates.x, radians(90.0f), 1e-6);
    sample.attitude_body_to_ned.from_euler(radians(45.0f), 0, 0);
    sample.sampled_us = sample.imu_updated_us = 102500;
    result = attitude.update(sample, {}, .0025f, 102510);
    ASSERT_TRUE(result.valid);
    EXPECT_LT(result.rates.x, 0);
    sample.attitude_body_to_ned.from_euler(radians(76.0f), 0, 0);
    sample.sampled_us = sample.imu_updated_us = 105000;
    EXPECT_FALSE(attitude.update(sample, {}, .0025f, 105010).valid);
    EXPECT_EQ(attitude.fault(), AP_UTB_SITLAttitude::Fault::ENVELOPE);
}

#endif // AP_UTB_RATE_SITL_AVAILABLE
AP_GTEST_MAIN()
// End Public functions

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
