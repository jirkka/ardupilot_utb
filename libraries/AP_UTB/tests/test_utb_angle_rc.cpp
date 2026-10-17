//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file test_utb_angle_rc.cpp
/// @brief Skutečné AP RC mapování, znaménka, validity a samonivelace existující UTB kaskády.
/// @author Jiri Kovar
/// @version 2026-10-17
/// @date 2026-10-17
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
#include <AP_UTB/AP_UTB_AngleRC.h>
#include <AP_UTB/AP_UTB_SITLAttitude.h>
#include <AP_UTB/AP_UTB_SITLRate.h>

// Původní HAL singleton vyžadovaný skutečnými AP RC kanály v GTest executable.
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
#if AP_UTB_RATE_SITL_AVAILABLE
class UTBAngleRC : public ::testing::Test
{
protected:
    RC_Channel roll, pitch, yaw, throttle;
    AP_UTB_AngleRC::Config config;
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
        return AP_UTB_AngleRC::read(roll, pitch, yaw, throttle, collective, config, signal);
    }
};
// End Private functions
// End Data type, constant, macro definition
// Public functions

/// @brief Ověřit kontrakt CenterAndTrueAPDeadband.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, CenterAndTrueAPDeadband)
{
    AP_UTB_AngleRC::Result result;
    for (int16_t pwm = 1470; pwm <= 1530; pwm++) {
        roll.set_radio_in(pwm); pitch.set_radio_in(pwm); yaw.set_radio_in(pwm);
        result = read();
        ASSERT_TRUE(result.valid);
        EXPECT_FLOAT_EQ(result.normalized.length(), 0);
        EXPECT_FLOAT_EQ(result.reference.length(), 0);
    }
}

/// @brief Ověřit kontrakt RollPitchYawBothSigns.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, RollPitchYawBothSigns)
{
    AP_UTB_AngleRC::Result result;
    for (int sign : {
             -1, 1
         }) {
        roll.set_radio_in(1500 + sign * 500);
        pitch.set_radio_in(1500);
        yaw.set_radio_in(1500 + sign * 500);
        result = read();
        ASSERT_TRUE(result.valid);
        EXPECT_FLOAT_EQ(result.normalized.x, sign);
        EXPECT_FLOAT_EQ(result.reference.x, radians(5.0f) * sign);
        EXPECT_FLOAT_EQ(result.reference.y, 0);
        EXPECT_FLOAT_EQ(result.reference.z, 0.1f * sign);
        roll.set_radio_in(1500);
        pitch.set_radio_in(1500 + sign * 500);
        result = read();
        EXPECT_FLOAT_EQ(result.reference.y, radians(5.0f) * sign);
    }
}

/// @brief Ověřit kontrakt ReverseUsesActualAPCalibration.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, ReverseUsesActualAPCalibration)
{
    AP_UTB_AngleRC::Result result;
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "REVERSED", 1));
    ASSERT_TRUE(AP_Param::set_object_value(&pitch, RC_Channel::var_info, "REVERSED", 1));
    ASSERT_TRUE(AP_Param::set_object_value(&yaw, RC_Channel::var_info, "REVERSED", 1));
    roll.set_radio_in(2000); pitch.set_radio_in(1000); yaw.set_radio_in(2000);
    result = read();
    ASSERT_TRUE(result.valid);
    EXPECT_FLOAT_EQ(result.normalized.x, -1);
    EXPECT_FLOAT_EQ(result.normalized.y, 1);
    EXPECT_FLOAT_EQ(result.normalized.z, -1);
    EXPECT_NEAR(Vector2f(result.reference.x, result.reference.y).length(), radians(5.0f), 1.0e-7f);
}

/// @brief Ověřit kontrakt AsymmetricEndpointsAndTrim.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, AsymmetricEndpointsAndTrim)
{
    AP_UTB_AngleRC::Result result;
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "MIN", 900));
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "TRIM", 1400));
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "MAX", 2100));
    roll.set_radio_in(1765); // 1430 + 0.5 * (2100 - 1430)
    result = read();
    ASSERT_TRUE(result.valid);
    EXPECT_FLOAT_EQ(result.normalized.x, 0.5f);
    EXPECT_NEAR(result.reference.x, radians(2.5f), 1.0e-7f);
    roll.set_radio_in(1135); // 1370 - 0.5 * (1370 - 900)
    EXPECT_FLOAT_EQ(read().normalized.x, -0.5f);
}

/// @brief Ověřit kontrakt CircularAngleAndIndependentYawLimits.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, CircularAngleAndIndependentYawLimits)
{
    AP_UTB_AngleRC::Result result;
    roll.set_radio_in(2000); pitch.set_radio_in(2000); yaw.set_radio_in(2000);
    result = read();
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.reference.x, radians(5.0f) / sqrtf(2), 1.0e-7f);
    EXPECT_NEAR(Vector2f(result.reference.x, result.reference.y).length(), radians(5.0f), 1.0e-7f);
    EXPECT_FLOAT_EQ(result.reference.z, 0.1f);
    config = {3, 0.05f};
    result = read();
    EXPECT_NEAR(Vector2f(result.reference.x, result.reference.y).length(), radians(3.0f), 1.0e-7f);
    EXPECT_FLOAT_EQ(result.reference.z, 0.05f);
}

/// @brief Ověřit kontrakt ManualThrottleIsIndependentOfAngle.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, ManualThrottleIsIndependentOfAngle)
{
    for (float value : {
             0.0f, 0.1f, 0.35f, 0.75f, 1.0f
         }) {
        roll.set_radio_in(2000);
        ASSERT_TRUE(read(value).valid);
        EXPECT_FLOAT_EQ(read(value).throttle, value);
    }
}

/// @brief Ověřit kontrakt RejectInvalidCalibrationAndChannelType.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, RejectInvalidCalibrationAndChannelType)
{
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "MIN", 1490));
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::CALIBRATION);
    ASSERT_TRUE(AP_Param::set_object_value(&roll, RC_Channel::var_info, "MIN", 1000));
    roll.set_range(1000);
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::CALIBRATION);
    roll.set_angle(4500);
    ASSERT_TRUE(AP_Param::set_object_value(&throttle, RC_Channel::var_info, "DZ", 1000));
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::CALIBRATION);
}

/// @brief Ověřit kontrakt RejectMissingAndOutOfCalibratedRange.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, RejectMissingAndOutOfCalibratedRange)
{
    roll.set_radio_in(0);
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::RANGE);
    roll.set_radio_in(2200);
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::RANGE);
    roll.set_radio_in(1500); throttle.set_radio_in(999);
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::RANGE);
    EXPECT_FALSE(read().valid);
    EXPECT_FLOAT_EQ(read().reference.length(), 0);
}

/// @brief Ověřit kontrakt InvalidFailsafeStaleAndReconnect.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, InvalidFailsafeStaleAndReconnect)
{
    signal.valid = false;
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::SIGNAL);
    signal.valid = true; signal.failsafe = true;
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::SIGNAL);
    signal.failsafe = false; signal.updated_ms = 1; signal.now_ms = 1002;
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::STALE);
    signal.updated_ms = 1002;
    EXPECT_TRUE(read().valid);
}

/// @brief Ověřit kontrakt MonotonicClockWrapAndTimeoutBoundary.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, MonotonicClockWrapAndTimeoutBoundary)
{
    signal = {20, UINT32_MAX - 9, 30, true, false};
    EXPECT_TRUE(read().valid);
    signal.now_ms = 21;
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::STALE);
    signal.timeout_ms = 0;
    EXPECT_FALSE(read().valid);
    signal = {20, 0, 30, true, false};
    EXPECT_TRUE(read().valid);
    signal.now_ms = 31;
    EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::STALE);
}

/// @brief Ověřit kontrakt RejectNaNInfAndUnsupportedEnvelope.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, RejectNaNInfAndUnsupportedEnvelope)
{
    for (float value : {
             NAN, INFINITY, -INFINITY, -0.1f, 1.1f
         }) {
        EXPECT_EQ(read(value).fault, AP_UTB_AngleRC::Fault::THROTTLE);
    }
    for (float value : {
             NAN, INFINITY, -1.0f, 0.0f, 5.01f
         }) {
        config.angle_deg = value;
        EXPECT_EQ(read().fault, AP_UTB_AngleRC::Fault::CONFIG);
    }
    config.angle_deg = 5;
    config.yaw_rate = 0.11f;
    EXPECT_FALSE(read().valid);
}

/// @brief Ověřit kontrakt CenterStickActivelyLevelsBothSignsAndAxes.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, CenterStickActivelyLevelsBothSignsAndAxes)
{
    AP_UTB_SITLAttitude attitude;
    AP_UTB_SITLRate rate;
    AP_UTB_RateController::Gains gains[3];
    AP_UTB_SITLRate::Inputs input;
    AP_UTB_SITLAttitude::Result outer;
    AP_UTB_RateController::Result inner;
    AP_UTB_AngleRC::Result pilot;
    Vector3f euler;
    AP_UTB_AngleRC::gains(gains);
    for (uint8_t axis = 0; axis < 2; axis++) {
        for (int sign : {
                 -1, 1
             }) {
            ASSERT_TRUE(attitude.configure({3, 3, 0.3f}));
            ASSERT_TRUE(rate.configure(gains));
            euler = {}; euler[axis] = sign * radians(5.0f);
            input = {};
            input.state.attitude_body_to_ned.from_euler(euler.x, euler.y, 0);
            input.state.attitude_valid = input.state.rates_valid = true;
            input.state.sampled_us = input.state.imu_updated_us = input.now_us = 100000;
            input.dt = input.nominal_dt = 0.0025f;
            input.frame_class = 1; input.frame_type = 18;
            pilot = read();
            outer = attitude.update(input.state, pilot.reference, input.dt, input.now_us);
            ASSERT_TRUE(outer.valid);
            EXPECT_NEAR(outer.rates[axis], -sign * radians(5.0f) * 3, 1.0e-6f);
            input.target = outer.rates;
            EXPECT_TRUE(rate.step(input, true).priming);
            input.state.sampled_us = input.state.imu_updated_us = input.now_us = 102500;
            inner = rate.step(input, true);
            ASSERT_TRUE(inner.valid);
            EXPECT_LT(inner.command[axis] * sign, 0);
            rate.reset();
            ASSERT_TRUE(rate.configure(gains));
            EXPECT_TRUE(rate.step(input, true).priming);
        }
    }
}

/// @brief Ověřit kontrakt BodyWorldHeadingAndRateClamp.
/// @param[in] None
/// @return None, nesoulad zachytí GTest assertions.
/// @details Skutečná AP normalizace a existující UTB controller output; nikoli pouhý crash test.
TEST_F(UTBAngleRC, BodyWorldHeadingAndRateClamp)
{
    AP_UTB_SITLAttitude attitude;
    AP_UTB_State state;
    AP_UTB_SITLAttitude::Result result;
    AP_UTB_AngleRC::Result pilot;
    roll.set_radio_in(2000); yaw.set_radio_in(2000);
    pilot = read();
    state.attitude_valid = state.rates_valid = true;
    state.sampled_us = state.imu_updated_us = 100000;
    for (float heading : {
             -3.0f, -1.5f, 0.0f, 1.5f, 3.0f
         }) {
        ASSERT_TRUE(attitude.configure({3, 3, 0.3f}));
        state.attitude_body_to_ned.from_euler(-radians(5.0f), 0, heading);
        result = attitude.update(state, pilot.reference, 0.0025f, 100000);
        ASSERT_TRUE(result.valid);
        EXPECT_NEAR(result.rates.x, 0.3f, 1.0e-6f);
        EXPECT_NEAR(result.rates.y, 0, 1.0e-6f);
        EXPECT_FLOAT_EQ(result.rates.z, 0.1f);
        EXPECT_TRUE(result.limited);
    }
}
#endif // AP_UTB_RATE_SITL_AVAILABLE
AP_GTEST_MAIN()

// End Public functions
// Static data declarations
// None
// End Static data declarations
// Private function prototypes
// None
// End Private function prototypes
