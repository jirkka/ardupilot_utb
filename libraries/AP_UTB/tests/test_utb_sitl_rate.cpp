//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       test_utb_sitl_rate.cpp
///  @brief      Analytické/lifecycle regrese příslušné komponenty; 9 deklarovaných TEST případů.
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
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include <AP_gtest.h>
#include <AP_UTB/AP_UTB_SITLRate.h>
#include <limits>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
#if AP_UTB_RATE_SITL_AVAILABLE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Inputs.
///
///  @param[in]  stamp Stamp.
///  @return     AP_UTB_SITLRate::Inputs.
///
///  @details    File-local helper (internal linkage). Declaration/definition: static AP_UTB_SITLRate::Inputs inputs(uint32_t
///              stamp = 100000).
//--------------------------------------------------------------------------------------------------------------------
static AP_UTB_SITLRate::Inputs inputs(uint32_t stamp = 100000)
{
    AP_UTB_SITLRate::Inputs in;
    in.state.attitude_body_to_ned = Quaternion(1, 0, 0, 0);
    in.state.attitude_valid = in.state.rates_valid = true;
    in.state.sampled_us = in.state.imu_updated_us = stamp;
    in.state.primary_gyro = 0;
    in.now_us = stamp + 10;
    in.dt = in.nominal_dt = 0.0025f;
    in.frame_class = 1; in.frame_type = 18;
    return in;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
static const AP_UTB_RateController::Gains gains[3] = {
    {0.31f, 0.23f, 0.004f, 0.19f, 20}, {0.31f, 0.23f, 0.004f, 0.19f, 20}, {0.31f, 0.23f, 0.004f, 0.19f, 20}
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify default and explicit configuration.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, DefaultAndExplicitConfiguration).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, DefaultAndExplicitConfiguration)
{
    AP_UTB_SITLRate rate;
    EXPECT_FALSE(rate.step(inputs(), true).valid);
    EXPECT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::CONFIG);
    ASSERT_TRUE(rate.configure(gains));
    EXPECT_TRUE(rate.step(inputs(), true).priming);
    auto in = inputs(102500); in.target.x = 0.15f;
    const auto out = rate.step(in, true);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.axis[0].p, 0.0465f, 1.0e-7f);
    EXPECT_GT(out.axis[0].i, 0);
    EXPECT_EQ(rate.calls(), 2U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify apbaseline does not execute utbpid.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, APBaselineDoesNotExecuteUTBPID).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, APBaselineDoesNotExecuteUTBPID)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
    EXPECT_TRUE(rate.step(inputs(), false).valid);
    EXPECT_TRUE(rate.step(inputs(102500), false).valid);
    EXPECT_EQ(rate.calls(), 0U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify faults latch and reset requires configuration.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, FaultsLatchAndResetRequiresConfiguration).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, FaultsLatchAndResetRequiresConfiguration)
{
    for (uint8_t fault = 1; fault <= 8; fault++) {
        AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
        rate.step(inputs(), true);
        auto in = inputs(102500);
        switch (fault) {
        case 1: in.state.rates_valid = false; break;
        case 2: in.dt = NAN; break;
        case 3: in.frame_type = 12; break;
        case 4: in.fast_rate = true; break;
        case 5: rate.invalidate(AP_UTB_SITLRate::Fault::CONFIG); break;
        case 6: in.target.x = 0.3f; in.state.rates_rads.x = -1.9f; break;
        case 7: in.state.rates_rads.x = 2.01f; break;
        case 8: in.state.imu_updated_us = 100000; break;
        }
        EXPECT_FALSE(rate.step(in, true).valid) << unsigned(fault);
        EXPECT_NE(rate.fault(), AP_UTB_SITLRate::Fault::NONE) << unsigned(fault);
        EXPECT_FALSE(rate.step(inputs(105000), true).valid);
        rate.reset(); EXPECT_FALSE(rate.step(inputs(107500), true).valid);
        ASSERT_TRUE(rate.configure(gains)); EXPECT_TRUE(rate.step(inputs(110000), true).priming);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify allocator limits only fault after armed full spool.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL,
///              AllocatorLimitsOnlyFaultAfterArmedFullSpool).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, AllocatorLimitsOnlyFaultAfterArmedFullSpool)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
    auto in = inputs(); in.allocation_limited = true;
    EXPECT_TRUE(rate.step(in, true).priming);
    in = inputs(102500); in.armed = in.full_spool = true; in.allocation_limited = true;
    EXPECT_TRUE(rate.step(in, true).priming);
    in = inputs(105000); in.armed = in.full_spool = in.allocation_limited = true;
    EXPECT_FALSE(rate.step(in, true).valid);
    EXPECT_EQ(rate.fault(), AP_UTB_SITLRate::Fault::SATURATION);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify arming and spool transitions reprime integral and d.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL,
///              ArmingAndSpoolTransitionsReprimeIntegralAndD).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, ArmingAndSpoolTransitionsReprimeIntegralAndD)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
    rate.step(inputs(), true);
    auto in = inputs(102500); in.target.x = 0.15f; ASSERT_TRUE(rate.step(in, true).valid);
    in = inputs(105000); in.armed = true;
    const auto out = rate.step(in, true);
    EXPECT_TRUE(out.priming); EXPECT_FLOAT_EQ(out.axis[0].i, 0); EXPECT_FLOAT_EQ(out.axis[0].d, 0);
    in = inputs(107500); in.armed = in.full_spool = true; EXPECT_TRUE(rate.step(in, true).priming);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify stale state and clock wrap.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, StaleStateAndClockWrap).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, StaleStateAndClockWrap)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
    auto in = inputs(UINT32_MAX - 1000); ASSERT_TRUE(rate.step(in, true).priming);
    in = inputs(1499); EXPECT_TRUE(rate.step(in, true).valid);
    in = inputs(3999); in.now_us += 10001; EXPECT_FALSE(rate.step(in, true).valid);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify nonfinite inputs never produce valid output.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, NonfiniteInputsNeverProduceValidOutput).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, NonfiniteInputsNeverProduceValidOutput)
{
    for (bool target : {
             false, true
         }) {
        AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
        auto in = inputs();
        if (target) {
            in.target.x = std::numeric_limits<float>::infinity();
        } else {
            in.state.rates_rads.x = NAN;
        }
        EXPECT_FALSE(rate.step(in, true).valid);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify finite reference profiles and balanced signs.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, FiniteReferenceProfilesAndBalancedSigns).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, FiniteReferenceProfilesAndBalancedSigns)
{
    for (uint8_t profile = 0; profile <= 7; profile++) {
        Vector3f total;
        const uint32_t duration = AP_UTB_SITLRate::duration_cycles(profile);
        EXPECT_EQ(duration, profile == 7 ? 8000U : 3200U);
        for (uint32_t cycle = 0; cycle < duration; cycle++) {
            const auto r = AP_UTB_SITLRate::reference(profile, cycle, 0.15f);
            EXPECT_FALSE(r.is_nan() || r.is_inf()); EXPECT_LE(r.length(), 0.6f); total += r;
        }
        if (profile <= 4 || profile == 7) {
            EXPECT_LT(total.length(), 1.0e-4f);
        }
        EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(profile, duration, 0.15f).length(), 0);
    }
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 399, 0.15f).z, 0);
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 400, 0.15f).z, 0.15f);
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 3599, 0.15f).z, 0.15f);
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 3600, 0.15f).z, 0);
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 4000, 0.15f).z, -0.15f);
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 7199, 0.15f).z, -0.15f);
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(7, 7200, 0.15f).z, 0);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify bounded transport delay and reset.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBSITL, BoundedTransportDelayAndReset).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBSITL, BoundedTransportDelayAndReset)
{
    AP_UTB_SITLRate rate; ASSERT_TRUE(rate.configure(gains));
    const Vector3f command(0.1f, -0.2f, 0.05f);
    EXPECT_FLOAT_EQ(rate.transport(command, true, true).length(), 0);
    EXPECT_EQ(rate.transport({}, true, true), command);
    EXPECT_FLOAT_EQ(rate.transport(command, true, false).length(), 0);
    EXPECT_FLOAT_EQ(rate.transport(command, true, true).length(), 0);
    rate.invalidate(AP_UTB_SITLRate::Fault::STATE);
    EXPECT_FLOAT_EQ(rate.transport(command, false, true).length(), 0);
}
#endif
AP_GTEST_MAIN()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private function prototypes
