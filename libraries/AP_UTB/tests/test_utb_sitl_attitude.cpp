//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       test_utb_sitl_attitude.cpp
///  @brief      Analytické/lifecycle regrese příslušné komponenty; 14 deklarovaných TEST případů.
///  @author     Jiri Kovar
///  @version    2026-10-16
///  @date       2026-10-16
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
#include <AP_UTB/AP_UTB_SITLAttitude.h>
#include <AP_UTB/AP_UTB_SITLRate.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
#if AP_UTB_RATE_SITL_AVAILABLE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      State.
///
///  @param[in]  r R.
///  @param[in]  p P.
///  @param[in]  y Y.
///  @param[in]  stamp Stamp.
///  @return     AP_UTB_State.
///
///  @details    File-local helper (internal linkage). Declaration/definition: static AP_UTB_State state(float r = 0, float p
///              = 0, float y = 0, uint32_t stamp = 100000).
//--------------------------------------------------------------------------------------------------------------------
static AP_UTB_State state(float r = 0, float p = 0, float y = 0, uint32_t stamp = 100000)
{
    AP_UTB_State s;
    s.attitude_body_to_ned.from_euler(r, p, y);
    s.attitude_valid = s.rates_valid = true;
    s.sampled_us = s.imu_updated_us = stamp;
    s.primary_gyro = 0;
    return s;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify default off and reset.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, DefaultOffAndReset).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, DefaultOffAndReset)
{
    AP_UTB_SITLAttitude c;
    EXPECT_FALSE(c.update(state(), {}, .0025f, 100010).valid);
    ASSERT_TRUE(c.configure({}));
    EXPECT_TRUE(c.update(state(), {}, .0025f, 100010).valid);
    EXPECT_EQ(c.calls(), 1U);
    c.reset(); EXPECT_EQ(c.calls(), 0U);
    EXPECT_FALSE(c.update(state(), {}, .0025f, 100010).valid);
    ASSERT_TRUE(c.configure({}));
    EXPECT_TRUE(c.update(state(), {}, .0025f, 100010).valid);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify zero and yaw rate without heading hold.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, ZeroAndYawRateWithoutHeadingHold).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, ZeroAndYawRateWithoutHeadingHold)
{
    for (float yaw : {
             0.0f, 1.2f, -3.14f, 3.14f
         }) {
        AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
        auto out = c.update(state(0, 0, yaw), {0, 0, .05f}, .0025f, 100010);
        ASSERT_TRUE(out.valid);
        EXPECT_NEAR(out.rates.x, 0, 1.0e-7f); EXPECT_NEAR(out.rates.y, 0, 1.0e-7f);
        EXPECT_FLOAT_EQ(out.rates.z, .05f);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify signed analytic single axis and return to level.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude,
///              SignedAnalyticSingleAxisAndReturnToLevel).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, SignedAnalyticSingleAxisAndReturnToLevel)
{
    for (uint8_t axis = 0; axis < 2; axis++) {
        for (float sign : {
                 -1.0f, 1.0f
             }) {
            for (float yaw : {
                     0.0f, 1.1f, -3.14159f, 3.14159f
                 }) {
                for (bool returning : {
                         false, true
                     }) {
                    AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({2, 3, .3f}));
                    Vector3f angle; angle[axis] = sign * radians(3.0f);
                    const auto s = state(returning ? angle.x : 0, returning ? angle.y : 0, yaw);
                    const auto out = c.update(s, returning ? Vector3f() : angle, .0025f, 100010);
                    ASSERT_TRUE(out.valid);
                    EXPECT_NEAR(out.error[axis], angle[axis] * (returning ? -1 : 1), 2.0e-7f);
                    EXPECT_NEAR(out.rates[axis], out.error[axis] * (axis == 0 ? 2 : 3), 1.0e-7f);
                    EXPECT_NEAR(out.rates[1-axis], 0, 1.0e-7f);
                }
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify combined independent rotation matrix reference.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude,
///              CombinedIndependentRotationMatrixReference).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, CombinedIndependentRotationMatrixReference)
{
    // Independent double-precision 321 matrix columns and dot products.
    for (float yaw : {
             0.0f, 1.2f, -3.14f
         }) {
        const double r = .02, p = -.03, rd = -.04, pd = .025;
        const double target[] = {cos(yaw)*sin(pd)*cos(rd)+sin(yaw)*sin(rd),
                                 sin(yaw)*sin(pd)*cos(rd)-cos(yaw)*sin(rd), cos(pd)*cos(rd)
                                };
        const double bx[] = {cos(yaw)*cos(p), sin(yaw)*cos(p), -sin(p)};
        const double by[] = {cos(yaw)*sin(p)*sin(r)-sin(yaw)*cos(r),
                             sin(yaw)*sin(p)*sin(r)+cos(yaw)*cos(r), cos(p)*sin(r)
                            };
        const double bz[] = {cos(yaw)*sin(p)*cos(r)+sin(yaw)*sin(r),
                             sin(yaw)*sin(p)*cos(r)-cos(yaw)*sin(r), cos(p)*cos(r)
                            };
        double x = 0, y = 0, z = 0;
        for (uint8_t i = 0; i < 3; i++) {
            x += bx[i]*target[i]; y += by[i]*target[i]; z += bz[i]*target[i];
        }
        const double k = atan2(hypot(x, y), z)/hypot(x, y);
        AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
        const auto out = c.update(state(r, p, yaw), {float(rd), float(pd), 0}, .0025f, 100010);
        ASSERT_TRUE(out.valid);
        EXPECT_NEAR(out.error.x, -y*k, 1.0e-7); EXPECT_NEAR(out.error.y, x*k, 1.0e-7);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify quaternion sign and yaw wrap invariant.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, QuaternionSignAndYawWrapInvariant).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, QuaternionSignAndYawWrapInvariant)
{
    auto s = state(.02f, -.03f, 3.14f);
    AP_UTB_SITLAttitude a, b; ASSERT_TRUE(a.configure({})); ASSERT_TRUE(b.configure({}));
    const auto out = a.update(s, {}, .0025f, 100010);
    for (uint8_t i = 0; i < 4; i++) {
        s.attitude_body_to_ned[i] *= -1;
    }
    const auto opposite = b.update(s, {}, .0025f, 100010);
    EXPECT_NEAR((out.rates-opposite.rates).length(), 0, 1.0e-7f);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify norm clamp preserves direction.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, NormClampPreservesDirection).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, NormClampPreservesDirection)
{
    AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({6, 6, .05f}));
    const auto out = c.update(state(), {radians(3.0f), radians(3.0f), .1f}, .0025f, 100010);
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.limited);
    EXPECT_NEAR(Vector2f(out.rates.x, out.rates.y).length(), .05f, 1.0e-7f);
    EXPECT_NEAR(out.rates.x/out.rates.y, out.error.x/out.error.y, 1.0e-6f);
    EXPECT_FLOAT_EQ(out.rates.z, .1f);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify invalid quaternion and state health.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, InvalidQuaternionAndStateHealth).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, InvalidQuaternionAndStateHealth)
{
    for (uint8_t fault = 0; fault < 7; fault++) {
        AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({})); auto s = state();
        switch (fault) {
        case 0: s.attitude_body_to_ned.zero(); break;
        case 1: s.attitude_body_to_ned.q1 = NAN; break;
        case 2: s.attitude_body_to_ned.q2 = INFINITY; break;
        case 3: s.attitude_valid = false; break;
        case 4: s.rates_valid = false; break;
        case 5: s.sampled_us -= 10001; break;
        case 6: s.imu_updated_us -= 10001; break;
        }
        EXPECT_FALSE(c.update(s, {}, .0025f, 100010).valid);
        EXPECT_EQ(c.fault(), AP_UTB_SITLAttitude::Fault::STATE);
        EXPECT_FALSE(c.update(state(), {}, .0025f, 100010).valid);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify invalid dt.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, InvalidDt).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, InvalidDt)
{
    for (float dt : {
             0.0f, -.1f, .000624f, .01001f, float(NAN), float(INFINITY)
         }) {
        AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
        EXPECT_FALSE(c.update(state(), {}, dt, 100010).valid);
        EXPECT_EQ(c.fault(), AP_UTB_SITLAttitude::Fault::DT);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify invalid reference and singularity rejected.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, InvalidReferenceAndSingularityRejected).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, InvalidReferenceAndSingularityRejected)
{
    for (Vector3f ref : {
             Vector3f(NAN, 0, 0), Vector3f(0, INFINITY, 0), Vector3f(0, 0, NAN),
             Vector3f(.09f, 0, 0), Vector3f(0, 0, .101f)
         }) {
        AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
        EXPECT_FALSE(c.update(state(), ref, .0025f, 100010).valid);
        EXPECT_EQ(c.fault(), AP_UTB_SITLAttitude::Fault::REFERENCE);
    }
    AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
    EXPECT_FALSE(c.update(state(0, radians(90.0f)), {}, .0025f, 100010).valid);
    EXPECT_EQ(c.fault(), AP_UTB_SITLAttitude::Fault::ENVELOPE);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify repeated insprimary and clock wrap.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, RepeatedINSPrimaryAndClockWrap).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, RepeatedINSPrimaryAndClockWrap)
{
    AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
    EXPECT_TRUE(c.update(state(0, 0, 0, UINT32_MAX-1000), {}, .0025f, UINT32_MAX-990).valid);
    EXPECT_TRUE(c.update(state(0, 0, 0, 1499), {}, .0025f, 1509).valid);
    EXPECT_FALSE(c.update(state(0, 0, 0, 1499), {}, .0025f, 1509).valid);
    EXPECT_EQ(c.fault(), AP_UTB_SITLAttitude::Fault::IMU);
    ASSERT_TRUE(c.configure({})); EXPECT_TRUE(c.update(state(), {}, .0025f, 100010).valid);
    auto s = state(0, 0, 0, 102500); s.primary_gyro = 1;
    EXPECT_FALSE(c.update(s, {}, .0025f, 102510).valid);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify configuration freeze and invalid gain.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, ConfigurationFreezeAndInvalidGain).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, ConfigurationFreezeAndInvalidGain)
{
    AP_UTB_SITLAttitude c; ASSERT_TRUE(c.configure({}));
    EXPECT_TRUE(c.configuration_matches({2, 2, .3f}));
    EXPECT_FALSE(c.configuration_matches({3, 2, .3f}));
    EXPECT_FALSE(c.configure({NAN, 2, .3f}));
    EXPECT_FALSE(c.configure({2, 7, .3f}));
    EXPECT_FALSE(c.configure({2, 2, .31f}));
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify finite references and original rate path.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, FiniteReferencesAndOriginalRatePath).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, FiniteReferencesAndOriginalRatePath)
{
    for (uint8_t p : {
             0, 1, 2, 4, 5, 6
         }) {
        for (uint32_t i = 0; i < AP_UTB_SITLAttitude::DURATION_CYCLES; i++) {
            auto ref = AP_UTB_SITLAttitude::reference(p, i, radians(3.0f), 0);
            EXPECT_LE(Vector2f(ref.x, ref.y).length(), radians(5.0f));
        }
        EXPECT_FLOAT_EQ(AP_UTB_SITLAttitude::reference(p, 5600, radians(3.0f), 0).length(), 0);
    }
    EXPECT_FLOAT_EQ(AP_UTB_SITLRate::reference(1, 400, .15f).x, .15f);
    EXPECT_EQ(AP_UTB_SITLRate::duration_cycles(1), 3200U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify attitude fault stops existing inner before its next update.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude,
///              AttitudeFaultStopsExistingInnerBeforeItsNextUpdate).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, AttitudeFaultStopsExistingInnerBeforeItsNextUpdate)
{
    AP_UTB_SITLAttitude outer; ASSERT_TRUE(outer.configure({}));
    AP_UTB_SITLRate inner;
    const AP_UTB_RateController::Gains gains[3] = {
        {.19f, .16f, .0015f, .19f, 15}, {.19f, .16f, .001f, .19f, 20}, {.3f, .6f, 0, .19f, 20}
    };
    ASSERT_TRUE(inner.configure(gains));
    auto bad = state(); bad.attitude_valid = false;
    EXPECT_FALSE(outer.update(bad, {}, .0025f, 100010).valid);
    inner.invalidate(AP_UTB_SITLRate::Fault::ATTITUDE);
    AP_UTB_SITLRate::Inputs in;
    in.state = state(); in.now_us = 100010; in.dt = in.nominal_dt = .0025f;
    in.frame_class = 1; in.frame_type = 18;
    const auto out = inner.step(in, true);
    EXPECT_FALSE(out.valid); EXPECT_FLOAT_EQ(out.command.length(), 0);
    EXPECT_EQ(inner.calls(), 0U);
    EXPECT_EQ(inner.fault(), AP_UTB_SITLRate::Fault::ATTITUDE);
    ASSERT_TRUE(outer.configure({}));
    EXPECT_TRUE(outer.update(state(), {}, .0025f, 100010).valid);
    EXPECT_FALSE(inner.step(in, true).valid);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify valid outer does not bypass inner frame fault.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBAttitude, ValidOuterDoesNotBypassInnerFrameFault).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBAttitude, ValidOuterDoesNotBypassInnerFrameFault)
{
    AP_UTB_SITLAttitude outer; ASSERT_TRUE(outer.configure({}));
    auto angle = outer.update(state(), {.03f, 0, 0}, .0025f, 100010);
    ASSERT_TRUE(angle.valid);
    AP_UTB_SITLRate inner;
    const AP_UTB_RateController::Gains gains[3] = {
        {.19f, .16f, .0015f, .19f, 15}, {.19f, .16f, .001f, .19f, 20}, {.3f, .6f, 0, .19f, 20}
    };
    ASSERT_TRUE(inner.configure(gains));
    AP_UTB_SITLRate::Inputs in;
    in.state = state(); in.target = angle.rates; in.now_us = 100010;
    in.dt = in.nominal_dt = .0025f; in.frame_class = 1; in.frame_type = 12;
    EXPECT_FALSE(inner.step(in, true).valid);
    EXPECT_EQ(inner.fault(), AP_UTB_SITLRate::Fault::FRAME);
    EXPECT_EQ(inner.calls(), 0U);
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
