// GNU GPL v3 or later.
/// @file test_utb_flight_shadow.cpp
/// @brief Recorder bounds/lifecycle tests; no motor, logger or live controller is supplied.
/// @author Jiri Kovar
/// @version 2026-10-25
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_UTB/AP_UTB_ChainShadow.h>
#include <AP_Common/ExpandingString.h>
#include <type_traits>
#include <cfenv>
#include <cstring>
#include <cstdlib>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
class UTBFlightTestAccess
{
public:
    static void rejected_export(ExpandingString &text)
    {
        static AP_UTB_ChainShadow wrapper;
        wrapper.init();
        wrapper._engine.start();
        wrapper._flight_session = true;
        wrapper.freeze(6);
        AP_UTB_ChainShadow::dump_flight(text, 255);
    }
    static AP_UTB_FlightShadow &fixture(int32_t count = 24000, int16_t decim = 1, int8_t run = 1)
    {
        static AP_UTB_FlightShadow recorder;
        recorder._run.set(run); recorder._count.set(count); recorder._decim.set(decim);
        recorder._stats = {}; recorder._head = 0; recorder._used = 0; recorder._stride = 0;
        recorder._started = false; recorder._closed = false; recorder._armed = false; recorder._spool = 0;
        recorder._reason = 0; recorder._limit = 0; recorder._start_us = 0; recorder._stop_us = 0;
        return recorder;
    }
};
static UTB_ChainShadowEngine &empty_engine()
{
    static UTB_ChainShadowEngine engine;
    engine.start();
    return engine;
}
TEST(UTBFlightRecorder, RejectedEmptyExportDoesNotRaiseInvalidFloat)
{
    ExpandingString text;
    std::feclearexcept(FE_ALL_EXCEPT);
    UTBFlightTestAccess::rejected_export(text);
    EXPECT_EQ(std::fetestexcept(FE_INVALID | FE_DIVBYZERO), 0);
    ASSERT_NE(text.get_string(), nullptr);
    EXPECT_NE(strstr(text.get_string(), "UTBFlightV1,frozen,reason=6,"), nullptr);
    EXPECT_NE(strstr(text.get_string(), "att,0,0,0.000000000,nan"), nullptr);
    const char *prefix = "flight_metric64,att,0,0,0000000000000000,";
    const char *metric = strstr(text.get_string(), prefix);
    ASSERT_NE(metric, nullptr);
    const uint64_t bits = strtoull(metric + strlen(prefix), nullptr, 16);
    double rmse;
    memcpy(&rmse, &bits, sizeof(rmse));
    EXPECT_TRUE(isnan(rmse)); // A quiet NaN payload is platform-dependent; zero samples are never a zero error.
}
TEST(UTBFlightRecorder, DefaultOffAndInvalidRequests)
{
    AP_UTB_FlightShadow &recorder = UTBFlightTestAccess::fixture(24000, 200, 0);
    EXPECT_FALSE(recorder.requested());
    EXPECT_FALSE(recorder.start());
    EXPECT_FALSE(UTBFlightTestAccess::fixture(48001).start());
    EXPECT_FALSE(UTBFlightTestAccess::fixture(1).start());
    EXPECT_FALSE(UTBFlightTestAccess::fixture(20, 0).start());
    EXPECT_FALSE(UTBFlightTestAccess::fixture(20, 401).start());
    EXPECT_FALSE(UTBFlightTestAccess::fixture(20, 2, 2).start());
}
TEST(UTBFlightRecorder, FixedValueStorage)
{
    EXPECT_TRUE(std::is_trivially_copyable<AP_UTB_FlightShadow::Record>::value);
    EXPECT_LE(sizeof(AP_UTB_FlightShadow::Record), 320U);
}
TEST(UTBFlightRecorder, FullRingOverwritesEveryOldestRowExplicitly)
{
    AP_UTB_FlightShadow &recorder = UTBFlightTestAccess::fixture();
    UTB_ChainShadowEngine &engine = empty_engine();
    ASSERT_TRUE(recorder.start());
    for (uint32_t i = 1; i <= 130; i++) {
        recorder.record(engine, i, false);
    }
    EXPECT_EQ(recorder.size(), 128);
    EXPECT_EQ(recorder.statistics().candidates, 130U);
    EXPECT_EQ(recorder.statistics().selected, 130U);
    EXPECT_EQ(recorder.statistics().overwritten, 2U);
    EXPECT_EQ(recorder.oldest(0).id, 3U);
    EXPECT_EQ(recorder.oldest(127).id, 130U);
    EXPECT_EQ(recorder.oldest(127).loop, 130U);
    EXPECT_EQ(engine.statistics().attempts, 0U); // Recording cannot execute or change the engine.
}
TEST(UTBFlightRecorder, DecimationDoesNotHideForcedFaultSample)
{
    AP_UTB_FlightShadow &recorder = UTBFlightTestAccess::fixture(1000, 200);
    UTB_ChainShadowEngine &engine = empty_engine();
    ASSERT_TRUE(recorder.start());
    for (uint32_t i = 1; i <= 500; i++) {
        recorder.record(engine, i, i == 500);
    }
    EXPECT_EQ(recorder.statistics().selected, 4U); // 1,201,401 and forced500.
    EXPECT_EQ(recorder.statistics().decimated, 496U);
    EXPECT_EQ(recorder.oldest(3).loop, 500U);
    EXPECT_TRUE(recorder.oldest(3).flags & 2U);
    EXPECT_FALSE(recorder.oldest(3).flags & 1U); // No synthetic rate is claimed for an empty engine.
}
TEST(UTBFlightRecorder, ArmEpochIsMetadataWithoutReimportingEngine)
{
    AP_UTB_FlightShadow &recorder = UTBFlightTestAccess::fixture();
    UTB_ChainShadowEngine &engine = empty_engine();
    ASSERT_TRUE(recorder.start());
    recorder.context(true, 3); recorder.record(engine, 1, false);
    EXPECT_EQ(recorder.oldest(0).epoch, 2U);
    EXPECT_EQ(recorder.oldest(0).spool, 3);
    EXPECT_TRUE(recorder.oldest(0).flags & 4U);
    recorder.context(false, 0); recorder.record(engine, 2, false);
    EXPECT_EQ(recorder.statistics().epochs, 3U);
    EXPECT_EQ(recorder.statistics().arm_changes, 2U);
    EXPECT_EQ(engine.attitude_engine().statistics().synchronizations, 0U);
}
TEST(UTBFlightRecorder, ClosedSessionNeverResumesOrChangesPublishedRows)
{
    AP_UTB_FlightShadow &recorder = UTBFlightTestAccess::fixture();
    UTB_ChainShadowEngine &engine = empty_engine();
    ASSERT_TRUE(recorder.start());
    recorder.record(engine, 1, false); recorder.stop(7);
    recorder.context(true, 3); recorder.record(engine, 2, true);
    EXPECT_FALSE(recorder.requested());
    EXPECT_FALSE(recorder.start());
    EXPECT_EQ(recorder.statistics().candidates, 1U);
    EXPECT_EQ(recorder.statistics().arm_changes, 0U);
    EXPECT_EQ(recorder.oldest(0).loop, 1U);
}
TEST(UTBFlightRecorder, PagedExportIsBoundedAndMarked)
{
    AP_UTB_FlightShadow &recorder = UTBFlightTestAccess::fixture();
    UTB_ChainShadowEngine &engine = empty_engine();
    ExpandingString page;
    ASSERT_TRUE(recorder.start());
    for (uint32_t i = 1; i <= 130; i++) {
        recorder.record(engine, i, false);
    }
    recorder.stop(1); recorder.dump_page(page, 0);
    EXPECT_FALSE(page.has_failed_allocation());
    EXPECT_LT(page.get_length(), 20000U);
    EXPECT_NE(strstr(page.get_string(), "UTBFlightRowsV1"), nullptr);
    EXPECT_NE(strstr(page.get_string(), "overwritten=2"), nullptr);
    page.reset(); recorder.dump_page(page, 8);
    EXPECT_EQ(page.get_length(), 0U);
}
#endif
AP_GTEST_MAIN()
