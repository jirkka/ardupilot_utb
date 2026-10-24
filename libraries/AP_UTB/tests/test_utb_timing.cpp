/* GNU GPL v3 or later. Paired timing engine regression tests.
 * @author Jiri Kovar
 * @version 2026-10-24
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#include <AP_UTB/AP_UTB_Timing.h>
#if AP_UTB_TIMING_ENABLED
static void sample(AP_UTB_TimingEngine &e, uint32_t cycle, uint32_t main, uint32_t delay = 100, int8_t uas = 0)
{
    e.begin(cycle, main); e.context(0, uas, true); e.rate(main + 10); e.yaw_reset();
    e.attitude(main + 10 + delay, 0); e.finish();
}
TEST(UTBTiming, ConstantPeriodsAndDelays)
{
    AP_UTB_TimingEngine e;
    e.start(2500);
    for (uint32_t i = 0; i < 20; i++) {
        sample(e, i, i*2500);
    }
    const auto &s = e.stats();
    EXPECT_EQ(s.cycles, 20U); EXPECT_EQ(s.metrics[0][0].count, 19U);
    EXPECT_EQ(s.metrics[0][0].absolute_error, 0U);
    EXPECT_EQ(s.metrics[0][2].sum, 19*2500);
    EXPECT_EQ(s.metrics[0][3].sum, 2000);
    EXPECT_EQ(s.metrics[0][4].minimum, 0); EXPECT_EQ(s.missing_rate, 0U);
}
TEST(UTBTiming, VariableDelayWithoutMainJitter)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0, 100); sample(e, 2, 2500, 900); sample(e, 3, 5000, 100);
    const auto &s = e.stats();
    EXPECT_EQ(s.metrics[0][0].absolute_error, 0U); EXPECT_EQ(s.metrics[0][1].absolute_error, 0U);
    EXPECT_EQ(s.metrics[0][2].minimum, 1700); EXPECT_EQ(s.metrics[0][2].maximum, 3300);
    EXPECT_EQ(s.metrics[0][2].absolute_error, 1600U);
    EXPECT_EQ(s.metrics[0][4].minimum, -800); EXPECT_EQ(s.metrics[0][4].maximum, 800);
    EXPECT_EQ(s.metrics[0][2].maximum - 2500, s.metrics[0][4].maximum);
}
TEST(UTBTiming, MainJitterWithFixedDelay)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0); sample(e, 2, 3000); sample(e, 3, 5000);
    EXPECT_EQ(e.stats().metrics[0][0].absolute_error, 1000U);
    EXPECT_EQ(e.stats().metrics[0][1].absolute_error, 1000U);
    EXPECT_EQ(e.stats().metrics[0][2].absolute_error, 1000U);
    EXPECT_EQ(e.stats().metrics[0][4].absolute_error, 0U);
}
TEST(UTBTiming, ClockAndCycleWrapIncludingTimestampZero)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, UINT32_MAX, UINT32_MAX - 2499U); sample(e, 0, 0); sample(e, 1, 2500);
    EXPECT_EQ(e.stats().metrics[0][0].count, 2U);
    EXPECT_EQ(e.stats().metrics[0][2].absolute_error, 0U); EXPECT_EQ(e.stats().cycle_gaps, 0U);
    EXPECT_EQ(e.stats().invalid_time, 0U);
}
TEST(UTBTiming, MissingSamplesNeverBorrowPreviousCycle)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0);
    e.begin(2, 2500); e.context(0, 0, true); e.finish();
    sample(e, 3, 5000); sample(e, 4, 7500);
    EXPECT_EQ(e.stats().missing_rate, 1U); EXPECT_EQ(e.stats().missing_att, 1U);
    EXPECT_EQ(e.stats().metrics[0][2].count, 1U); EXPECT_EQ(e.stats().metrics[0][0].count, 3U);
}
TEST(UTBTiming, DuplicateCallsExplicit)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0);
    e.begin(2, 2500); e.rate(2510); e.rate(2520); e.attitude(2600, 0); e.attitude(2700, 1); e.finish();
    EXPECT_EQ(e.stats().duplicate_rate, 1U); EXPECT_EQ(e.stats().duplicate_att, 1U);
    EXPECT_EQ(e.stats().metrics[0][2].count, 0U); EXPECT_EQ(e.stats().metrics[0][3].count, 1U);
}
TEST(UTBTiming, UnfinishedCycleAndGap)
{
    AP_UTB_TimingEngine e;
    e.start(2500); e.begin(1, 0); sample(e, 3, 5000);
    EXPECT_EQ(e.stats().cycles, 2U); EXPECT_EQ(e.stats().missing_finish, 1U);
    EXPECT_EQ(e.stats().cycle_gaps, 1U); EXPECT_EQ(e.stats().metrics[0][0].count, 0U);
}
TEST(UTBTiming, PhaseBoundaryRetainedInAggregate)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0, 100, 0); sample(e, 2, 2500, 100, 0);
    sample(e, 3, 5000, 300, 1); sample(e, 4, 7500, 300, 1);
    EXPECT_EQ(e.stats().phase_changes, 1U); EXPECT_EQ(e.stats().metrics[0][0].count, 3U);
    EXPECT_EQ(e.stats().metrics[1][0].count, 1U); EXPECT_EQ(e.stats().metrics[2][0].count, 1U);
    EXPECT_EQ(e.stats().metrics[0][4].maximum, 200);
}
TEST(UTBTiming, WorstContextBoundedWithEightNeighbors)
{
    AP_UTB_TimingEngine e;
    e.start(2500);
    for (uint32_t i = 1; i <= 100; i++) {
        sample(e, i, i*2500, i < 50 ? 100 : 600);
    }
    EXPECT_EQ(e.worst_cycle(), 50U); EXPECT_EQ(e.worst_count(), 17U);
    EXPECT_EQ(e.worst(0).cycle, 42U); EXPECT_EQ(e.worst(16).cycle, 58U);
    EXPECT_EQ(e.worst(8).yaw_resets, 1U);
}
TEST(UTBTiming, ExtremaPositionsAndStructuralCounters)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 10); sample(e, 2, 2510); sample(e, 3, 7510);
    const auto &m = e.stats().metrics[0][0];
    EXPECT_EQ(m.minimum, 2500); EXPECT_EQ(m.maximum, 5000);
    EXPECT_EQ(m.min_cycle, 2U); EXPECT_EQ(m.max_cycle, 3U); EXPECT_EQ(m.max_us, 7510U);
    EXPECT_EQ(m.over_nominal, 1U); EXPECT_EQ(m.over_two_periods, 1U);
}
TEST(UTBTiming, BackwardTimeExplicitInvalid)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 3000); sample(e, 2, 2000);
    EXPECT_EQ(e.stats().invalid_time, 3U); EXPECT_EQ(e.stats().metrics[0][0].count, 0U);
}
TEST(UTBTiming, UnsupportedAndModeTransitionsAreVisible)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0);
    e.begin(2, 2500); e.context(5, -1, false); e.rate(2510); e.attitude(2610, 0); e.finish();
    EXPECT_EQ(e.stats().unsupported, 1U); EXPECT_EQ(e.stats().phase_changes, 1U);
    EXPECT_EQ(e.stats().metrics[3][3].count, 1U);
}
TEST(UTBTiming, CancellationRetainsUnfinishedSample)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0); e.begin(2, 2500); e.rate(2510); e.close_pending();
    EXPECT_EQ(e.stats().cycles, 2U); EXPECT_EQ(e.stats().missing_finish, 1U);
    EXPECT_EQ(e.stats().missing_att, 1U); EXPECT_FALSE(e.pending());
}
TEST(UTBTiming, RestartClearsHistoryAndExtrema)
{
    AP_UTB_TimingEngine e;
    e.start(2500); sample(e, 1, 0); sample(e, 2, 5000);
    e.start(2500); sample(e, 1, 0); sample(e, 2, 2500);
    EXPECT_EQ(e.stats().cycles, 2U); EXPECT_EQ(e.stats().metrics[0][0].absolute_error, 0U);
    EXPECT_EQ(e.stats().metrics[0][0].maximum, 2500);
}
#endif
AP_GTEST_MAIN()
