// NOTICE: UTB diagnostic tests, GPL-3.0-or-later.
/// @file test_utb_logger_trace.cpp
/// @brief Deterministic tests of bounded main-only RAM capture.
/// @author Jiri Kovar
/// @version 2026-10-21
/// @date 2026-10-21
#include <AP_gtest.h>
#include <AP_Logger/AP_Logger_UTBTrace_Recorder.h>
using namespace UTBLoggerTrace;

// Global test functions; no live HAL, logger or motor objects.
TEST(UTBLoggerTrace, InactiveAndNativeThreshold)
{
    Recorder r;
    EXPECT_EQ(r.begin(Event::START_LOG, 1, 2), 0U);
    r.loop(10000, 2500);
    r.loop(13000, 2500);
    EXPECT_FALSE(r.frozen());
    r.loop(16001, 2500);
    ASSERT_TRUE(r.frozen());
    EXPECT_EQ(r.period(), 3001U);
    EXPECT_EQ(r.trigger(), 16001U);
    EXPECT_EQ(r.begin(Event::START_LOG, 16002, 2), 0U);
}
TEST(UTBLoggerTrace, NestedSpanAndImmutableFreeze)
{
    Recorder r;
    uint32_t outer;
    uint32_t inner;
    r.loop(10000, 2500);
    outer = r.begin(Event::START_LOG, 10100, 4);
    inner = r.begin(Event::FIND_PAGE, 10200, 4);
    r.end(inner, 14000, 10200, Event::FIND_PAGE);
    r.end(outer, 15000, 10100, Event::START_LOG);
    r.loop(16000, 2500);
    ASSERT_TRUE(r.frozen());
    EXPECT_EQ(r.entry(0).duration, 4900U);
    EXPECT_EQ(r.entry(1).duration, 3800U);
    r.end(outer, 17000, 10100, Event::START_LOG);
    r.loop(18000, 2500);
    EXPECT_EQ(r.entry(0).duration, 4900U);
    EXPECT_EQ(r.period(), 6000U);
}
TEST(UTBLoggerTrace, ClockWrap)
{
    Recorder r;
    uint32_t ticket;
    r.loop(UINT32_MAX - 1000, 2500);
    ticket = r.begin(Event::WAIT_READY, UINT32_MAX - 100, -1);
    r.end(ticket, 99, UINT32_MAX - 100, Event::WAIT_READY);
    r.loop(1499, 2500);
    EXPECT_FALSE(r.frozen());
    r.loop(5000, 2500);
    ASSERT_TRUE(r.frozen());
    EXPECT_EQ(r.entry(0).duration, 200U);
    EXPECT_EQ(r.entry(0).task, -1);
    EXPECT_EQ(r.period(), 3501U);
}
TEST(UTBLoggerTrace, RingOverflowKeepsAggregateAndChronology)
{
    Recorder r;
    uint32_t outer;
    r.loop(10000, 2500);
    outer = r.begin(Event::START_LOG, 10100, 4);
    for (uint32_t i = 0; i < Recorder::CAPACITY + 5; i++) {
        uint32_t ticket = r.begin(Event::WAIT_READY, 10200 + i, 4);
        r.end(ticket, 10201 + i, 10200 + i, Event::WAIT_READY);
    }
    r.end(outer, 15500, 10100, Event::START_LOG);
    r.loop(16000, 2500);
    ASSERT_TRUE(r.frozen());
    EXPECT_EQ(r.count(), Recorder::CAPACITY);
    EXPECT_EQ(r.lost_completions(), 1U);
    EXPECT_EQ(r.summary(Event::START_LOG).maximum, 5400U);
    for (uint32_t i = 1; i < r.count(); i++) {
        EXPECT_EQ(r.entry(i).ticket, r.entry(i - 1).ticket + 1);
    }
}
TEST(UTBLoggerTrace, ArmedAbortIsNotALongCycle)
{
    Recorder r;
    r.loop(10000, 2500);
    r.abort(11000);
    ASSERT_TRUE(r.frozen());
    EXPECT_TRUE(r.aborted());
    EXPECT_EQ(r.period(), 0U);
    EXPECT_EQ(r.begin(Event::START_LOG, 12000, 2), 0U);
    r.loop(16000, 2500);
    EXPECT_EQ(r.period(), 0U);
}
TEST(UTBLoggerTrace, NativeBudgetOverrunKeepsTaskIdentity)
{
    Recorder r;
    r.task_overrun(3, 100, 50);
    EXPECT_EQ(r.count(), 0U);
    r.loop(10000, 2500);
    r.task_overrun(42, 10100, 5993);
    r.loop(16500, 2500);
    ASSERT_TRUE(r.frozen());
    EXPECT_EQ(r.entry(0).task, 42);
    EXPECT_EQ(r.entry(0).start, 10100U);
    EXPECT_EQ(r.entry(0).duration, 5993U);
    EXPECT_EQ(r.summary(Event::BUDGET_OVERRUN).task, 42);
}
AP_GTEST_MAIN()
