/* GNU GPL v3 or later.
 * @author Jiri Kovar
 * @version 2026-10-25
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#include <AP_UTB/AP_UTB_ExecutionTiming.h>
#if AP_UTB_TIMING_EXEC_ENABLED
using E = AP_UTB_ExecutionTiming;
static void sample(E &e, uint32_t cycle, uint32_t base, int skip = -1, bool consume = true, uint32_t motor = 100)
{
    const uint32_t offset[E::EVENTS] = {10, 20, 40, motor, motor + 30, motor + 80, motor + 100, motor + 120, motor + 150};
    e.begin(cycle, base);
    for (uint8_t k = 0; k < E::EVENTS; k++) {
        if (k != skip && (consume || (k != E::CONSUME_BEGIN && k != E::CONSUME_END))) {
            e.event(E::Event(k), cycle, base + offset[k]);
        }
    }
    e.close();
}
TEST(UTBExecution, ExactEightMetrics)
{
    E e; e.start(); sample(e, 1, 1000);
    const uint32_t expected[] = {100,30,130,250,90,20,200,220};
    for (uint8_t k = 0; k < 8; k++) {
        EXPECT_EQ(e.stats().metric[k].count, 1U); EXPECT_EQ(e.stats().metric[k].sum, expected[k]);
        EXPECT_EQ(e.stats().metric[k].minimum, expected[k]); EXPECT_EQ(e.stats().metric[k].maximum, expected[k]);
    }
    EXPECT_FALSE(e.failed());
}
TEST(UTBExecution, OptionalConsumeAbsentNotZeroSample)
{
    E e; e.start(); sample(e,1,0,-1,false);
    EXPECT_EQ(e.stats().metric[5].count,0U); EXPECT_EQ(e.stats().metric[2].count,1U); EXPECT_FALSE(e.failed());
}
TEST(UTBExecution, MissingEveryRequiredEventRejectsWholeSet)
{
    for (int k : {
             E::RATE,E::MOTOR_BEGIN,E::MOTOR_END,E::ATTITUDE,E::MODE_END,E::FAST_END,E::SCHED_END
         }) {
        E e; e.start(); sample(e,1,0,k);
        EXPECT_EQ(e.stats().missing,1U); EXPECT_EQ(e.stats().metric[2].count,0U); EXPECT_TRUE(e.failed());
    }
}
TEST(UTBExecution, HalfConsumeIsMissing)
{
    E e; e.start(); sample(e,1,0,E::CONSUME_END);
    EXPECT_EQ(e.stats().missing,1U); EXPECT_EQ(e.stats().metric[5].count,0U);
}
TEST(UTBExecution, DuplicateMotorNeverBecomesZeroDuration)
{
    E e; e.start(); e.begin(1,0); e.event(E::RATE,1,10); e.event(E::MOTOR_BEGIN,1,20); e.event(E::MOTOR_BEGIN,1,25); e.close();
    EXPECT_EQ(e.stats().duplicate,1U); EXPECT_EQ(e.stats().metric[0].count,0U);
}
TEST(UTBExecution, WrongLoopAndUnopenedEvents)
{
    E e; e.start(); e.event(E::RATE,1,10); e.begin(4,100); e.event(E::RATE,3,110); e.close();
    EXPECT_EQ(e.stats().invalid,2U); EXPECT_EQ(e.stats().metric[0].count,0U);
}
TEST(UTBExecution, BackwardTime)
{
    E e; e.start(); e.begin(1,100); e.event(E::RATE,1,99); e.close(); EXPECT_GT(e.stats().invalid,0U);
}
TEST(UTBExecution, ReversedEventOrder)
{
    E e; e.start(); e.begin(1,0); e.event(E::MOTOR_END,1,100); e.event(E::MOTOR_BEGIN,1,120); e.close();
    EXPECT_GT(e.stats().invalid,0U); EXPECT_EQ(e.stats().metric[1].count,0U);
}
TEST(UTBExecution, WrapAndTimestampZero)
{
    E e; e.start(); sample(e,UINT32_MAX,UINT32_MAX-99); sample(e,0,2400);
    EXPECT_EQ(e.stats().metric[0].minimum,100U); EXPECT_EQ(e.stats().metric[3].sum,500U); EXPECT_FALSE(e.failed());
}
TEST(UTBExecution, UnfinishedSchedulerClosesBeforeNextLoop)
{
    E e; e.start(); sample(e,1,0,E::SCHED_END); sample(e,2,2500);
    EXPECT_EQ(e.stats().cycles,2U); EXPECT_EQ(e.stats().metric[3].count,1U); EXPECT_EQ(e.stats().missing,1U);
}
TEST(UTBExecution, BeginWithoutCloseRetainsMissing)
{
    E e; e.start(); e.begin(1,0); sample(e,2,2500); EXPECT_EQ(e.stats().cycles,2U); EXPECT_GT(e.stats().missing,0U);
}
TEST(UTBExecution, SequenceGap)
{
    E e; e.start(); sample(e,1,0); sample(e,3,5000); EXPECT_EQ(e.stats().gaps,1U); EXPECT_EQ(e.stats().metric[2].count,1U);
}
TEST(UTBExecution, UnsupportedNoFakeValidMetrics)
{
    E e; e.start(); e.begin(1,0); e.unsupported(); e.close(); EXPECT_EQ(e.stats().unsupported,1U); EXPECT_EQ(e.stats().metric[0].count,0U);
}
TEST(UTBExecution, BoundedWorstContext)
{
    E e; e.start();
    for (uint32_t k=1; k<=1000; k++) {
        sample(e,k,k*2500,-1,true,k==50?1000:100);
    }
    EXPECT_EQ(e.context_count(),17U); EXPECT_EQ(e.context(8).cycle,50U);
    EXPECT_EQ(e.context(0).cycle,42U); EXPECT_EQ(e.context(16).cycle,58U);
}
TEST(UTBExecution, EmptyContextAndRestart)
{
    E e; e.start(); EXPECT_EQ(e.context_count(),0U); sample(e,1,0); e.start(); EXPECT_EQ(e.context_count(),0U); EXPECT_EQ(e.stats().cycles,0U);
}
TEST(UTBExecution, SumExtremaAndThresholds)
{
    E e; e.start(); sample(e,1,0,-1,true,2501); sample(e,2,10000,-1,true,5000);
    EXPECT_EQ(e.stats().metric[0].sum,7501U); EXPECT_EQ(e.stats().metric[0].over_nominal,2U);
    EXPECT_EQ(e.stats().metric[0].over_two_periods,1U); EXPECT_EQ(e.stats().metric[0].max_cycle,2U);
    EXPECT_EQ(e.stats().metric[0].max_us,10000U);
}
TEST(UTBExecution, EqualTimestampsAreValidAtClockResolution)
{
    E e; e.start(); e.begin(1,0);
    for (uint8_t k=0; k<E::EVENTS; k++) {
        e.event(E::Event(k),1,0);
    }
    e.close(); EXPECT_FALSE(e.failed()); EXPECT_EQ(e.stats().metric[2].count,1U); EXPECT_EQ(e.stats().metric[2].sum,0U);
}
TEST(UTBExecution, MultiwordSumAndReadOnlyFrozenCopy)
{
    E e; e.start();
    for (uint32_t k = 1; k <= 20000; k++) {
        sample(e, k, k * 1000000U, -1, true, 500000);
    }
    EXPECT_EQ(e.stats().metric[0].sum, 10000000000ULL);
    const auto before = e.stats().metric[0].sum;
    EXPECT_EQ(e.stats().metric[0].count, 20000U); e.close();
    EXPECT_EQ(e.stats().metric[0].sum, before);
}
TEST(UTBExecution, MainTimestampBackwards)
{
    E e; e.start(); sample(e, 1, 5000); sample(e, 2, 4000);
    EXPECT_EQ(e.stats().invalid, 1U); EXPECT_EQ(e.stats().metric[0].count, 1U);
}
#endif
AP_GTEST_MAIN()
