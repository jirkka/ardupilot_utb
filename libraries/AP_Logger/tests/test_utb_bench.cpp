//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       test_utb_bench.cpp
///  @brief      Analytické/lifecycle regrese příslušné komponenty; 12 deklarovaných TEST případů.
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
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_BENCH_ENABLED
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#include <thread>
#include <chrono>
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
using namespace UTBBench;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify stats and bins.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, StatsAndBins).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, StatsAndBins)
{
    Stats stats;
    for (uint32_t v : {
             0U, 1U, 2U, 7U, 4096U
         }) {
        stats.record(v);
    }
    Sample s;
    ASSERT_TRUE(stats.read(s));
    EXPECT_EQ(s.count, 5U); EXPECT_EQ(s.sum, 4106U);
    EXPECT_EQ(s.minimum, 0U); EXPECT_EQ(s.maximum, 4096U);
    EXPECT_EQ(s.bins[0], 2U); EXPECT_EQ(s.bins[1], 1U);
    EXPECT_EQ(s.bins[2], 1U); EXPECT_EQ(s.bins[BINS-1], 1U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify sum carry without64 bit atomic.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, SumCarryWithout64BitAtomic).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, SumCarryWithout64BitAtomic)
{
    Stats stats; stats.record(UINT32_MAX); stats.record(1);
    Sample s; ASSERT_TRUE(stats.read(s)); EXPECT_EQ(s.sum, uint64_t(1) << 32);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify identity.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, Identity).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, Identity)
{
    EXPECT_EQ(classify(0, true, 0), Thread::MAIN);
    EXPECT_EQ(classify(12, false, 12), Thread::UTB_WORKER);
    EXPECT_EQ(classify(13, false, 12), Thread::OTHER);
    EXPECT_EQ(classify(0, false, 12), Thread::UNKNOWN);
    EXPECT_EQ(classify(12, false, 0), Thread::OTHER);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify overlap and unknown.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, OverlapAndUnknown).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, OverlapAndUnknown)
{
    Interval i;
    EXPECT_FALSE(i.overlap(90, 130));
    i.begin(100); EXPECT_FALSE(i.overlap(90, 130));
    i.end(120); EXPECT_TRUE(i.overlap(110, 130));
    EXPECT_FALSE(i.overlap(120, 130)); EXPECT_FALSE(i.overlap(80, 100));
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify timer wrap.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, TimerWrap).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, TimerWrap)
{
    Interval i; i.begin(UINT32_MAX - 20); i.end(5);
    EXPECT_TRUE(i.overlap(UINT32_MAX - 10, 10));
    EXPECT_FALSE(i.overlap(6, 20));
    EXPECT_EQ(uint32_t(5 - (UINT32_MAX - 20)), 26U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify atomic snapshot consistency.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, AtomicSnapshotConsistency).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, AtomicSnapshotConsistency)
{
    Stats stats; std::atomic<bool> done{false};
    std::thread writer([&]() {
        for (uint32_t i=0; i<100000; i++) {
            stats.record(7);
        } done.store(true);
    });
    uint32_t reads = 0;
    do {
        Sample s;
        if (stats.read(s)) {
            EXPECT_EQ(s.sum, uint64_t(s.count) * 7);
            EXPECT_EQ(s.bins[2], s.count); reads++;
        }
    } while (!done.load());
    writer.join(); EXPECT_GT(reads, 0U);
    Sample s; ASSERT_TRUE(stats.read(s)); EXPECT_EQ(s.count, 100000U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify lifecycle failure.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, LifecycleFailure).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, LifecycleFailure)
{
    const uint32_t attempts_before = create_attempts.load();
    const uint32_t failures_before = create_failures.load();
    creation(false, 3072); EXPECT_EQ(life.load(), uint32_t(Life::NOT_REQUESTED));
    creation(true, 3072); EXPECT_EQ(life.load(), uint32_t(Life::CREATE_ATTEMPT));
    creation_failed(); EXPECT_EQ(life.load(), uint32_t(Life::CREATE_FAILED));
    EXPECT_EQ(stack_requested.load(), 3072U);
    EXPECT_EQ(create_attempts.load(), attempts_before + 1U);
    EXPECT_EQ(create_failures.load(), failures_before + 1U);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class CountingSemaphore : public AP_HAL::Semaphore
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Take.
    ///
    ///  @param[in]  timeout Timeout.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool take(uint32_t timeout).
    //--------------------------------------------------------------------------------------------------------------------
    bool take(uint32_t timeout) override
    {
        takes++;
        last_timeout = timeout;
        return true;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Take nonblocking.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool take_nonblocking().
    //--------------------------------------------------------------------------------------------------------------------
    bool take_nonblocking() override
    {
        tries++;
        return true;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Give.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool give().
    //--------------------------------------------------------------------------------------------------------------------
    bool give() override
    {
        gives++;
        return true;
    }
    uint32_t takes = 0, tries = 0, gives = 0, last_timeout = UINT32_MAX;
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify original acquire release policy.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, OriginalAcquireReleasePolicy).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, OriginalAcquireReleasePolicy)
{
    CountingSemaphore sem;
    MutexState state;
    {
        MutexProbe probe(sem, state, 123);
        EXPECT_EQ(sem.takes, 1U);
        EXPECT_EQ(sem.gives, 0U);
        EXPECT_EQ(sem.tries, 0U);
        EXPECT_EQ(sem.last_timeout, uint32_t(HAL_SEMAPHORE_BLOCK_FOREVER));
    }
    EXPECT_EQ(sem.gives, 1U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify scheduler wrap and deadline.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, SchedulerWrapAndDeadline).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, SchedulerWrapAndDeadline)
{
    const uint32_t before = deadlines.load();
    loop(UINT32_MAX - 1000U, 2500);
    loop(1499U, 2500);
    loop(4000U, 2500);
    Sample period, jitter;
    ASSERT_TRUE(metrics[uint8_t(Metric::LOOP_PERIOD)].read(period));
    ASSERT_TRUE(metrics[uint8_t(Metric::LOOP_JITTER)].read(jitter));
    EXPECT_EQ(period.count, 2U);
    EXPECT_EQ(period.sum, 5001U);
    EXPECT_EQ(period.minimum, 2500U);
    EXPECT_EQ(period.maximum, 2501U);
    EXPECT_EQ(jitter.sum, 1U);
    EXPECT_EQ(deadlines.load(), before + 1U);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify host only overhead.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, HostOnlyOverhead).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, HostOnlyOverhead)
{
    Stats stats;
    const uint32_t iterations = 100000;
    const auto start = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < iterations; i++) {
        stats.record(i & 255U);
    }
    const auto middle = std::chrono::steady_clock::now();
    Sample sample;
    for (uint32_t i = 0; i < iterations; i++) {
        ASSERT_TRUE(stats.read(sample));
    }
    const auto end = std::chrono::steady_clock::now();
    const double record_ns = std::chrono::duration<double, std::nano>(middle - start).count() / iterations;
    const double read_ns = std::chrono::duration<double, std::nano>(end - middle).count() / iterations;
    printf("HOST_ONLY record_ns=%.2f read_ns=%.2f; includes host scheduling, not H743 timing\n", record_ns, read_ns);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify config frame consistency.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, ConfigFrameConsistency).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, ConfigFrameConsistency)
{
    Frame<4> frame;
    std::atomic<bool> done{false};
    std::thread writer([&]() {
        for (uint32_t i = 1; i < 20000; i++) {
            const uint32_t data[4] = {i, i, i, i}; frame.publish(data);
        }
        done.store(true);
    });
    do {
        uint32_t data[4];
        if (frame.read(data)) {
            EXPECT_EQ(data[0], data[1]); EXPECT_EQ(data[0], data[2]); EXPECT_EQ(data[0], data[3]);
        }
    } while (!done.load());
    writer.join();
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Verify stack watermark.
///
///  @param[in]  None
///  @return     None; GTest assertions report failures.
///
///  @details    Global exported function. Declaration/definition: TEST(UTBBench, StackWatermark).
//--------------------------------------------------------------------------------------------------------------------
TEST(UTBBench, StackWatermark)
{
    stack_sample(3072, 1200); stack_sample(3072, 1300); stack_sample(3072, 800);
    EXPECT_EQ(stack_total.load(), 3072U); EXPECT_EQ(stack_free_min.load(), 800U);
}
#endif
AP_GTEST_MAIN()

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
