// GNU GPL v3 or later.
/// @file test_utb_flight_bin.cpp
/// @brief Fixed queue, finite ARM lifecycle, wrap and actual packed packet admission.
/// @author Jiri Kovar
/// @version 2026-10-26
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_UTB/AP_UTB_ChainShadow.h>
#include <AP_UTB/FlightBinLogStructure.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
#if AP_UTB_FLIGHT_BIN_AVAILABLE
#include <cstring>
#include <thread>
class UTBFlightBinTestAccess
{
public:
    static void configure(AP_UTB_FlightBin &b, int8_t enable = 1, int8_t rate = 10, int16_t seconds = 60)
    {
        b.worker_ready(true);
        b._enable.set(enable);
        b._rate.set(rate);
        b._seconds.set(seconds);
        if (b.state() == AP_UTB_FlightBin::State::IDLE && !b._last_armed) {
            b.arm_edge(false, true, true, 0);
        }
    }
    static void unobserved_opt_in(AP_UTB_FlightBin &b)
    {
        b._enable.set(1);
        b.worker_ready(true);
    }
    static void enqueue(AP_UTB_FlightBin &b, const AP_UTB_FlightBin::Sample &s)
    {
        b.enqueue(s);
    }
    static void wrap(AP_UTB_FlightBin::Queue &q)
    {
        q._head.store(0xfffffffeU);
        q._tail.store(0xfffffffeU);
    }
};
TEST(FlightBin, QueueWholeSetAndWrap)
{
    AP_UTB_FlightBin::Queue q;
    AP_UTB_FlightBin::Sample in {}, out {};
    UTBFlightBinTestAccess::wrap(q);
    EXPECT_FALSE(q.pop(out));
    for (uint32_t i = 1; i <= 4; i++) {
        in.seq = i; in.pid[2][9] = float(i);
        ASSERT_TRUE(q.push(in)); EXPECT_EQ(q.producer_depth(), i);
    }
    EXPECT_FALSE(q.push(in)); EXPECT_EQ(q.depth(), 4U);
    for (uint32_t i = 1; i <= 4; i++) {
        ASSERT_TRUE(q.pop(out)); EXPECT_EQ(out.seq, i); EXPECT_FLOAT_EQ(out.pid[2][9], float(i));
    }
    EXPECT_EQ(q.depth(), 0U);
    EXPECT_FALSE(q.pop(out));
}
TEST(FlightBin, ConcurrentPublicationKeepsWholeCopies)
{
    AP_UTB_FlightBin::Queue queue;
    std::thread producer([&queue]() {
        AP_UTB_FlightBin::Sample value {};
        for (uint32_t i = 1; i <= 10000; i++) {
            value.seq = i; value.pid[2][9] = float(i);
            while (!queue.push(value)) {
                std::this_thread::yield();
            }
        }
    });
    AP_UTB_FlightBin::Sample value {};
    for (uint32_t i = 1; i <= 10000; i++) {
        while (!queue.pop(value)) {
            std::this_thread::yield();
        }
        EXPECT_EQ(value.seq, i); EXPECT_FLOAT_EQ(value.pid[2][9], float(i));
    }
    producer.join();
    EXPECT_EQ(queue.depth(), 0U);
}
TEST(FlightBin, RuntimeOffAndOptInBeforeArm)
{
    AP_UTB_FlightBin batched;
    batched.arm_edge(false, true, true, 1); // Last DISARMED cycle still had ENABLE=0.
    UTBFlightBinTestAccess::unobserved_opt_in(batched);
    EXPECT_FALSE(batched.arm_edge(true, true, true, 2)); // ARM+enable in one command batch cannot opt in.
    AP_UTB_FlightBin b;
    EXPECT_FALSE(b.clock_needed());
    EXPECT_FALSE(b.arm_edge(true, true, true, 10));
    UTBFlightBinTestAccess::configure(b);
    EXPECT_FALSE(b.arm_edge(true, true, true, 20)); // Enabling during ARM cannot trigger.
    EXPECT_FALSE(b.arm_edge(false, true, true, 30));
    EXPECT_TRUE(b.arm_edge(true, true, true, 40));
    b.stop(AP_UTB_FlightBin::Reason::DISARM);
    b.arm_edge(false, true, true, 50);
    EXPECT_FALSE(b.arm_edge(true, true, true, 60)); // Exactly one session per boot.
}
TEST(FlightBin, StopConditions)
{
    AP_UTB_FlightBin b;
    using R = AP_UTB_FlightBin::Reason;
    UTBFlightBinTestAccess::configure(b);
    ASSERT_TRUE(b.arm_edge(true, true, true, 100));
    EXPECT_EQ(b.limit(), 24000U);
    EXPECT_EQ(b.stop_condition(true, 0, true, 101), R::NONE);
    EXPECT_EQ(b.stop_condition(false, 0, true, 101), R::DISARM);
    EXPECT_EQ(b.stop_condition(true, 9, true, 101), R::MODE);
    EXPECT_EQ(b.stop_condition(true, 0, false, 101), R::UNSUPPORTED);
    EXPECT_EQ(b.stop_condition(true, 0, true, 101, true), R::FAILSAFE);
    EXPECT_EQ(b.stop_condition(true, 0, true, 60000100), R::TIME);
    UTBFlightBinTestAccess::configure(b, 0);
    EXPECT_EQ(b.stop_condition(true, 0, true, 101), R::DISABLED);
    UTBFlightBinTestAccess::configure(b, 1, 20);
    EXPECT_EQ(b.stop_condition(true, 0, true, 101), R::CONFIG);
}
TEST(FlightBin, RejectedConfigurationAndConflict)
{
    AP_UTB_FlightBin bad, conflict, unsupported;
    UTBFlightBinTestAccess::configure(bad, 1, 127);
    EXPECT_FALSE(bad.arm_edge(true, true, true, 10));
    EXPECT_EQ(bad.state(), AP_UTB_FlightBin::State::STOPPED);
    UTBFlightBinTestAccess::configure(conflict);
    EXPECT_FALSE(conflict.arm_edge(true, true, false, 10));
    UTBFlightBinTestAccess::configure(unsupported);
    EXPECT_FALSE(unsupported.arm_edge(true, false, true, 10));
    unsupported.stop(AP_UTB_FlightBin::Reason::UNSUPPORTED);
    EXPECT_EQ(unsupported.counts().unsupported, 1U);
}
TEST(FlightBin, WorkerCreationFailureDoesNotStartProducer)
{
    AP_UTB_FlightBin b;
    UTBFlightBinTestAccess::configure(b);
    b.worker_ready(false);
    EXPECT_FALSE(b.arm_edge(true, true, true, 10));
    EXPECT_EQ(b.state(), AP_UTB_FlightBin::State::STOPPED);
    EXPECT_EQ(b.counts().queued, 0U);
}
TEST(FlightBin, AbsoluteClockReconstruction)
{
    uint64_t value = 0;
    ASSERT_TRUE(AP_UTB_FlightBin::expand_time(0xfffffff0U, 0x100000020ULL, value));
    EXPECT_EQ(value, 0xfffffff0ULL);
    ASSERT_TRUE(AP_UTB_FlightBin::expand_time(0x30U, 0x100000020ULL, value));
    EXPECT_EQ(value, 0x100000030ULL); // Capture may be later than the loop's clock sample.
    ASSERT_TRUE(AP_UTB_FlightBin::expand_time(0x10U, 0x300000020ULL, value));
    EXPECT_EQ(value, 0x300000010ULL);
    EXPECT_FALSE(AP_UTB_FlightBin::expand_time(0xfffffff0U, 0x20ULL, value));
}

class UTBChainTestAccess
{
public:
    static void attempts(UTB_ChainShadowEngine &e, uint32_t n)
    {
        e._stats.attempts = n;
    }
};
TEST(FlightBin, DecimationAndExplicitInvalid)
{
    UTB_ChainShadowEngine engine;
    AP_UTB_FlightBin ten, twenty;
    UTBFlightBinTestAccess::configure(ten);
    UTBFlightBinTestAccess::configure(twenty, 1, 20);
    ten.arm_edge(true, true, true, 1); twenty.arm_edge(true, true, true, 1);
    for (uint32_t i = 1; i <= 400; i++) {
        UTBChainTestAccess::attempts(engine, i);
        ten.sample(engine, i, i*2500U); twenty.sample(engine, i, i*2500U);
    }
    EXPECT_EQ(ten.counts().requested, 10U); EXPECT_EQ(twenty.counts().requested, 20U);
    EXPECT_EQ(ten.counts().invalid, 10U); EXPECT_EQ(twenty.counts().invalid, 20U);
    EXPECT_EQ(ten.counts().queued, 0U); // Empty/unpaired engine is never fabricated as a valid record.
}
TEST(FlightBin, SlowConsumerDropsWholeNewSet)
{
    AP_UTB_FlightBin b;
    AP_UTB_FlightBin::Sample s {};
    for (uint32_t i = 0; i < 10; i++) {
        UTBFlightBinTestAccess::enqueue(b, s);
    }
    EXPECT_EQ(b.counts().queued, 4U); EXPECT_EQ(b.counts().dropped, 6U);
    EXPECT_EQ(b.counts().hwm, 4U); EXPECT_EQ(b.counts().depth, 4U);
}

class PacketWriter
{
public:
    unsigned count = 0, bytes = 0, reject = 100;
    bool WriteBlock_first_succeed(const void *data, uint16_t size)
    {
        const auto &p = *static_cast<const UTBFlightLog::PID *>(data);
        EXPECT_EQ(p.time_us, 0x100000010ULL); EXPECT_EQ(p.seq, 37U);
        if (count < 6) {
            EXPECT_EQ(p.axis, count / 2);
        }
        bytes += size;
        return count++ != reject;
    }
};
TEST(FlightBin, PacketPackingAndPartialAdmission)
{
    AP_UTB_FlightBin::Sample s {};
    PacketWriter good, partial;
    s.seq = 37; s.time_us = 0x100000010ULL;
    ASSERT_TRUE(UTBFlightLog::write_sample(s, good));
    EXPECT_EQ(good.count, 7U); EXPECT_EQ(good.bytes, 450U);
    partial.reject = 2;
    EXPECT_FALSE(UTBFlightLog::write_sample(s, partial));
    EXPECT_EQ(partial.count, 7U); // No short circuit or unbounded retry.
}
class DelayedWriter
{
public:
    std::atomic<bool> entered {false}, released {false};
    bool WriteBlock_first_succeed(const void *, uint16_t)
    {
        entered.store(true, std::memory_order_release);
        while (!released.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        return false; // The transport does not depend on the backend making progress or admitting data.
    }
};
TEST(FlightBin, BlockedBackendCannotHoldAQueueSlot)
{
    AP_UTB_FlightBin::Queue queue;
    AP_UTB_FlightBin::Sample input {};
    DelayedWriter writer;
    ASSERT_TRUE(queue.push(input));
    std::thread consumer([&queue, &writer]() {
        AP_UTB_FlightBin::Sample copy {};
        if (queue.pop(copy)) {
            EXPECT_FALSE(UTBFlightLog::write_sample(copy, writer));
        }
    });
    while (!writer.entered.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    // The consumer's logger is blocked, but its original queue slot is already released.
    for (uint32_t i = 0; i < 4; i++) {
        EXPECT_TRUE(queue.push(input));
    }
    EXPECT_FALSE(queue.push(input));
    writer.released.store(true, std::memory_order_release);
    consumer.join();
    EXPECT_EQ(queue.depth(), 4U);
}
#endif
AP_GTEST_MAIN()
