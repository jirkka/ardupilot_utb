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
#pragma once
#include <AP_HAL/AP_HAL.h>
#ifndef AP_UTB_BENCH_ENABLED
#define AP_UTB_BENCH_ENABLED 0
#endif
#if AP_UTB_BENCH_ENABLED
#include <atomic>

namespace UTBBench
{
static_assert(ATOMIC_INT_LOCK_FREE == 2, "Benchmark requires lock-free 32-bit atomics");
constexpr uint8_t BINS = 12;
enum class Thread : uint8_t { MAIN, UTB_WORKER, OTHER, UNKNOWN };
enum class Life : uint8_t { NOT_REQUESTED, CREATE_ATTEMPT, CREATE_FAILED, RUNNING };
enum class Metric : uint8_t {
    MAIN_ACQUIRE,
    WORKER_ACQUIRE,
    WORKER_HOLD,
    OTHER_ACQUIRE,
    UNKNOWN_ACQUIRE,
    LOOP_PERIOD,
    LOOP_JITTER,
    PRODUCER,
    CONSUMER,
    WORKER_PASS,
    EXPORT,
    COUNT
};
struct Sample {
    uint32_t count = 0;
    uint64_t sum = 0;
    uint32_t minimum = 0;
    uint32_t maximum = 0;
    uint32_t last = 0;
    uint32_t bins[BINS] {};
};
// One writer per metric. All published words are atomic, including histogram.
// Readers make one attempt; no retry/spin or allocation.
class Stats
{
public:
    void record(uint32_t value);
    bool read(Sample &out) const;
    static uint8_t bin(uint32_t value);
private:
    std::atomic<uint32_t> _seq{0}, _count{0}, _lo{0}, _hi{0};
    std::atomic<uint32_t> _minimum{UINT32_MAX}, _maximum{0}, _last{0};
    std::atomic<uint32_t> _bins[BINS] {};
};
template<uint8_t N> class Frame
{
public:
    void publish(const uint32_t (&data)[N])
    {
        const uint32_t seq = _seq.load();
        _seq.store(seq + 1);
        for (uint8_t i = 0; i < N; i++) {
            _data[i].store(data[i]);
        }
        _seq.store(seq + 2);
    }
    bool read(uint32_t (&out)[N]) const
    {
        const uint32_t seq = _seq.load();
        if (seq & 1U) {
            return false;
        }
        uint32_t data[N];
        for (uint8_t i = 0; i < N; i++) {
            data[i] = _data[i].load();
        }
        if (seq != _seq.load()) {
            return false;
        }
        for (uint8_t i = 0; i < N; i++) {
            out[i] = data[i];
        }
        return true;
    }
private:
    std::atomic<uint32_t> _seq{0};
    std::atomic<uint32_t> _data[N] {};
};
class Interval
{
public:
    void begin(uint32_t start);
    void end(uint32_t finish);
    bool overlap(uint32_t start, uint32_t finish) const;
    uint32_t generation() const
    {
        return _seq.load();
    }
private:
    std::atomic<uint32_t> _seq{0}, _start{0}, _finish{0};
};
struct MutexState {
    explicit MutexState(uint32_t backend = 0);
    Interval worker;
    Stats other, unknown;
};
bool read_metric(uint8_t metric, Sample &out);
extern std::atomic<uint32_t> measured_backend;
using Identity = uintptr_t (*)();
void init(Identity identity);
Thread classify(uintptr_t id, bool main, uintptr_t worker);
Thread current_thread();
extern Stats metrics[uint8_t(Metric::COUNT)];
extern std::atomic<uint32_t> wait_limit, exceeds, collisions, collision_max, unknown;
extern std::atomic<uint32_t> task_overruns, deadlines, export_skips;
extern std::atomic<uint32_t> life, worker_loops, worker_begin, worker_done;
extern std::atomic<uint32_t> stack_requested, stack_total, stack_free_min;
extern std::atomic<uint32_t> exporter_life, create_attempts, create_failures;
void creation(bool requested, uint32_t stack);
void creation_failed();
void worker_enter();
void worker_start(uint32_t now);
void worker_finish(uint32_t now);
void stack_sample(uint32_t total, uint32_t free);
void loop(uint32_t now, uint32_t nominal);
void task_overrun();
class Duration
{
public:
    explicit Duration(Metric metric) : _metric(metric), _start(AP_HAL::micros()) {}
    ~Duration()
    {
        metrics[uint8_t(_metric)].record(AP_HAL::micros() - _start);
    }
private:
    Metric _metric;
    uint32_t _start;
};
class MutexProbe
{
public:
    MutexProbe(AP_HAL::Semaphore &sem, MutexState &state, uint32_t line);
    ~MutexProbe();
private:
    AP_HAL::Semaphore &_sem;
    MutexState &_state;
    Thread _thread;
    uint32_t _acquire;
    uint32_t _taken;
    bool _collision = false;
};
}
#endif // AP_UTB_BENCH_ENABLED
