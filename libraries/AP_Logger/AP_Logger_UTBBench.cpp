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
#include "AP_Logger_UTBBench.h"
#if AP_UTB_BENCH_ENABLED
#include <AP_Math/AP_Math.h>
extern const AP_HAL::HAL &hal;
namespace UTBBench
{
Stats metrics[uint8_t(Metric::COUNT)];
std::atomic<uint32_t> measured_backend{0};
static std::atomic<MutexState *> primary_mutex{nullptr};
MutexState::MutexState(uint32_t backend)
{
    // Backend instances are constructed by the main thread during logger init.
    if (backend != 0 && primary_mutex.load() == nullptr) {
        primary_mutex.store(this);
        measured_backend.store(backend);
    }
}
bool read_metric(uint8_t metric, Sample &out)
{
    if (metric == uint8_t(Metric::OTHER_ACQUIRE) || metric == uint8_t(Metric::UNKNOWN_ACQUIRE)) {
        const MutexState *state = primary_mutex.load();
        if (state == nullptr) {
            out = {};
            return true;
        }
        return (metric == uint8_t(Metric::OTHER_ACQUIRE) ? state->other : state->unknown).read(out);
    }
    return metric < uint8_t(Metric::COUNT) && metrics[metric].read(out);
}
std::atomic<uint32_t> wait_limit{0}, exceeds{0}, collisions{0}, collision_max{0}, unknown{0};
std::atomic<uint32_t> task_overruns{0}, deadlines{0}, export_skips{0};
std::atomic<uint32_t> life{0}, worker_loops{0}, worker_begin{0}, worker_done{0};
std::atomic<uint32_t> stack_requested{0}, stack_total{0}, stack_free_min{UINT32_MAX};
std::atomic<uint32_t> exporter_life{0}, create_attempts{0}, create_failures{0};
static std::atomic<Identity> identity{nullptr};
static std::atomic<uintptr_t> worker_id{0};
static_assert(ATOMIC_POINTER_LOCK_FREE == 2, "Benchmark identity must not lock");
static void increment(std::atomic<uint32_t> &v)
{
    v.store(v.load() + 1);
}
uint8_t Stats::bin(uint32_t value)
{
    uint8_t b = 0;
    while (value > 1 && b < BINS - 1) {
        value >>= 1;
        b++;
    }
    return b;
}
void Stats::record(uint32_t value)
{
    const uint32_t seq = _seq.load();
    _seq.store(seq + 1);
    const uint32_t lo = _lo.load();
    _lo.store(lo + value);
    if (lo + value < lo) {
        increment(_hi);
    }
    const uint32_t count = _count.load();
    _count.store(count + 1);
    _minimum.store(MIN(_minimum.load(), value));
    _maximum.store(MAX(_maximum.load(), value));
    _last.store(value);
    increment(_bins[bin(value)]);
    _seq.store(seq + 2);
}
bool Stats::read(Sample &out) const
{
    const uint32_t before = _seq.load();
    if (before & 1U) {
        return false;
    }
    Sample s;
    s.count = _count.load();
    s.sum = (uint64_t(_hi.load()) << 32) | _lo.load();
    s.minimum = s.count == 0 ? 0 : _minimum.load();
    s.maximum = _maximum.load();
    s.last = _last.load();
    for (uint8_t i = 0; i < BINS; i++) {
        s.bins[i] = _bins[i].load();
    }
    if (before != _seq.load()) {
        return false;
    }
    out = s;
    return true;
}
void Interval::begin(uint32_t start)
{
    _seq.store(_seq.load() + 1);
    _start.store(start);
}
void Interval::end(uint32_t finish)
{
    _finish.store(finish);
    _seq.store(_seq.load() + 1);
}
bool Interval::overlap(uint32_t start, uint32_t finish) const
{
    if (finish - start >= 0x80000000U) {
        return false;
    }
    const uint32_t seq = _seq.load();
    if (seq == 0 || (seq & 1U)) {
        return false;
    }
    const uint32_t a = _start.load(), b = _finish.load();
    if (seq != _seq.load()) {
        return false;
    }
    // Signed ordering is valid for intervals shorter than 2^31 microseconds.
    return int32_t(finish - a) > 0 && int32_t(b - start) > 0;
}
void init(Identity fn)
{
    identity.store(fn);
}
Thread classify(uintptr_t id, bool main, uintptr_t worker)
{
    if (main) {
        return Thread::MAIN;
    }
    if (id == 0) {
        return Thread::UNKNOWN;
    }
    return worker != 0 && id == worker ? Thread::UTB_WORKER : Thread::OTHER;
}
Thread current_thread()
{
    const Identity fn = identity.load();
    return classify(fn == nullptr ? 0 : fn(), hal.scheduler->in_main_thread(), worker_id.load());
}
void creation(bool requested, uint32_t stack)
{
    if (requested) {
        increment(create_attempts);
    }
    stack_requested.store(requested ? stack : 0);
    life.store(uint32_t(requested ? Life::CREATE_ATTEMPT : Life::NOT_REQUESTED));
}
void creation_failed()
{
    increment(create_failures);
    life.store(uint32_t(Life::CREATE_FAILED));
}
void worker_enter()
{
    const Identity fn = identity.load();
    worker_id.store(fn == nullptr ? 0 : fn());
    life.store(uint32_t(Life::RUNNING));
}
void worker_start(uint32_t now)
{
    worker_begin.store(now);
}
void worker_finish(uint32_t now)
{
    worker_done.store(now);
    increment(worker_loops);
}
void stack_sample(uint32_t total, uint32_t free)
{
    stack_total.store(total);
    stack_free_min.store(MIN(stack_free_min.load(), free));
}
void loop(uint32_t now, uint32_t nominal)
{
    static uint32_t previous;
    static bool primed;
    if (primed) {
        const uint32_t dt = now - previous;
        metrics[uint8_t(Metric::LOOP_PERIOD)].record(dt);
        metrics[uint8_t(Metric::LOOP_JITTER)].record(dt > nominal ? dt - nominal : nominal - dt);
        if (dt > nominal) {
            increment(deadlines);
        }
    }
    previous = now;
    primed = true;
}
void task_overrun()
{
    increment(task_overruns);
}
MutexProbe::MutexProbe(AP_HAL::Semaphore &sem, MutexState &state, uint32_t line) :
    _sem(sem), _state(state), _thread(current_thread())
{
    const uint32_t generation = _state.worker.generation();
    // Preserve WithSemaphore watchdog attribution as well as acquire policy.
    if (_thread == Thread::MAIN) {
        hal.util->persistent_data.semaphore_line = line;
    }
    const uint32_t t0 = AP_HAL::micros();
    _sem.take_blocking();
    _taken = AP_HAL::micros();
    if (_thread == Thread::MAIN) {
        hal.util->persistent_data.semaphore_line = 0;
    }
    _acquire = _taken - t0;
    if (_thread == Thread::UTB_WORKER) {
        _state.worker.begin(_taken);
    } else if (_thread == Thread::MAIN) {
        _collision = generation != _state.worker.generation() && _state.worker.overlap(t0, _taken);
    } else {
        // OTHER/UNKNOWN can have multiple writers; serialize their Stats
        // in per-mutex storage with the existing lock, adding no new lock.
        (_thread == Thread::OTHER ? _state.other : _state.unknown).record(_acquire);
    }
}
MutexProbe::~MutexProbe()
{
    if (_thread == Thread::UTB_WORKER) {
        // Conservative overlap endpoint is published before unlock. The final
        // hold timestamp remains immediately adjacent to the original give().
        _state.worker.end(AP_HAL::micros());
    }
    const uint32_t t2 = AP_HAL::micros();
    _sem.give();
    // Counter work is outside the measured mutex. It never calls logging.
    if (_thread == Thread::MAIN) {
        metrics[uint8_t(Metric::MAIN_ACQUIRE)].record(_acquire);
        const uint32_t limit = wait_limit.load();
        if (limit != 0 && _acquire > limit) {
            increment(exceeds);
        }
        if (_collision) {
            increment(collisions);
            collision_max.store(MAX(collision_max.load(), _acquire));
        } else {
            // Includes uncontended acquisitions: lack of overlap is not proof.
            increment(unknown);
        }
    } else if (_thread == Thread::UTB_WORKER) {
        metrics[uint8_t(Metric::WORKER_ACQUIRE)].record(_acquire);
        metrics[uint8_t(Metric::WORKER_HOLD)].record(t2 - _taken);
    }
}
}
#endif // AP_UTB_BENCH_ENABLED
