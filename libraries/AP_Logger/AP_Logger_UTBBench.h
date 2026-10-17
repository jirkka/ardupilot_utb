//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_Logger_UTBBench.h
///  @brief      BENCH default0, typy a fixed atomic API pro instrumentaci.
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
#pragma once
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include <AP_HAL/AP_HAL.h>
#ifndef AP_UTB_BENCH_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
#define AP_UTB_BENCH_ENABLED 0
#endif
#if AP_UTB_BENCH_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include <atomic>

namespace UTBBench
{
static_assert(ATOMIC_INT_LOCK_FREE == 2, "Benchmark requires lock-free 32-bit atomics");
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
constexpr uint8_t BINS = 12;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
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
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Record.
    ///
    ///  @param[in]  value Value supplied to this operation.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void record(uint32_t value).
    //--------------------------------------------------------------------------------------------------------------------
    void record(uint32_t value);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Read.
    ///
    ///  @param[out] out Out.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool read(Sample &out).
    //--------------------------------------------------------------------------------------------------------------------
    bool read(Sample &out) const;
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Bin.
    ///
    ///  @param[in]  value Value supplied to this operation.
    ///  @return     uint8_t.
    ///
    ///  @details    Public class member. Declaration/definition: static uint8_t bin(uint32_t value).
    //--------------------------------------------------------------------------------------------------------------------
    static uint8_t bin(uint32_t value);
private:
    std::atomic<uint32_t> _seq{0}, _count{0}, _lo{0}, _hi{0};
    std::atomic<uint32_t> _minimum{UINT32_MAX}, _maximum{0}, _last{0};
    std::atomic<uint32_t> _bins[BINS] {};
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
template<uint8_t N> class Frame
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Publish.
    ///
    ///  @param[in]  data Data.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void publish(const uint32_t (&data)[N]).
    //--------------------------------------------------------------------------------------------------------------------
    void publish(const uint32_t (&data)[N])
    {
        const uint32_t seq = _seq.load();
        _seq.store(seq + 1);
        for (uint8_t i = 0; i < N; i++) {
            _data[i].store(data[i]);
        }
        _seq.store(seq + 2);
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Read.
    ///
    ///  @param[out] out Out.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool read(uint32_t (&out)[N]).
    //--------------------------------------------------------------------------------------------------------------------
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
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class Interval
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Begin.
    ///
    ///  @param[in]  start Start.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void begin(uint32_t start).
    //--------------------------------------------------------------------------------------------------------------------
    void begin(uint32_t start);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      End.
    ///
    ///  @param[in]  finish Finish.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void end(uint32_t finish).
    //--------------------------------------------------------------------------------------------------------------------
    void end(uint32_t finish);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Overlap.
    ///
    ///  @param[in]  start Start.
    ///  @param[in]  finish Finish.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool overlap(uint32_t start, uint32_t finish).
    //--------------------------------------------------------------------------------------------------------------------
    bool overlap(uint32_t start, uint32_t finish) const;
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Generation.
    ///
    ///  @param[in]  None
    ///  @return     uint32_t.
    ///
    ///  @details    Public class member. Declaration/definition: uint32_t generation().
    //--------------------------------------------------------------------------------------------------------------------
    uint32_t generation() const
    {
        return _seq.load();
    }
private:
    std::atomic<uint32_t> _seq{0}, _start{0}, _finish{0};
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
struct MutexState {
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Mutex state.
    ///
    ///  @param[in]  backend Backend.
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: explicit MutexState(uint32_t backend = 0).
    //--------------------------------------------------------------------------------------------------------------------
    explicit MutexState(uint32_t backend = 0);
    Interval worker;
    Stats other, unknown;
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Read metric.
///
///  @param[in]  metric Metric.
///  @param[out] out Out.
///  @return     bool.
///
///  @details    Global exported function. Declaration/definition: bool read_metric(uint8_t metric, Sample &out).
//--------------------------------------------------------------------------------------------------------------------
bool read_metric(uint8_t metric, Sample &out);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
extern std::atomic<uint32_t> measured_backend;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
using Identity = uintptr_t (*)();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Init.
///
///  @param[in]  identity Identity.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void init(Identity identity).
//--------------------------------------------------------------------------------------------------------------------
void init(Identity identity);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Attribute a thread identity without inventing a known owner for ambiguous data.
///
///  @param[in]  id Id.
///  @param[in]  main Main.
///  @param[in]  worker Worker.
///  @return     Thread.
///
///  @details    Global exported function. Declaration/definition: Thread classify(uintptr_t id, bool main, uintptr_t
///              worker).
//--------------------------------------------------------------------------------------------------------------------
Thread classify(uintptr_t id, bool main, uintptr_t worker);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Classify the caller as MAIN, UTB_WORKER, OTHER or UNKNOWN.
///
///  @param[in]  None
///  @return     Thread.
///
///  @details    Global exported function. Declaration/definition: Thread current_thread().
//--------------------------------------------------------------------------------------------------------------------
Thread current_thread();
extern Stats metrics[uint8_t(Metric::COUNT)];
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
extern std::atomic<uint32_t> wait_limit, exceeds, collisions, collision_max, unknown;
extern std::atomic<uint32_t> task_overruns, deadlines, export_skips;
extern std::atomic<uint32_t> life, worker_loops, worker_begin, worker_done;
extern std::atomic<uint32_t> stack_requested, stack_total, stack_free_min;
extern std::atomic<uint32_t> exporter_life, create_attempts, create_failures;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Creation.
///
///  @param[in]  requested Requested.
///  @param[in]  stack Stack.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void creation(bool requested, uint32_t stack).
//--------------------------------------------------------------------------------------------------------------------
void creation(bool requested, uint32_t stack);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Creation failed.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void creation_failed().
//--------------------------------------------------------------------------------------------------------------------
void creation_failed();
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Worker enter.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void worker_enter().
//--------------------------------------------------------------------------------------------------------------------
void worker_enter();
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Worker start.
///
///  @param[in]  now Now.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void worker_start(uint32_t now).
//--------------------------------------------------------------------------------------------------------------------
void worker_start(uint32_t now);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Worker finish.
///
///  @param[in]  now Now.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void worker_finish(uint32_t now).
//--------------------------------------------------------------------------------------------------------------------
void worker_finish(uint32_t now);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Stack sample.
///
///  @param[in]  total Total.
///  @param[in]  free Free.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void stack_sample(uint32_t total, uint32_t free).
//--------------------------------------------------------------------------------------------------------------------
void stack_sample(uint32_t total, uint32_t free);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Loop.
///
///  @param[in]  now Now.
///  @param[in]  nominal Nominal.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void loop(uint32_t now, uint32_t nominal).
//--------------------------------------------------------------------------------------------------------------------
void loop(uint32_t now, uint32_t nominal);
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Task overrun.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void task_overrun().
//--------------------------------------------------------------------------------------------------------------------
void task_overrun();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class Duration
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Duration.
    ///
    ///  @param[in]  metric Metric.
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: explicit Duration(Metric metric).
    //--------------------------------------------------------------------------------------------------------------------
    explicit Duration(Metric metric) : _metric(metric), _start(AP_HAL::micros()) {}
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      ~duration.
    ///
    ///  @param[in]  None
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: ~Duration().
    //--------------------------------------------------------------------------------------------------------------------
    ~Duration()
    {
        metrics[uint8_t(_metric)].record(AP_HAL::micros() - _start);
    }
private:
    Metric _metric;
    uint32_t _start;
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
class MutexProbe
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Mutex probe.
    ///
    ///  @param[in]  sem Sem.
    ///  @param[in]  state Sampled attitude, gyro and validity metadata.
    ///  @param[in]  line Line.
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: MutexProbe(AP_HAL::Semaphore &sem, MutexState &state,
    ///              uint32_t line).
    //--------------------------------------------------------------------------------------------------------------------
    MutexProbe(AP_HAL::Semaphore &sem, MutexState &state, uint32_t line);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      ~mutex probe.
    ///
    ///  @param[in]  None
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: ~MutexProbe().
    //--------------------------------------------------------------------------------------------------------------------
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
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
