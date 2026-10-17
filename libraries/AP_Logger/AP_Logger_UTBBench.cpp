//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_Logger_UTBBench.cpp
///  @brief      Fixed lock-free stats/identity/interval atribuce a mutex timing.
///  @author     Jiri Kovar
///  @version    2026-10-17
///  @date       2026-10-17
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
#include "AP_Logger_UTBBench.h"
#if AP_UTB_BENCH_ENABLED
#include <AP_Math/AP_Math.h>
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
extern const AP_HAL::HAL &hal;
namespace UTBBench
{
// Pevné statistiky jednotlivých měřených veličin.
Stats metrics[uint8_t(Metric::COUNT)];
// Identifikátor backendu, jehož mutex se skutečně měří.
std::atomic<uint32_t> measured_backend{0};
// Volitelný první registrovaný backend; ukazatel zůstává nutný do registrace.
static std::atomic<MutexState *> primary_mutex{nullptr};
// Mez čekání a počty potvrzených nebo nejednoznačných kolizí.
// Nastavitelná mez main acquire latence v mikrosekundách; nula vypíná počítání překročení.
std::atomic<uint32_t> wait_limit{0};
// Počet main acquire latencí nad nastavenou mezí.
std::atomic<uint32_t> exceeds{0};
// Počet potvrzených časových překryvů main acquire s držením mutexu workerem.
std::atomic<uint32_t> collisions{0};
// Největší main acquire latence při potvrzeném překryvu v mikrosekundách.
std::atomic<uint32_t> collision_max{0};
// Počet main acquire událostí bez jednoznačně potvrzeného překryvu.
std::atomic<uint32_t> unknown{0};
// Scheduler události a vynechané diagnostické exporty.
// Počet scheduler úloh, které překročily svůj jednotlivý časový rozpočet.
std::atomic<uint32_t> task_overruns{0};
// Počet period main loop delších než nominální perioda; není důkaz hard deadline miss.
std::atomic<uint32_t> deadlines{0};
// Počet vynechaných diagnostických exportů.
std::atomic<uint32_t> export_skips{0};
// Životní cyklus a monotónní heartbeat workeru.
// Aktuální fáze životního cyklu UTB workeru podle enum Life.
std::atomic<uint32_t> life{0};
// Počet dokončených průchodů workeru.
std::atomic<uint32_t> worker_loops{0};
// Monotónní čas začátku posledního průchodu workeru v mikrosekundách.
std::atomic<uint32_t> worker_begin{0};
// Monotónní čas dokončení posledního průchodu workeru v mikrosekundách.
std::atomic<uint32_t> worker_done{0};
// Přidělený stack a nejmenší pozorovaná volná rezerva v bytech.
// Požadovaná velikost stacku workeru v bytech.
std::atomic<uint32_t> stack_requested{0};
// Skutečná velikost přiděleného pracovního prostoru podle HAL v bytech.
std::atomic<uint32_t> stack_total{0};
// Nejmenší pozorovaná volná rezerva stacku; UINT32_MAX znamená dosud nezměřeno.
std::atomic<uint32_t> stack_free_min{UINT32_MAX};
// Stav exportéru a výsledky pokusů o vytvoření workeru.
// Stav samostatného diagnostického exportéru.
std::atomic<uint32_t> exporter_life{0};
// Počet požadavků na vytvoření workeru.
std::atomic<uint32_t> create_attempts{0};
// Počet neúspěšných pokusů o vytvoření workeru.
std::atomic<uint32_t> create_failures{0};
// Volitelná HAL funkce pro zjištění identity aktuálního vlákna.
static std::atomic<Identity> identity{nullptr};
// Identita workeru po jeho skutečném spuštění.
static std::atomic<uintptr_t> worker_id{0};
static_assert(ATOMIC_POINTER_LOCK_FREE == 2, "Benchmark identity must not lock");
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Mutex state.
///
///  @param[in]  backend Backend.
///  @return     None (constructor/destructor).
///
///  @details    Public class member. Declaration/definition: MutexState::MutexState(uint32_t backend).
//--------------------------------------------------------------------------------------------------------------------
MutexState::MutexState(uint32_t backend)
{
    // Backend instances are constructed by the main thread during logger init.
    if (backend != 0 && primary_mutex.load() == nullptr) {
        primary_mutex.store(this);
        measured_backend.store(backend);
    }
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Read metric.
///
///  @param[in]  metric Metric.
///  @param[out] out Out.
///  @return     bool.
///
///  @details    Global exported function. Declaration/definition: bool read_metric(uint8_t metric, Sample &out).
//--------------------------------------------------------------------------------------------------------------------
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
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Increment.
///
///  @param[in]  v V.
///  @return     None.
///
///  @details    File-local helper (internal linkage). Declaration/definition: static void increment(std::atomic<uint32_t>
///              &v).
//--------------------------------------------------------------------------------------------------------------------
static void increment(std::atomic<uint32_t> &v)
{
    v.store(v.load() + 1);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Bin.
///
///  @param[in]  value Value supplied to this operation.
///  @return     uint8_t.
///
///  @details    Public class member. Declaration/definition: uint8_t Stats::bin(uint32_t value).
//--------------------------------------------------------------------------------------------------------------------
uint8_t Stats::bin(uint32_t value)
{
    // Index histogramového koše odpovídající velikosti vzorku.
    uint8_t b;

    b = 0;
    while (value > 1 && b < BINS - 1) {
        value >>= 1;
        b++;
    }
    return b;
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Record.
///
///  @param[in]  value Value supplied to this operation.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void Stats::record(uint32_t value).
//--------------------------------------------------------------------------------------------------------------------
void Stats::record(uint32_t value)
{
    // Verze atomicky publikovaného diagnostického snapshotu.
    uint32_t seq;
    // Spodní 32bitová část součtu diagnostických časů.
    uint32_t lo;
    // Počet naměřených hodnot v diagnostické statistice.
    uint32_t count;

    seq = _seq.load();
    _seq.store(seq + 1);
    lo = _lo.load();
    _lo.store(lo + value);
    if (lo + value < lo) {
        increment(_hi);
    }
    count = _count.load();
    _count.store(count + 1);
    _minimum.store(MIN(_minimum.load(), value));
    _maximum.store(MAX(_maximum.load(), value));
    _last.store(value);
    increment(_bins[bin(value)]);
    _seq.store(seq + 2);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Read.
///
///  @param[out] out Out.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool Stats::read(Sample &out).
//--------------------------------------------------------------------------------------------------------------------
bool Stats::read(Sample &out) const
{
    // Verze snapshotu odečtená před čtením jeho dat.
    uint32_t before;
    // Index průchodu pevnou kolekcí.
    uint8_t i;

    before = _seq.load();
    if (before & 1U) {
        return false;
    }
    Sample s;
    s.count = _count.load();
    s.sum = (uint64_t(_hi.load()) << 32) | _lo.load();
    s.minimum = s.count == 0 ? 0 : _minimum.load();
    s.maximum = _maximum.load();
    s.last = _last.load();
    for (i = 0; i < BINS; i++) {
        s.bins[i] = _bins[i].load();
    }
    if (before != _seq.load()) {
        return false;
    }
    out = s;
    return true;
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Begin.
///
///  @param[in]  start Start.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void Interval::begin(uint32_t start).
//--------------------------------------------------------------------------------------------------------------------
void Interval::begin(uint32_t start)
{
    _seq.store(_seq.load() + 1);
    _start.store(start);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      End.
///
///  @param[in]  finish Finish.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void Interval::end(uint32_t finish).
//--------------------------------------------------------------------------------------------------------------------
void Interval::end(uint32_t finish)
{
    _finish.store(finish);
    _seq.store(_seq.load() + 1);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Overlap.
///
///  @param[in]  start Start.
///  @param[in]  finish Finish.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool Interval::overlap(uint32_t start, uint32_t finish).
//--------------------------------------------------------------------------------------------------------------------
bool Interval::overlap(uint32_t start, uint32_t finish) const
{
    // Verze atomicky publikovaného diagnostického snapshotu.
    uint32_t seq;
    // Začátek posledního úplného intervalu držení mutexu v mikrosekundách.
    uint32_t a;
    // Konec posledního úplného intervalu držení mutexu.
    uint32_t b;

    if (finish - start >= 0x80000000U) {
        return false;
    }
    seq = _seq.load();
    if (seq == 0 || (seq & 1U)) {
        return false;
    }
    a = _start.load();
    b = _finish.load();
    if (seq != _seq.load()) {
        return false;
    }
    // Signed ordering is valid for intervals shorter than 2^31 microseconds.
    return int32_t(finish - a) > 0 && int32_t(b - start) > 0;
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Init.
///
///  @param[in]  fn Fn.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void init(Identity fn).
//--------------------------------------------------------------------------------------------------------------------
void init(Identity fn)
{
    identity.store(fn);
}
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
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Classify the caller as MAIN, UTB_WORKER, OTHER or UNKNOWN.
///
///  @param[in]  None
///  @return     Thread.
///
///  @details    Global exported function. Declaration/definition: Thread current_thread().
//--------------------------------------------------------------------------------------------------------------------
Thread current_thread()
{
    const Identity fn = identity.load();
    return classify(fn == nullptr ? 0 : fn(), hal.scheduler->in_main_thread(), worker_id.load());
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Creation.
///
///  @param[in]  requested Requested.
///  @param[in]  stack Stack.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void creation(bool requested, uint32_t stack).
//--------------------------------------------------------------------------------------------------------------------
void creation(bool requested, uint32_t stack)
{
    if (requested) {
        increment(create_attempts);
    }
    stack_requested.store(requested ? stack : 0);
    life.store(uint32_t(requested ? Life::CREATE_ATTEMPT : Life::NOT_REQUESTED));
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Creation failed.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void creation_failed().
//--------------------------------------------------------------------------------------------------------------------
void creation_failed()
{
    increment(create_failures);
    life.store(uint32_t(Life::CREATE_FAILED));
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Worker enter.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void worker_enter().
//--------------------------------------------------------------------------------------------------------------------
void worker_enter()
{
    const Identity fn = identity.load();
    worker_id.store(fn == nullptr ? 0 : fn());
    life.store(uint32_t(Life::RUNNING));
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Worker start.
///
///  @param[in]  now Now.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void worker_start(uint32_t now).
//--------------------------------------------------------------------------------------------------------------------
void worker_start(uint32_t now)
{
    worker_begin.store(now);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Worker finish.
///
///  @param[in]  now Now.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void worker_finish(uint32_t now).
//--------------------------------------------------------------------------------------------------------------------
void worker_finish(uint32_t now)
{
    worker_done.store(now);
    increment(worker_loops);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Stack sample.
///
///  @param[in]  total Total.
///  @param[in]  free Free.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void stack_sample(uint32_t total, uint32_t free).
//--------------------------------------------------------------------------------------------------------------------
void stack_sample(uint32_t total, uint32_t free)
{
    stack_total.store(total);
    stack_free_min.store(MIN(stack_free_min.load(), free));
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Loop.
///
///  @param[in]  now Now.
///  @param[in]  nominal Nominal.
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void loop(uint32_t now, uint32_t nominal).
//--------------------------------------------------------------------------------------------------------------------
void loop(uint32_t now, uint32_t nominal)
{
    // Rozestup hlavních smyček v mikrosekundách.
    uint32_t dt;

    static uint32_t previous;
    static bool primed;
    if (primed) {
        dt = now - previous;
        metrics[uint8_t(Metric::LOOP_PERIOD)].record(dt);
        metrics[uint8_t(Metric::LOOP_JITTER)].record(dt > nominal ? dt - nominal : nominal - dt);
        if (dt > nominal) {
            increment(deadlines);
        }
    }
    previous = now;
    primed = true;
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Task overrun.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void task_overrun().
//--------------------------------------------------------------------------------------------------------------------
void task_overrun()
{
    increment(task_overruns);
}
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Mutex probe.
///
///  @param[in]  sem Sem.
///  @param[in]  state Sampled attitude, gyro and validity metadata.
///  @param[in]  line Line.
///  @return     None (constructor/destructor).
///
///  @details    Public class member. Declaration/definition: MutexProbe::MutexProbe(AP_HAL::Semaphore &sem, MutexState
///              &state, uint32_t line).
//--------------------------------------------------------------------------------------------------------------------
MutexProbe::MutexProbe(AP_HAL::Semaphore &sem, MutexState &state, uint32_t line) :
    _sem(sem), _state(state), _thread(current_thread())
{
    // Generace intervalu workeru před pokusem o získání mutexu.
    uint32_t generation;
    // Čas těsně před původním získáním mutexu v mikrosekundách.
    uint32_t t0;

    generation = _state.worker.generation();
    // Preserve WithSemaphore watchdog attribution as well as acquire policy.
    if (_thread == Thread::MAIN) {
        hal.util->persistent_data.semaphore_line = line;
    }
    t0 = AP_HAL::micros();
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
//--------------------------------------------------------------------------------------------------------------------
///  @brief      ~mutex probe.
///
///  @param[in]  None
///  @return     None (constructor/destructor).
///
///  @details    Public class member. Declaration/definition: MutexProbe::~MutexProbe().
//--------------------------------------------------------------------------------------------------------------------
MutexProbe::~MutexProbe()
{
    // Čas těsně před původním uvolněním mutexu v mikrosekundách.
    uint32_t t2;
    // Nastavená mez sledované acquire latence v mikrosekundách.
    uint32_t limit;

    if (_thread == Thread::UTB_WORKER) {
        // Conservative overlap endpoint is published before unlock. The final
        // hold timestamp remains immediately adjacent to the original give().
        _state.worker.end(AP_HAL::micros());
    }
    t2 = AP_HAL::micros();
    _sem.give();
    // Counter work is outside the measured mutex. It never calls logging.
    if (_thread == Thread::MAIN) {
        metrics[uint8_t(Metric::MAIN_ACQUIRE)].record(_acquire);
        limit = wait_limit.load();
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private function prototypes
