// GNU GPL v3 or later. Original controller algorithms retain their copyright.
/// @file AP_UTB_FlightBin.h
/// @brief Finite passive Flight BIN transport; one main producer and existing UTB worker.
/// @author Jiri Kovar
/// @version 2026-10-26
#pragma once
#include "AP_UTB_config.h"
#include <AP_Logger/AP_Logger_config.h>
#define AP_UTB_FLIGHT_BIN_AVAILABLE (AP_UTB_FLIGHT_SHADOW_AVAILABLE && AP_UTB_FLIGHT_BIN_ENABLED && AP_UTB_CHAIN_SHADOW_ENABLED && HAL_LOGGING_ENABLED)
#if AP_UTB_FLIGHT_BIN_AVAILABLE
#include <AP_Param/AP_Param.h>
#include <atomic>

class AP_Logger;
class ExpandingString;
class UTB_ChainShadowEngine;

class AP_UTB_FlightBin
{
public:
    enum class State : uint8_t { IDLE, RUNNING, STOPPED };
    enum class Reason : uint8_t { NONE, COUNT, DISARM, MODE, FAULT, UNSUPPORTED, CONFLICT, DISABLED, TIME, CONFIG, FAILSAFE, WORKER };
    struct Sample {
        uint64_t time_us, source_us;
        uint32_t seq, loop, source;
        uint16_t flags[3];
        uint8_t mode;
        float dt;
        float pid[3][10]; // Native P/I/D/FF/D_FF followed by independent UTB terms.
        float rate[3][7]; // Gyro, native/UTB target, control, and scaled FF outputs.
        float attitude[12]; // Source actual, native target and independent UTB target quaternions.
    };
    class Queue
    {
    public:
        static constexpr uint32_t CAPACITY = 4;
        bool push(const Sample &sample);
        uint32_t producer_depth() const
        {
            return _producer_depth;
        }
        bool pop(Sample &sample);
        uint32_t depth() const;
    private:
        friend class UTBFlightBinTestAccess;
        Sample _slots[CAPACITY] {};
        uint32_t _producer_depth = 0; // Producer-only conservative occupancy before publication.
        std::atomic<uint32_t> _head {0}, _tail {0};
    };
    struct Counts {
        uint32_t requested, queued, accepted, dropped, invalid, unsupported, rejected, hwm, depth;
    };
    AP_UTB_FlightBin();
    // Main thread only. Observe every ARM state, even when runtime disabled.
    bool arm_edge(bool armed, bool mode_supported, bool available, uint64_t now_us);
    Reason stop_condition(bool armed, uint8_t mode, bool supported, uint64_t now_us, bool failsafe = false) const;
    void stop(Reason reason);
    void sample(const UTB_ChainShadowEngine &engine, uint32_t loop, uint64_t now_us);
    void worker_ready(bool ready)
    {
        _worker_ready = ready;
    }
    void worker(AP_Logger &logger, bool available);
    Counts counts() const;
    void dump(ExpandingString &text) const;
    State state() const
    {
        return State(_state.load(std::memory_order_acquire));
    }
    uint32_t limit() const
    {
        return uint32_t(_duration) * 400U;
    }
    bool clock_needed() const
    {
        return _enable.get() != 0 || state() == State::RUNNING;
    }
    static bool expand_time(uint32_t low, uint64_t reference, uint64_t &result);
    static bool make_sample(const UTB_ChainShadowEngine &engine, uint32_t loop, uint32_t seq,
                            uint64_t reference, Sample &sample);
    static const AP_Param::GroupInfo var_info[];
private:
    friend class UTBFlightBinTestAccess;
    void enqueue(const Sample &value);
    AP_Int8 _enable, _rate;
    AP_Int16 _seconds;
    Queue _queue;
    std::atomic<uint32_t> _state {0}, _reason {0};
    std::atomic<uint32_t> _requested {0}, _queued {0}, _accepted {0}, _dropped {0};
    std::atomic<uint32_t> _invalid {0}, _unsupported {0}, _rejected {0}, _hwm {0};
    uint64_t _start_us = 0;
    uint32_t _last_status_ms = 0;
    uint16_t _duration = 60;
    uint8_t _hz = 10;
    bool _worker_ready = false; // Main-owned startup result; no retry or allocation on ARM.
    bool _last_armed = false, _attempted = false, _status_sent = false;
    bool _opted_in_disarmed = false; // Opt-in must be observed before the ARM edge, not in the same command batch.
    std::atomic<bool> _final_sent {false}; // Final admission attempt and counters published for DISARMED export.
};
static_assert(ATOMIC_INT_LOCK_FREE == 2, "Flight queue requires lock-free 32-bit publication");
#endif
