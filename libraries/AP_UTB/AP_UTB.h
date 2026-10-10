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
#include "AP_UTB_config.h"
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_ENABLED
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_UTB_MotorMixer.h"

class AP_UTB
{
public:
    AP_UTB();
    CLASS_NO_COPY(AP_UTB);
    enum class CalculationReason : uint8_t {
        NONE,
        PRIMING,
        STATE_INVALID,
        DT_INVALID,
        REFERENCE_INVALID,
        CONFIG_INVALID,
        FRAME_MISMATCH,
        FAST_RATE_UNSUPPORTED,
        CONTROLLER_INVALID,
        MIXER_INVALID,
        ACRO_UNAVAILABLE,
        PRIMARY_GYRO_CHANGED,
    };
    enum class ObservationReason : uint8_t {
        NONE,
        LOGGING_UNAVAILABLE,
        QUEUE_FULL,
        WRITE_FAILED,
    };
    enum class PolicyReason : uint8_t {
        NONE,
        EXPERIMENT_SUPPRESSED,
        TIME_BUDGET,
    };
    enum HealthFlag : uint16_t {
        ENABLED = 1U << 0,
        SHADOW_REQUESTED = 1U << 1,
        STATE_VALID = 1U << 2,
        CONTROLLER_EVALUATED = 1U << 3,
        CONTROLLER_VALID = 1U << 4,
        MIXER_EVALUATED = 1U << 5,
        MIXER_VALID = 1U << 6,
        LOGGING_AVAILABLE = 1U << 7,
        SHADOW_HEALTHY = 1U << 8,
        SHADOW_OBSERVABLE = 1U << 9,
    };
    struct TargetCapture {
        Vector3f target;
        uint64_t time_us = 0;
        uint32_t seq = 0;
        uint8_t mode = 0;
        bool valid = false;
    };
    struct Inputs {
        AP_UTB_State state;
        Vector3f reference;
        TargetCapture ap_target;
        uint64_t time_us = 0;
        uint32_t started_us = 0;
        uint32_t rc_input_ms = 0;
        float dt = 0;
        float nominal_dt = 0;
        float thrust = 0;
        uint8_t mode = 0;
        uint8_t frame_class = 0;
        uint8_t frame_type = 0;
        bool armed = false;
        bool fast_rate = false;
        bool reference_valid = false;
        bool acro_available = true;
        bool logging_available = false;
    };
    struct Snapshot {
        Inputs input;
        AP_UTB_RateController::Result control;
        AP_UTB_MotorMixer::Result mixer;
        uint32_t seq = 0;
        uint32_t epoch = 0;
        uint16_t flags = 0;
        CalculationReason calculation = CalculationReason::NONE;
        ObservationReason observation = ObservationReason::NONE;
        PolicyReason policy = PolicyReason::NONE;
        uint32_t execution_us = 0;
        uint32_t max_execution_us = 0;
        uint32_t drops = 0;
        uint32_t missing = 0;
        uint32_t overruns = 0;
        float effective_log_rate = 0;
        uint16_t requested_log_rate = 0;
    };
    // Fixed AP SPSC data transport; only the independent status mailbox uses try-lock.
    class Queue
    {
    public:
        static constexpr uint8_t CAPACITY = 4;
        explicit Queue(AP_HAL::Semaphore *semaphore = nullptr) :
            _buffer(_storage, sizeof(_storage)), _objects(&_buffer),
            _sem(semaphore == nullptr ? &_default_sem : semaphore) {}
        bool publish(const Snapshot &value, bool enqueue);
        bool push(const Snapshot &value)
        {
            return publish(value, true);
        }
        bool pop(Snapshot &value);
        bool read_status(Snapshot &value);
        uint8_t size() const
        {
            return _objects.available();
        }
        uint8_t high_water() const
        {
            return _high_water.load();
        }
        uint32_t enqueued() const
        {
            return _enqueued.load();
        }
    private:
        uint8_t _storage[(CAPACITY + 1) * sizeof(Snapshot)] {};
        ByteBuffer _buffer;
        ObjectBuffer<Snapshot> _objects;
        HAL_Semaphore _default_sem;
        AP_HAL::Semaphore *_sem;
        Snapshot _status;
        std::atomic<uint32_t> _enqueued{0};
        std::atomic<uint8_t> _high_water{0};

    };
    struct Pipeline {
        uint32_t requested;
        uint32_t enqueued;
        uint32_t written;
        uint32_t dropped;
        uint32_t missing;
        uint32_t discarded;
        uint8_t high_water;
        uint8_t depth;
    };
    Pipeline pipeline() const
    {
        Pipeline p;
        // Read downstream first: monotonic counters retain written <= enqueued <= requested.
        p.written = _written_sets.load();
        p.enqueued = _queue.enqueued();
        p.requested = _requested_sets.load();
        p.dropped = _drops.load();
        p.missing = _missing.load();
        p.discarded = _discarded.load();
        p.high_water = _queue.high_water();
        p.depth = _queue.size();
        return p;
    }
    void set_logging_available(bool available)
    {
        _logging_available.store(available);
    }
    bool logging_available() const
    {
        return _logging_available.load();
    }
    bool logging_requested() const
    {
        return _logging_requested.load();
    }
    uint32_t epoch() const
    {
        return _epoch.load();
    }
    bool read_log_status(Snapshot &value)
    {
        return _queue.read_status(value);
    }
    void init(const AP_AHRS_View &view);
    bool enabled() const
    {
        return _enabled_at_boot;
    }
    bool shadow_requested() const
    {
        return enabled() && _shadow.get() == 1;
    }
    void update_state();
    void sample_state(AP_UTB_State &state) const
    {
        _provider.sample(state);
    }
    const AP_UTB_State &state() const
    {
        return _state;
    }
    bool healthy() const
    {
        return (_snapshot.flags & SHADOW_HEALTHY) != 0;
    }
    void capture_target(const Vector3f &target, uint64_t time_us, uint8_t mode);
    const TargetCapture &captured_target() const
    {
        return _capture;
    }
    void update(const Inputs &input);
    // Pure diagnostic engine is also exercised with synthetic inputs in unit tests.
    void evaluate(const Inputs &input, const AP_UTB_RateController::Gains (&gains)[3], Snapshot &snapshot);
    void reset_shadow();
    bool time_budget_latched() const
    {
        return _budget_latched;
    }
    void record_execution(uint32_t elapsed_us, float nominal_dt);
    const Snapshot &snapshot() const
    {
        return _snapshot;
    }
    bool pop_log(Snapshot &snapshot)
    {
        return _queue.pop(snapshot);
    }
    void log_result(bool success);
    void log_discarded()
    {
        ++_discarded;
    }
    bool publish_diagnostics(Snapshot &snapshot, bool selected);
#if AP_UTB_BENCH_ENABLED
    uint32_t bench_wait_us() const
    {
        return MAX(0, _bench_wait.get());
    }
    uint16_t bench_log_rate() const
    {
        return MAX(0, _log_rate.get());
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    bool bench_fail_create() const
    {
        return _bench_fail.get() != 0;
    }
#endif
#endif
    static const AP_Param::GroupInfo var_info[];
private:
#if AP_UTB_BENCH_ENABLED
    AP_Int32 _bench_wait;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_Int8 _bench_fail;
#endif
#endif
    AP_Int8 _enable;
    AP_Int8 _shadow;
    AP_Int16 _log_rate;
    AP_Float _p[3];
    AP_Float _i[3];
    AP_Float _d[3];
    AP_Float _imax[3];
    AP_Float _d_hz[3];
    bool _enabled_at_boot = false;
    bool _was_requested = false;
    bool _experiment_started = false;
    bool _context_valid = false;
    bool _budget_latched = false;
    uint8_t _last_mode = 0;
    uint8_t _last_class = 0;
    uint8_t _last_type = 0;
    bool _last_armed = false;
    uint32_t _last_imu_us = 0;
    uint8_t _last_gyro = 255;
    uint32_t _seq = 0;
    uint32_t _last_observed_missing = 0;
    std::atomic<uint32_t> _drops{0};
    std::atomic<uint32_t> _missing{0};
    std::atomic<uint32_t> _discarded{0};
    std::atomic<uint32_t> _requested_sets{0};
    std::atomic<uint32_t> _epoch{0};
    std::atomic<bool> _logging_available{false};
    std::atomic<bool> _logging_requested{false};
    std::atomic<bool> _backend_failed{false};
    uint32_t _overruns = 0;
    uint8_t _consecutive_overruns = 0;
    uint32_t _max_execution_us = 0;
    uint64_t _last_log_us = 0;
    uint64_t _rate_start_us = 0;
    uint32_t _rate_start_written = 0;
    int16_t _last_log_rate = 0;
    std::atomic<uint32_t> _written_sets{0};
    TargetCapture _capture;
    AP_UTB_APStateProvider _provider;
    AP_UTB_State _state;
    AP_UTB_RateController _controller;
    AP_UTB_MotorMixer _mixer;
    Snapshot _snapshot;
    Queue _queue;
};
#endif // AP_UTB_ENABLED
