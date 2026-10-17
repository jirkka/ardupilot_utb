//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB.h
///  @brief      Manager SHADOW, parametry, reset, oddělená matematická/logging policy, bounded queue a epochy.
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
#pragma once
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "AP_UTB_config.h"
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_ENABLED
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_UTB_MotorMixer.h"
#if AP_UTB_BENCH_ENABLED
#include "AP_UTB_SyntheticReference.h"
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
/// @brief UTB component; Manager SHADOW, parametry, reset, oddělená matematická/logging policy, bounded queue
///        a epochy.
class AP_UTB
{
public:
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Initialize AP_Param defaults; control and logging remain disabled by default.
    ///
    ///  @param[in]  None
    ///  @return     None (constructor/destructor).
    ///
    ///  @details    Public class member. Declaration/definition: AP_UTB().
    //--------------------------------------------------------------------------------------------------------------------
    AP_UTB();
    CLASS_NO_COPY(AP_UTB);
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Public functions
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
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
        uint32_t validation_time_us = 0; // Monotonic time captured after sampling; time_us remains the log timestamp.
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
#if AP_UTB_BENCH_ENABLED
        uint32_t synthetic_cycle = 0;
        uint8_t reference_source = 0; // 0: RC, 1: internal reference requested (possibly invalid).
        uint8_t synthetic_profile = 0;
        uint8_t synthetic_phase = 0;
        uint8_t synthetic_status = 0;
        bool watchdog = false;
        bool bench_safe = false;
#endif
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
        uint8_t log_format = 0;
    };
    // Fixed AP SPSC data transport; only the independent status mailbox uses try-lock.
    class Queue
    {
    public:
        static constexpr uint8_t CAPACITY = 4;
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // End Data type, constant, macro definition
        //====================================================================================================================
        // Public functions
        //--------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Queue.
        ///
        ///  @param[in]  semaphore Semaphore.
        ///  @return     None (constructor/destructor).
        ///
        ///  @details    Public class member. Declaration/definition: explicit Queue(AP_HAL::Semaphore *semaphore = nullptr).
        //--------------------------------------------------------------------------------------------------------------------
        explicit Queue(AP_HAL::Semaphore *semaphore = nullptr) :
            _buffer(_storage, sizeof(_storage)), _objects(&_buffer),
            _sem(semaphore == nullptr ? _default_sem : *semaphore) {}
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Publish.
        ///
        ///  @param[in]  value Value supplied to this operation.
        ///  @param[in]  enqueue Enqueue.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool publish(const Snapshot &value, bool enqueue).
        //--------------------------------------------------------------------------------------------------------------------
        bool publish(const Snapshot &value, bool enqueue);
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Push.
        ///
        ///  @param[in]  value Value supplied to this operation.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool push(const Snapshot &value).
        //--------------------------------------------------------------------------------------------------------------------
        bool push(const Snapshot &value)
        {
            return publish(value, true);
        }
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Pop.
        ///
        ///  @param[in]  value Value supplied to this operation.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool pop(Snapshot &value).
        //--------------------------------------------------------------------------------------------------------------------
        bool pop(Snapshot &value);
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Try to copy the latest status mailbox without blocking.
        ///
        ///  @param[in]  value Value supplied to this operation.
        ///  @return     bool.
        ///
        ///  @details    Public class member. Declaration/definition: bool read_status(Snapshot &value).
        //--------------------------------------------------------------------------------------------------------------------
        bool read_status(Snapshot &value);
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Size.
        ///
        ///  @param[in]  None
        ///  @return     uint8_t.
        ///
        ///  @details    Public class member. Declaration/definition: uint8_t size().
        //--------------------------------------------------------------------------------------------------------------------
        uint8_t size() const
        {
            return _objects.available();
        }
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      High water.
        ///
        ///  @param[in]  None
        ///  @return     uint8_t.
        ///
        ///  @details    Public class member. Declaration/definition: uint8_t high_water().
        //--------------------------------------------------------------------------------------------------------------------
        uint8_t high_water() const
        {
            return _high_water.load();
        }
        //--------------------------------------------------------------------------------------------------------------------
        ///  @brief      Enqueued.
        ///
        ///  @param[in]  None
        ///  @return     uint32_t.
        ///
        ///  @details    Public class member. Declaration/definition: uint32_t enqueued().
        //--------------------------------------------------------------------------------------------------------------------
        uint32_t enqueued() const
        {
            return _enqueued.load();
        }
    private:
        uint8_t _storage[(CAPACITY + 1) * sizeof(Snapshot)] {};
        ByteBuffer _buffer;
        ObjectBuffer<Snapshot> _objects;
        HAL_Semaphore _default_sem;
        // Platný zámek status mailboxu; vlastní výchozí nebo dodaný testem.
        AP_HAL::Semaphore &_sem;
        Snapshot _status;
        std::atomic<uint32_t> _enqueued{0};
        std::atomic<uint8_t> _high_water{0};

    };
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Public functions
    //====================================================================================================================
    // Data type, constant, macro definition
    //--------------------------------------------------------------------------------------------------------------------
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
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // End Data type, constant, macro definition
    //====================================================================================================================
    // Public functions
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Pipeline.
    ///
    ///  @param[in]  None
    ///  @return     Pipeline.
    ///
    ///  @details    Public class member. Declaration/definition: Pipeline pipeline().
    //--------------------------------------------------------------------------------------------------------------------
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
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Set logging available.
    ///
    ///  @param[in]  available Available.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void set_logging_available(bool available).
    //--------------------------------------------------------------------------------------------------------------------
    void set_logging_available(bool available)
    {
        _logging_available.store(available);
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Logging available.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool logging_available().
    //--------------------------------------------------------------------------------------------------------------------
    bool logging_available() const
    {
        return _logging_available.load();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Logging requested.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool logging_requested().
    //--------------------------------------------------------------------------------------------------------------------
    bool logging_requested() const
    {
        return _logging_requested.load();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Epoch.
    ///
    ///  @param[in]  None
    ///  @return     uint32_t.
    ///
    ///  @details    Public class member. Declaration/definition: uint32_t epoch().
    //--------------------------------------------------------------------------------------------------------------------
    uint32_t epoch() const
    {
        return _epoch.load();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Read log status.
    ///
    ///  @param[in]  value Value supplied to this operation.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool read_log_status(Snapshot &value).
    //--------------------------------------------------------------------------------------------------------------------
    bool read_log_status(Snapshot &value)
    {
        return _queue.read_status(value);
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Init.
    ///
    ///  @param[in]  view View.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void init(const AP_AHRS_View &view).
    //--------------------------------------------------------------------------------------------------------------------
    void init(const AP_AHRS_View &view);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Report whether this component is enabled in the current configuration.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool enabled().
    //--------------------------------------------------------------------------------------------------------------------
    bool enabled() const
    {
        return _enabled_at_boot;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Shadow requested.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool shadow_requested().
    //--------------------------------------------------------------------------------------------------------------------
    bool shadow_requested() const
    {
        return enabled() && _shadow.get() == 1;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Update state.
    ///
    ///  @param[in]  None
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void update_state().
    //--------------------------------------------------------------------------------------------------------------------
    void update_state();
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sample state.
    ///
    ///  @param[in]  state Sampled attitude, gyro and validity metadata.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void sample_state(AP_UTB_State &state).
    //--------------------------------------------------------------------------------------------------------------------
    void sample_state(AP_UTB_State &state) const
    {
        _provider.sample(state);
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      State.
    ///
    ///  @param[in]  None
    ///  @return     const AP_UTB_State &.
    ///
    ///  @details    Public class member. Declaration/definition: const AP_UTB_State &state().
    //--------------------------------------------------------------------------------------------------------------------
    const AP_UTB_State &state() const
    {
        return _state;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Healthy.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool healthy().
    //--------------------------------------------------------------------------------------------------------------------
    bool healthy() const
    {
        return (_snapshot.flags & SHADOW_HEALTHY) != 0;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Capture the AP rate target together with its time, mode and sequence identity.
    ///
    ///  @param[in]  target Target.
    ///  @param[in]  time_us Sample/log timestamp in microseconds.
    ///  @param[in]  mode Mode.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void capture_target(const Vector3f &target, uint64_t
    ///              time_us, uint8_t mode).
    //--------------------------------------------------------------------------------------------------------------------
    void capture_target(const Vector3f &target, uint64_t time_us, uint8_t mode);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Captured target.
    ///
    ///  @param[in]  None
    ///  @return     const TargetCapture &.
    ///
    ///  @details    Public class member. Declaration/definition: const TargetCapture &captured_target().
    //--------------------------------------------------------------------------------------------------------------------
    const TargetCapture &captured_target() const
    {
        return _capture;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Update.
    ///
    ///  @param[in]  input Explicit input snapshot.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void update(const Inputs &input).
    //--------------------------------------------------------------------------------------------------------------------
    void update(const Inputs &input);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Select a real controller sample at the requested logging cadence without backfilling.
    ///
    ///  @param[in]  time_us Sample/log timestamp in microseconds.
    ///  @param[in]  nominal_dt Nominal controller interval in seconds.
    ///  @param[in]  rate Rate.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool select_log_sample(uint64_t time_us, float nominal_dt,
    ///              int16_t rate).
    //--------------------------------------------------------------------------------------------------------------------
    bool select_log_sample(uint64_t time_us, float nominal_dt, int16_t rate);
    // Pure diagnostic engine is also exercised with synthetic inputs in unit tests.
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Evaluate SHADOW validity and controller/mixer mathematics from explicit sampled inputs.
    ///
    ///  @param[in]  input Explicit input snapshot.
    ///  @param[in]  gains Per-axis controller gains.
    ///  @param[in]  snapshot Snapshot.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void evaluate(const Inputs &input, const
    ///              AP_UTB_RateController::Gains (&gains)[3], Snapshot &snapshot).
    //--------------------------------------------------------------------------------------------------------------------
    void evaluate(const Inputs &input, const AP_UTB_RateController::Gains (&gains)[3], Snapshot &snapshot);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Reset shadow.
    ///
    ///  @param[in]  None
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void reset_shadow().
    //--------------------------------------------------------------------------------------------------------------------
    void reset_shadow();
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Time budget latched.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool time_budget_latched().
    //--------------------------------------------------------------------------------------------------------------------
    bool time_budget_latched() const
    {
        return _budget_latched;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Record producer wall time and apply the existing time-budget policy.
    ///
    ///  @param[in]  elapsed_us Elapsed us.
    ///  @param[in]  nominal_dt Nominal controller interval in seconds.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void record_execution(uint32_t elapsed_us, float
    ///              nominal_dt).
    //--------------------------------------------------------------------------------------------------------------------
    void record_execution(uint32_t elapsed_us, float nominal_dt);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Snapshot.
    ///
    ///  @param[in]  None
    ///  @return     const Snapshot &.
    ///
    ///  @details    Public class member. Declaration/definition: const Snapshot &snapshot().
    //--------------------------------------------------------------------------------------------------------------------
    const Snapshot &snapshot() const
    {
        return _snapshot;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Pop log.
    ///
    ///  @param[in]  snapshot Snapshot.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool pop_log(Snapshot &snapshot).
    //--------------------------------------------------------------------------------------------------------------------
    bool pop_log(Snapshot &snapshot)
    {
        return _queue.pop(snapshot);
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Record successful complete-set admission or backend failure.
    ///
    ///  @param[in]  success Success.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void log_result(bool success).
    //--------------------------------------------------------------------------------------------------------------------
    void log_result(bool success);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Log discarded.
    ///
    ///  @param[in]  None
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void log_discarded().
    //--------------------------------------------------------------------------------------------------------------------
    void log_discarded()
    {
        ++_discarded;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Publish fixed diagnostic data and update queue/drop accounting.
    ///
    ///  @param[in]  snapshot Snapshot.
    ///  @param[in]  selected Selected.
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool publish_diagnostics(Snapshot &snapshot, bool
    ///              selected).
    //--------------------------------------------------------------------------------------------------------------------
    bool publish_diagnostics(Snapshot &snapshot, bool selected);
#if AP_UTB_BENCH_ENABLED
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Synthetic requested.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool synthetic_requested().
    //--------------------------------------------------------------------------------------------------------------------
    bool synthetic_requested() const
    {
        return _synthetic_request.get() != 0;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Gate and advance the DISARMED internal diagnostic reference.
    ///
    ///  @param[in]  input Explicit input snapshot.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void prepare_synthetic(Inputs &input).
    //--------------------------------------------------------------------------------------------------------------------
    void prepare_synthetic(Inputs &input);
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Bench wait us.
    ///
    ///  @param[in]  None
    ///  @return     uint32_t.
    ///
    ///  @details    Public class member. Declaration/definition: uint32_t bench_wait_us().
    //--------------------------------------------------------------------------------------------------------------------
    uint32_t bench_wait_us() const
    {
        return MAX(0, _bench_wait.get());
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Bench log rate.
    ///
    ///  @param[in]  None
    ///  @return     uint16_t.
    ///
    ///  @details    Public class member. Declaration/definition: uint16_t bench_log_rate().
    //--------------------------------------------------------------------------------------------------------------------
    uint16_t bench_log_rate() const
    {
        return MAX(0, _log_rate.get());
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Bench fail create.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool bench_fail_create().
    //--------------------------------------------------------------------------------------------------------------------
    bool bench_fail_create() const
    {
        return _bench_fail.get() != 0;
    }
#endif
#endif
    static const AP_Param::GroupInfo var_info[];
#if AP_UTB_RATE_SITL_AVAILABLE
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl attitude.
    ///
    ///  @param[in]  None
    ///  @return     int8_t.
    ///
    ///  @details    Public class member. Declaration/definition: int8_t sitl_attitude().
    //--------------------------------------------------------------------------------------------------------------------
    int8_t sitl_attitude() const
    {
        return _sitl_attitude.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl angle.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_angle().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_angle() const
    {
        return _sitl_angle.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl angle roll p.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_angle_roll_p().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_angle_roll_p() const
    {
        return _sitl_angle_roll_p.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl angle pitch p.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_angle_pitch_p().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_angle_pitch_p() const
    {
        return _sitl_angle_pitch_p.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl angle rate max.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_angle_rate_max().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_angle_rate_max() const
    {
        return _sitl_angle_rate_max.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl yaw rate.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_yaw_rate().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_yaw_rate() const
    {
        return _sitl_yaw_rate.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl throttle offset.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_throttle_offset().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_throttle_offset() const
    {
        return _sitl_throttle_offset.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl antiwindup.
    ///
    ///  @param[in]  None
    ///  @return     int8_t.
    ///
    ///  @details    Public class member. Declaration/definition: int8_t sitl_antiwindup().
    //--------------------------------------------------------------------------------------------------------------------
    int8_t sitl_antiwindup() const { return _sitl_antiwindup.get(); }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl owner.
    ///
    ///  @param[in]  None
    ///  @return     int8_t.
    ///
    ///  @details    Public class member. Declaration/definition: int8_t sitl_owner().
    //--------------------------------------------------------------------------------------------------------------------
    int8_t sitl_owner() const
    {
        return _sitl_owner.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl profile.
    ///
    ///  @param[in]  None
    ///  @return     int8_t.
    ///
    ///  @details    Public class member. Declaration/definition: int8_t sitl_profile().
    //--------------------------------------------------------------------------------------------------------------------
    int8_t sitl_profile() const
    {
        return _sitl_profile.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl run.
    ///
    ///  @param[in]  None
    ///  @return     bool.
    ///
    ///  @details    Public class member. Declaration/definition: bool sitl_run().
    //--------------------------------------------------------------------------------------------------------------------
    bool sitl_run() const
    {
        return _sitl_run.get() == 1;
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl delay.
    ///
    ///  @param[in]  None
    ///  @return     int8_t.
    ///
    ///  @details    Public class member. Declaration/definition: int8_t sitl_delay().
    //--------------------------------------------------------------------------------------------------------------------
    int8_t sitl_delay() const
    {
        return _sitl_delay.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl amplitude.
    ///
    ///  @param[in]  None
    ///  @return     float.
    ///
    ///  @details    Public class member. Declaration/definition: float sitl_amplitude().
    //--------------------------------------------------------------------------------------------------------------------
    float sitl_amplitude() const
    {
        return _sitl_amplitude.get();
    }
    //--------------------------------------------------------------------------------------------------------------------
    ///  @brief      Sitl gains.
    ///
    ///  @param[in]  gains Per-axis controller gains.
    ///  @return     None.
    ///
    ///  @details    Public class member. Declaration/definition: void sitl_gains(AP_UTB_RateController::Gains (&gains)[3]).
    //--------------------------------------------------------------------------------------------------------------------
    void sitl_gains(AP_UTB_RateController::Gains (&gains)[3]) const
    {
        for (uint8_t i = 0; i < 3; i++) {
            gains[i] = {_p[i].get(), _i[i].get(), _d[i].get(), _imax[i].get(), _d_hz[i].get()};
        }
    }
#endif
private:
#if AP_UTB_RATE_SITL_AVAILABLE
    AP_Int8 _sitl_attitude;
    AP_Float _sitl_angle, _sitl_angle_roll_p, _sitl_angle_pitch_p, _sitl_angle_rate_max;
    AP_Float _sitl_yaw_rate, _sitl_throttle_offset;
    AP_Int8 _sitl_antiwindup;
    AP_Int8 _sitl_owner;
    AP_Int8 _sitl_profile;
    AP_Int8 _sitl_run;
    AP_Float _sitl_amplitude;
    AP_Int8 _sitl_delay;
#endif
#if AP_UTB_BENCH_ENABLED
    AP_Int8 _synthetic_request;
    AP_UTB_SyntheticReference _synthetic;
    int8_t _last_synthetic_request = 0;
    AP_Int32 _bench_wait;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_Int8 _bench_fail;
#endif
#endif
    AP_Int8 _enable;
    AP_Int8 _shadow;
    AP_Int16 _log_rate;
    AP_Int8 _log_format;
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
