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
#include "Copter.h"
#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
#include <AP_UTB/AP_UTB_Reference.h>

void Copter::utb_shadow_update()
{
#if AP_UTB_BENCH_ENABLED
    UTBBench::Duration duration(UTBBench::Metric::PRODUCER);
    utb_bench_config();
#endif
    if (!utb.enabled()) {
        return;
    }
    AP_UTB::Inputs in;
    if (!utb.shadow_requested()) {
        in.mode = uint8_t(flightmode->mode_number());
        in.time_us = AP_HAL::micros64();
        utb.update(in);
        return;
    }
    in.started_us = AP_HAL::micros();
    in.time_us = AP_HAL::micros64();
    in.dt = scheduler.get_last_loop_time_s();
    in.nominal_dt = scheduler.get_loop_period_s();
    in.mode = uint8_t(flightmode->mode_number());
    in.armed = motors->armed();
    in.frame_class = g2.frame_class.get();
    in.frame_type = g.frame_type.get();
    in.fast_rate = using_rate_thread || get_fast_rate_type() != FastRateType::FAST_RATE_DISABLED;
    if (utb.time_budget_latched()) {
        in.logging_available = utb.logging_available();
        utb.update(in);
        return;
    }
    utb.sample_state(in.state);
    in.ap_target = utb.captured_target();
    in.ap_target.valid &= !in.fast_rate && in.ap_target.mode == in.mode;
    in.rc_input_ms = g2.rc_channels.last_input_ms();
    in.logging_available = utb.logging_available();
#if MODE_ACRO_ENABLED
    const RC_Channel *channels[] = {channel_roll, channel_pitch, channel_yaw};
    bool calibration_valid = true;
    for (const RC_Channel *channel : channels) {
        if (channel == nullptr) {
            calibration_valid = false;
            break;
        }
        calibration_valid &= channel->get_radio_min() < channel->get_radio_trim() - channel->get_dead_zone() &&
                             channel->get_radio_trim() + channel->get_dead_zone() < channel->get_radio_max();
    }
    calibration_valid &= channel_throttle != nullptr;
    if (channel_throttle != nullptr) {
        calibration_valid &= channel_throttle->get_type() == RC_Channel::ControlType::RANGE &&
                             channel_throttle->get_radio_min() + channel_throttle->get_dead_zone() <
                             channel_throttle->get_radio_max();
    }
    // get_control_mid() divides by max-min-deadzone; validate before calling the AP helper.
    const int16_t mid = calibration_valid ? get_throttle_mid() : 0;
    const int16_t control = calibration_valid ? channel_throttle->get_control_in() : 0;
    if (calibration_valid && mid > 0 && mid < 1000 && control >= 0 && control <= 1000 &&
        g2.rc_channels.has_valid_input() && !failsafe.radio) {
        const Vector3f sticks(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz());
        in.reference_valid = AP_UTB_Reference::rates(sticks, g2.command_model_acro_rp.get_rate(),
                             g2.command_model_acro_y.get_rate(), g2.command_model_acro_rp.get_expo(),
                             g2.command_model_acro_y.get_expo(), in.reference);
        in.thrust = mode_acro.get_pilot_desired_throttle();
    }
#else
    in.acro_available = false;
#endif
    utb.update(in);
}
#endif // AP_UTB_ENABLED

#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
#include "utb_log.h"
#include "utb_bench_log.h"
#if AP_UTB_BENCH_ENABLED
void utb_bench_stack();
#endif

void Copter::utb_log_init()
{
#if HAL_LOGGING_ENABLED
#if AP_UTB_BENCH_ENABLED
    UTBBench::creation(utb.enabled(), 3072);
#endif
    if (utb.enabled()) {
        // Allocation is confined to vehicle startup, never the producer.
        bool created;
#if AP_UTB_BENCH_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (utb.bench_fail_create()) {
            created = false;
        } else
#endif
        {
            created = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Copter::utb_log_thread, void),
                                                   "utb_log", 3072, AP_HAL::Scheduler::PRIORITY_IO, 0);
        }
        if (!created) {
#if AP_UTB_BENCH_ENABLED
            UTBBench::creation_failed();
#endif
            utb.set_logging_available(false);
        }
    }
#endif
}

void Copter::utb_log_thread()
{
#if HAL_LOGGING_ENABLED
#if AP_UTB_BENCH_ENABLED
    UTBBench::worker_enter();
    uint32_t last_stack_ms = 0;
#endif
    while (true) {
#if AP_UTB_BENCH_ENABLED
        const uint32_t pass_start = AP_HAL::micros();
        UTBBench::worker_start(pass_start);
        if (AP_HAL::millis() - last_stack_ms >= 1000) {
            utb_bench_stack();
            last_stack_ms = AP_HAL::millis();
        }
#endif
        const bool available = logger.should_log(UINT32_MAX) && logger.logging_present() && logger.logging_started();
        utb.set_logging_available(available);
        AP_UTB::Snapshot status;
        if (utb.read_log_status(status)) {
            utb_status_update(status);
        }
        // Bounded drain, with a sleep even if the producer outpaces the backend.
        for (uint8_t i = 0; i < AP_UTB::Queue::CAPACITY && available; i++) {
            if (!utb_log_update()) {
                break;
            }
        }
#if AP_UTB_BENCH_ENABLED
        const uint32_t pass_done = AP_HAL::micros();
        UTBBench::metrics[uint8_t(UTBBench::Metric::WORKER_PASS)].record(pass_done - pass_start);
        UTBBench::worker_finish(pass_done);
#endif
        hal.scheduler->delay(1);
    }
#endif
}

bool Copter::utb_log_update()
{
#if AP_UTB_BENCH_ENABLED
    UTBBench::Duration duration(UTBBench::Metric::CONSUMER);
#endif
#if HAL_LOGGING_ENABLED
    const uint32_t log_start_us = AP_HAL::micros();
    static uint32_t max_log_us;
    // Worker only: backend semaphore waits cannot hold the producer queue lock.
    AP_UTB::Snapshot s;
    const bool popped = utb.pop_log(s);
    if (popped && utb.logging_requested() && s.epoch == utb.epoch()) {
        bool complete = true;
        const auto &in = s.input;
        for (uint8_t axis = 0; axis < 3; axis++) {
            const auto &a = s.control.axis[axis];
            const float residual = a.raw - s.mixer.achieved[axis];
            const uint8_t sat = (residual > 1.0e-5f ? 1 : 0) | (residual < -1.0e-5f ? 2 : 0) |
                                (a.integration_blocked ? 4 : 0);
            const log_UTBR rate = {
                LOG_PACKET_HEADER_INIT(LOG_UTBR_MSG),
                in.time_us, s.seq, in.mode, axis, in.reference[axis], in.state.rates_rads[axis],
                a.error, a.p, a.i, a.d, a.raw, a.output, in.ap_target.target[axis],
                uint8_t(s.control.valid), sat
            };
            complete &= logger.WriteBlock_first_succeed(&rate, sizeof(rate));
        }
        const auto &m = s.mixer;
        const log_UTBM motors_log = {
            LOG_PACKET_HEADER_INIT(LOG_UTBM_MSG),
            in.time_us, s.seq, in.mode, s.control.command.x, s.control.command.y, s.control.command.z,
            in.thrust, m.thrust[0], m.thrust[1], m.thrust[2], m.thrust[3], m.scale, m.collective_shift,
            uint8_t(m.valid), m.lower_mask, m.upper_mask
        };
        complete &= logger.WriteBlock_first_succeed(&motors_log, sizeof(motors_log));
        const log_UTBA achieved = {
            LOG_PACKET_HEADER_INIT(LOG_UTBA_MSG),
            in.time_us, s.seq, in.mode, m.achieved.x, m.achieved.y, m.achieved.z, m.achieved_thrust,
            m.positive_limits, m.negative_limits, uint8_t(m.moments_scaled), uint8_t(m.collective_shifted)
        };
        complete &= logger.WriteBlock_first_succeed(&achieved, sizeof(achieved));
        const log_UTBT timing = {
            LOG_PACKET_HEADER_INIT(LOG_UTBT_MSG),
            in.time_us, s.seq, in.mode, in.ap_target.time_us, in.ap_target.seq, in.ap_target.mode,
            uint8_t(in.ap_target.valid && in.ap_target.seq == s.seq), in.rc_input_ms, in.dt,
            s.execution_us, s.max_execution_us, s.requested_log_rate, s.effective_log_rate, s.drops, s.missing
        };
        complete &= logger.WriteBlock_first_succeed(&timing, sizeof(timing));
#if AP_UTB_BENCH_ENABLED
        const log_UBEP epoch = { LOG_PACKET_HEADER_INIT(LOG_UBEP_MSG), in.time_us, s.seq, s.epoch };
        complete &= logger.WriteBlock_first_succeed(&epoch, sizeof(epoch));
#endif
        utb.log_result(complete);
    } else if (popped) {
        utb.log_discarded();
    }
    AP_UTB::Snapshot current;
    if (popped && s.epoch == utb.epoch() && utb.logging_requested()) {
        current = s;
    } else if (!utb.read_log_status(current)) {
        return popped;
    }
    static uint64_t last_status_us;
    static uint16_t last_flags;
    static uint8_t last_calculation;
    static uint8_t last_observation;
    static uint8_t last_policy;
    const uint64_t now = AP_HAL::micros64();
    const uint16_t flags = utb.logging_requested() ? current.flags : AP_UTB::ENABLED | AP_UTB::LOGGING_AVAILABLE;
    const uint8_t calculation = uint8_t(current.calculation);
    const uint8_t observation = uint8_t(current.observation);
    const uint8_t policy = uint8_t(current.policy);
    max_log_us = MAX(max_log_us, AP_HAL::micros() - log_start_us);
    if (last_flags == flags && last_calculation == calculation && last_observation == observation &&
        last_policy == policy && now - last_status_us < 1000000U) {
        return popped;
    }
    const log_UTBS status = {
        LOG_PACKET_HEADER_INIT(LOG_UTBS_MSG),
        current.input.time_us == 0 ? now : current.input.time_us, current.seq,
        current.input.mode,
        flags, calculation, observation, policy,
        current.input.dt, current.execution_us, current.max_execution_us, current.requested_log_rate,
        current.effective_log_rate, current.drops, current.missing, current.overruns, max_log_us
    };
    if (logger.WriteBlock_first_succeed(&status, sizeof(status))) {
        last_status_us = now;
        last_flags = flags;
        last_calculation = calculation;
        last_observation = observation;
        last_policy = policy;
    }
    const auto counters = utb.pipeline();
    static uint64_t last_pipeline_us;
    if (now - last_pipeline_us >= 1000000U) {
        const log_UTBQ pipeline = {
            LOG_PACKET_HEADER_INIT(LOG_UTBQ_MSG), now, current.seq, counters.requested, counters.enqueued,
            counters.written, counters.dropped, counters.missing, counters.high_water, counters.depth,
            utb.epoch(), current.input.state.primary_gyro, counters.discarded
        };
        logger.WriteBlock_first_succeed(&pipeline, sizeof(pipeline));
        last_pipeline_us = now;
    }
    max_log_us = MAX(max_log_us, AP_HAL::micros() - log_start_us);
    return popped;
#else
    return false;
#endif // HAL_LOGGING_ENABLED
}
#endif // AP_UTB_ENABLED

#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
// Slow diagnostic status also exposes start policy when no logger is available.
void Copter::utb_status_update(const AP_UTB::Snapshot &s)
{
    if (!utb.enabled()) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    const uint16_t flags = utb.logging_requested() ? s.flags : uint16_t(AP_UTB::ENABLED);
    const uint32_t key = uint32_t(flags) | (uint32_t(s.calculation) << 10) |
                         (uint32_t(s.observation) << 16) | (uint32_t(s.policy) << 20);
    static uint32_t last_key = UINT32_MAX;
    static uint32_t last_ms;
    if (now - last_ms >= 1000 && (key != last_key || now - last_ms >= 5000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UTB flags=%u calc=%u obs=%u policy=%u",
                      unsigned(flags), unsigned(s.calculation), unsigned(s.observation), unsigned(s.policy));
        last_key = key;
        last_ms = now;
    }
}
#endif // AP_UTB_ENABLED
