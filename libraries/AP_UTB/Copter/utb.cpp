//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       utb.cpp
///  @brief      Main SHADOW producer, worker lifecycle a bounded logger drain, pomalý status.
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
#include "../../../ArduCopter/Copter.h"
#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
#include <AP_UTB/AP_UTB_Reference.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Sample AP state and compute SHADOW diagnostics without motor output authority.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void Copter::utb_shadow_update().
//--------------------------------------------------------------------------------------------------------------------
void Copter::utb_shadow_update()
{
#if AP_UTB_BENCH_ENABLED
    // Příznak požadavku na interní diagnostickou referenci.
    bool synthetic;
#endif
#if MODE_ACRO_ENABLED
    // Platnost RC kalibrace před použitím AP mapování.
    bool calibration_valid;
    // Kalibrovaný střed vstupu plynu.
    int16_t mid;
    // Aktuální kalibrovaný povel plynu.
    int16_t control;
#endif

#if AP_UTB_BENCH_ENABLED
    // Rozsah objektu zachovává přesný začátek a konec měření producentu.
    UTBBench::Duration duration(UTBBench::Metric::PRODUCER);
    utb_bench_config();
#endif
    if (!utb.enabled()) {
        return;
    }
    // Vstupní snapshot vzniká až za ENABLE guardem, aby vypnutá cesta zůstala krátká.
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
#if AP_UTB_BENCH_ENABLED
    in.watchdog = hal.util->was_watchdog_reset();
    in.bench_safe = !any_failsafe_triggered() && g.failsafe_throttle.get() != FS_THR_DISABLED;
#endif
    if (utb.time_budget_latched()) {
        in.logging_available = utb.logging_available();
#if AP_UTB_BENCH_ENABLED
        utb.prepare_synthetic(in);
#endif
        utb.update(in);
        return;
    }
    utb.sample_state(in.state);
    in.validation_time_us = AP_HAL::micros();
    in.ap_target = utb.captured_target();
    in.ap_target.valid &= !in.fast_rate && in.ap_target.mode == in.mode;
    in.rc_input_ms = g2.rc_channels.last_input_ms();
    in.logging_available = utb.logging_available();
#if !MODE_ACRO_ENABLED
    in.acro_available = false;
#endif
#if AP_UTB_BENCH_ENABLED
    synthetic = utb.synthetic_requested();
    utb.prepare_synthetic(in);
    if (synthetic) {
        utb.update(in);
        return;
    }
#endif
#if MODE_ACRO_ENABLED
    const RC_Channel *channels[] = {channel_roll, channel_pitch, channel_yaw};
    calibration_valid = true;
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
    mid = calibration_valid ? get_throttle_mid() : 0;
    control = calibration_valid ? channel_throttle->get_control_in() : 0;
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
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include "utb_log.h"
#include "utb_bench_log.h"
#if AP_UTB_BENCH_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Sample the worker stack watermark using the platform diagnostic API.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Global exported function. Declaration/definition: void utb_bench_stack().
//--------------------------------------------------------------------------------------------------------------------
void utb_bench_stack();
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Create the logger worker during startup and handle thread creation failure.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void Copter::utb_log_init().
//--------------------------------------------------------------------------------------------------------------------
void Copter::utb_log_init()
{
#if HAL_LOGGING_ENABLED
    // Výsledek vytvoření logger worker threadu.
    bool created;
#endif

#if HAL_LOGGING_ENABLED
#if AP_UTB_BENCH_ENABLED
    UTBBench::creation(utb.enabled(), 3072);
#endif
    if (utb.enabled()) {
        // Allocation is confined to vehicle startup, never the producer.

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

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Drain the bounded diagnostic queue in the dedicated logger worker.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void Copter::utb_log_thread().
//--------------------------------------------------------------------------------------------------------------------
void Copter::utb_log_thread()
{
#if HAL_LOGGING_ENABLED
#if AP_UTB_BENCH_ENABLED
    // Čas posledního odečtu stack watermark v milisekundách.
    uint32_t last_stack_ms;
#endif
#endif
#if HAL_LOGGING_ENABLED
#if AP_UTB_BENCH_ENABLED
    // Monotónní začátek průchodu workeru v mikrosekundách.
    uint32_t pass_start;
#endif
#endif
#if HAL_LOGGING_ENABLED
    // Aktuální dostupnost loggeru pro zápis diagnostiky.
    bool available;
#if AP_UTB_BENCH_ENABLED
    // Monotónní konec průchodu workeru v mikrosekundách.
    uint32_t pass_done;
#endif
#endif
#if HAL_LOGGING_ENABLED
    // Index průchodu pevnou kolekcí.
    uint8_t i;
#endif

#if HAL_LOGGING_ENABLED
#if AP_UTB_BENCH_ENABLED
    UTBBench::worker_enter();
    last_stack_ms = 0;
#endif
    while (true) {
#if AP_UTB_BENCH_ENABLED
        pass_start = AP_HAL::micros();
        UTBBench::worker_start(pass_start);
        if (AP_HAL::millis() - last_stack_ms >= 1000) {
            utb_bench_stack();
            last_stack_ms = AP_HAL::millis();
        }
#endif
        available = logger.should_log(UINT32_MAX) && logger.logging_present() && logger.logging_started();
        utb.set_logging_available(available);
        // Snapshot je lokální jedné iteraci workeru; žádný nový heap ani zámek.
        AP_UTB::Snapshot status;
        if (utb.read_log_status(status)) {
            utb_status_update(status);
        }
        // Bounded drain, with a sleep even if the producer outpaces the backend.
        for (i = 0; i < AP_UTB::Queue::CAPACITY && available; i++) {
            if (!utb_log_update()) {
                break;
            }
        }
#if AP_UTB_BENCH_ENABLED
        pass_done = AP_HAL::micros();
        UTBBench::metrics[uint8_t(UTBBench::Metric::WORKER_PASS)].record(pass_done - pass_start);
        UTBBench::worker_finish(pass_done);
#endif
        hal.scheduler->delay(1);
    }
#endif
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Consume at most one diagnostic snapshot and report backend admission.
///
///  @param[in]  None
///  @return     bool.
///
///  @details    Non-public class member. Declaration/definition: bool Copter::utb_log_update().
//--------------------------------------------------------------------------------------------------------------------
bool Copter::utb_log_update()
{
#if HAL_LOGGING_ENABLED
    // Nejdelší dokončený consumer průchod v mikrosekundách.
    static uint32_t max_log_us;
    // Čas posledního uloženého status heartbeat v mikrosekundách.
    static uint64_t last_status_us;
    // Příznaky posledního uloženého stavu.
    static uint16_t last_flags;
    // Poslední uložený důvod platnosti výpočtu.
    static uint8_t last_calculation;
    // Poslední uložený stav dostupnosti diagnostiky.
    static uint8_t last_observation;
    // Poslední uložené rozhodnutí experimentální policy.
    static uint8_t last_policy;
#if AP_UTB_BENCH_ENABLED
    // Čas posledních uložených metadat reference v mikrosekundách.
    static uint64_t last_reference_us;
    // Klíč poslední uložené kombinace zdroje a fáze reference.
    static uint32_t last_reference_key = UINT32_MAX;
#endif
    // Čas posledního uloženého pipeline heartbeat v mikrosekundách.
    static uint64_t last_pipeline_us;
#endif
#if HAL_LOGGING_ENABLED
    // Monotónní začátek spotřeby snapshotu v mikrosekundách.
    uint32_t log_start_us;
    // Určuje, zda se podařilo vyjmout snapshot z fronty.
    bool popped;
    // Úspěšné předání všech částí diagnostické sady.
    bool complete;
    // Rozdíl požadovaného a dosaženého normalizovaného povelu.
    float residual;
    // Bity kladné/záporné saturace a blokování integrátoru dané osy.
    uint8_t sat;
    // Aktuální monotónní čas použitý touto diagnostickou operací.
    uint64_t now;
#if AP_UTB_BENCH_ENABLED
    // Identita zdroje, profilu a stavu interní reference.
    uint32_t reference_key;
#endif
#endif
#if HAL_LOGGING_ENABLED
    // Bitové příznaky stavu diagnostiky.
    uint16_t flags;
    // Číselný důvod platnosti nebo odmítnutí výpočtu.
    uint8_t calculation;
    // Číselný stav pozorovatelnosti diagnostiky.
    uint8_t observation;
    // Číselný výsledek bezpečnostní politiky experimentu.
    uint8_t policy;
    // Index průchodu pevnou kolekcí.
    uint8_t axis;
#endif

#if AP_UTB_BENCH_ENABLED
    // Rozsah objektu zachovává přesný začátek a konec měření consumeru.
    UTBBench::Duration duration(UTBBench::Metric::CONSUMER);
#endif
#if HAL_LOGGING_ENABLED
    log_start_us = AP_HAL::micros();
    // Worker only: backend semaphore waits cannot hold the producer queue lock.
    // Buffer patří pouze této consumer operaci; zachovaný rozsah omezuje stack.
    AP_UTB::Snapshot s;
    popped = utb.pop_log(s);
    if (popped && utb.logging_requested() && s.epoch == utb.epoch()) {
        complete = true;
        const auto &in = s.input;
        if (s.log_format == 1) {
            log_UTBC packed = {
                LOG_PACKET_HEADER_INIT(LOG_UTBC_MSG),
                in.time_us, s.seq, s.epoch, in.mode, in.ap_target.time_us, in.ap_target.seq,
                in.ap_target.mode, uint8_t(in.ap_target.valid && in.ap_target.seq == s.seq),
                in.rc_input_ms, {}, s.drops, s.missing, s.effective_log_rate
            };
            UTBLogPacket::encode(s, packed.data);
            complete = logger.WriteBlock_first_succeed(&packed, sizeof(packed));
        } else {
            for (axis = 0; axis < 3; axis++) {
                const auto &a = s.control.axis[axis];
                residual = a.raw - s.mixer.achieved[axis];
                sat = (residual > 1.0e-5f ? 1 : 0) | (residual < -1.0e-5f ? 2 : 0) |
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
        }
        utb.log_result(complete);
    } else if (popped) {
        utb.log_discarded();
    }
    // Samostatný status buffer vzniká až po zpracování datové sady.
    AP_UTB::Snapshot current;
    if (popped && s.epoch == utb.epoch() && utb.logging_requested()) {
        current = s;
    } else if (!utb.read_log_status(current)) {
        return popped;
    }
    now = AP_HAL::micros64();
#if AP_UTB_BENCH_ENABLED
    const auto &reference = current.input;
    reference_key = reference.reference_source | (uint32_t(reference.synthetic_profile) << 8) |
                    (uint32_t(reference.synthetic_phase) << 16) | (uint32_t(reference.synthetic_status) << 24);
    if (reference_key != last_reference_key || now - last_reference_us >= 1000000U) {
        const log_UTBX metadata = { LOG_PACKET_HEADER_INIT(LOG_UTBX_MSG), reference.time_us, current.seq,
                                    utb.logging_requested() ? current.epoch : utb.epoch(), reference.synthetic_cycle, reference.reference_source,
                                    reference.synthetic_profile, reference.synthetic_phase, reference.synthetic_status
                                  };
        if (logger.WriteBlock_first_succeed(&metadata, sizeof(metadata))) {
            last_reference_us = now;
            last_reference_key = reference_key;
        }
    }
#endif
    flags = utb.logging_requested() ? current.flags : AP_UTB::ENABLED | AP_UTB::LOGGING_AVAILABLE;
    calculation = uint8_t(current.calculation);
    observation = uint8_t(current.observation);
    policy = uint8_t(current.policy);
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
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Utb status update.
///
///  @param[in]  s S.
///  @return     None.
///
///  @details    Non-public class member. Declaration/definition: void Copter::utb_status_update(const AP_UTB::Snapshot &s).
//--------------------------------------------------------------------------------------------------------------------
void Copter::utb_status_update(const AP_UTB::Snapshot &s)
{
    // Poslední ohlášený společný klíč stavu.
    static uint32_t last_key = UINT32_MAX;
    // Čas posledního telemetrického statusu v milisekundách.
    static uint32_t last_ms;
    // Aktuální monotónní čas použitý touto diagnostickou operací.
    uint32_t now;
    // Bitové příznaky stavu diagnostiky.
    uint16_t flags;
    // Identifikátor aktuálního exportovaného stavu nebo metriky.
    uint32_t key;

    if (!utb.enabled()) {
        return;
    }
    now = AP_HAL::millis();
    flags = utb.logging_requested() ? s.flags : uint16_t(AP_UTB::ENABLED);
    key = uint32_t(flags) | (uint32_t(s.calculation) << 10) |
          (uint32_t(s.observation) << 16) | (uint32_t(s.policy) << 20);
    if (now - last_ms >= 1000 && (key != last_key || now - last_ms >= 5000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UTB flags=%u calc=%u obs=%u policy=%u",
                      unsigned(flags), unsigned(s.calculation), unsigned(s.observation), unsigned(s.policy));
        last_key = key;
        last_ms = now;
    }
}
#endif // AP_UTB_ENABLED

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Private functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
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
