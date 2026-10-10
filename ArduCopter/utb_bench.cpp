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
#include "utb_bench_log.h"
#if AP_UTB_BENCH_ENABLED
void utb_bench_platform_init();
static UTBBench::Frame<4> bench_config;
void Copter::utb_bench_init()
{
    utb_bench_platform_init();
    const uint32_t initial[4] = {1, 0, 0, 0};
    bench_config.publish(initial);
    UTBBench::exporter_life.store(uint32_t(UTBBench::Life::CREATE_ATTEMPT));
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Copter::utb_bench_thread, void),
                                      "utb_bench", 3072, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        UTBBench::exporter_life.store(uint32_t(UTBBench::Life::CREATE_FAILED));
    }
}
void Copter::utb_bench_config()
{
#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
    const uint16_t rate = utb.bench_log_rate();
    const uint32_t values[4] = {
        uint32_t(!utb.enabled() ? 1 : !utb.shadow_requested() ? 2 : rate == 200 ? 3 : rate == 400 ? 4 : 5),
        utb.epoch(), rate, utb.bench_wait_us()
    };
    uint32_t previous[4];
    if (!bench_config.read(previous) || memcmp(previous, values, sizeof(values)) != 0) {
        bench_config.publish(values);
        UTBBench::wait_limit.store(values[3]);
    }
#endif
}
static bool bench_telemetry(uint8_t key, uint32_t value)
{
#if HAL_GCS_ENABLED
    const mavlink_channel_t chan = MAVLINK_COMM_0;
    if (gcs().chan(uint8_t(chan)) == nullptr || !comm_chan_lock(chan).take_nonblocking()) {
        return false;
    }
    bool sent = false;
    if (comm_get_txspace(chan) >= MAVLINK_MAX_PACKET_LEN) {
        char name[10] = {'U','B',char('0' + key / 10),char('0' + key % 10),0};
        mavlink_message_t msg;
        mavlink_msg_named_value_int_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan,
                                              &msg, AP_HAL::millis(), name, int32_t(value));
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(bytes, &msg);
        comm_send_buffer(chan, bytes, len);
        sent = true;
    }
    comm_chan_lock(chan).give();
    return sent;
#else
    return false;
#endif
}
void Copter::utb_bench_thread()
{
    UTBBench::exporter_life.store(uint32_t(UTBBench::Life::RUNNING));
    while (true) {
        hal.scheduler->delay(1000);
        UTBBench::Duration duration(UTBBench::Metric::EXPORT);
        const uint64_t now = AP_HAL::micros64();
        uint32_t config[4] = {255, 0, 0, 0};
        const bool config_valid = bench_config.read(config);
        const uint32_t epoch = config[1];
        const uint8_t variant = config_valid ? config[0] : 255;
#if !HAL_LOGGING_ENABLED
        (void)now; (void)epoch; (void)variant;
#endif
        bool complete = config_valid;
        for (uint8_t i = 0; i < uint8_t(UTBBench::Metric::COUNT); i++) {
            UTBBench::Sample s;
            if (!UTBBench::read_metric(i, s)) {
                complete = false;
                continue;
            }
#if HAL_LOGGING_ENABLED
            const log_UBMT metric = { LOG_PACKET_HEADER_INIT(LOG_UBMT_MSG), now, epoch, variant,
                                      i, s.count, s.sum, s.minimum, s.maximum, s.last
                                    };
            complete &= logger.WriteBlock_first_succeed(&metric, sizeof(metric));
            log_UBHI hist = { LOG_PACKET_HEADER_INIT(LOG_UBHI_MSG), now, epoch, i, {} };
            memcpy(hist.h, s.bins, sizeof(hist.h));
            complete &= logger.WriteBlock_first_succeed(&hist, sizeof(hist));
#endif
        }
        uint32_t values[] = {
            variant, epoch, config[2], config[3],
            UTBBench::exceeds.load(), UTBBench::collisions.load(), UTBBench::collision_max.load(),
            UTBBench::unknown.load(), UTBBench::task_overruns.load(), UTBBench::deadlines.load(),
            UTBBench::life.load(), UTBBench::worker_loops.load(), UTBBench::worker_begin.load(),
            UTBBench::worker_done.load(), UTBBench::stack_requested.load(), UTBBench::stack_total.load(),
            UTBBench::stack_free_min.load(), UTBBench::export_skips.load(), UTBBench::exporter_life.load(),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };
#if AP_UTB_ENABLED && FRAME_CONFIG == MULTICOPTER_FRAME
        const auto pipe = utb.pipeline();
        values[19] = pipe.requested; values[20] = pipe.enqueued; values[21] = pipe.written;
        values[22] = pipe.dropped; values[23] = pipe.missing; values[24] = pipe.high_water;
        values[25] = pipe.depth; values[26] = pipe.discarded;
#endif
        const UTBBench::Metric timing[] = {UTBBench::Metric::MAIN_ACQUIRE, UTBBench::Metric::WORKER_HOLD,
                                           UTBBench::Metric::LOOP_PERIOD, UTBBench::Metric::PRODUCER, UTBBench::Metric::CONSUMER,
                                           UTBBench::Metric::EXPORT
                                          };
        for (uint8_t i = 0; i < ARRAY_SIZE(timing); i++) {
            UTBBench::Sample sample;
            if (UTBBench::metrics[uint8_t(timing[i])].read(sample)) {
                values[27 + i] = sample.maximum;
            } else {
                complete = false;
            }
        }
        values[33] = UTBBench::measured_backend.load();
        values[34] = UTBBench::create_attempts.load();
        values[35] = UTBBench::create_failures.load();
#if HAL_LOGGING_ENABLED
        values[36] = logger.should_log(UINT32_MAX) && logger.logging_present() && logger.logging_started();
#endif
        for (uint8_t key = 0; key < ARRAY_SIZE(values); key++) {
#if HAL_LOGGING_ENABLED
            const log_UBST state = { LOG_PACKET_HEADER_INIT(LOG_UBST_MSG), now, epoch, variant, key, values[key] };
            complete &= logger.WriteBlock_first_succeed(&state, sizeof(state));
#endif
            complete &= bench_telemetry(key, values[key]);
        }
        if (!complete) {
            UTBBench::export_skips.store(UTBBench::export_skips.load() + 1);
        }
    }
}
#endif // AP_UTB_BENCH_ENABLED
