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
#include <AP_UTB/AP_UTB_config.h>
#if AP_UTB_ENABLED && HAL_LOGGING_ENABLED
#include <AP_Logger/LogStructure.h>

// @LoggerMessage: UTBS
// @Description: Shadow diagnostic health and policy; flags never grant motor authority
// @Field: TimeUS: Shadow sample timestamp in microseconds since boot
// @Field: Seq: Shadow cycle sequence number
// @Field: Md: ArduPilot flight mode number
// @Field: Flg: Health bitmask: enabled/requested/state/evaluated/valid/logger/healthy/observable, see AP_UTB.h
// @Field: Calc: CalculationReason enum from AP_UTB.h
// @Field: Obs: ObservationReason enum from AP_UTB.h
// @Field: Pol: Experiment policy enum from AP_UTB.h
// @Field: Dt: Actual scheduler loop interval in seconds
// @Field: Ex: Shadow execution time in microseconds; SITL stopped clock may give zero
// @Field: Max: Maximum observed shadow execution time in microseconds
// @Field: Req: Requested diagnostic sets per second
// @Field: Eff: Successfully enqueued complete sets per elapsed experiment second, not persisted rate
// @Field: Dr: Diagnostic sets dropped on bounded queue overflow
// @Field: Miss: Sets with at least one backend write failure
// @Field: Ov: Shadow time budget overruns
// @Field: LM: Maximum observed logger consumer execution time in microseconds
struct PACKED log_UTBS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t mode;
    uint16_t flags;
    uint8_t calculation;
    uint8_t observation;
    uint8_t policy;
    float dt;
    uint32_t execution_us;
    uint32_t max_us;
    uint16_t requested;
    float effective;
    uint32_t drops;
    uint32_t missing;
    uint32_t overruns;
    uint32_t logger_max_us;
};

// @LoggerMessage: UTBR
// @Description: One body-rate axis of the AP-derived UTB shadow experiment; not an AP control command
// @Field: TimeUS: Shadow sample timestamp in microseconds since boot
// @Field: Seq: Shadow cycle sequence number
// @Field: Md: ArduPilot flight mode number
// @Field: Ax: Body axis 0 roll, 1 pitch, 2 yaw
// @Field: Des: AP-derived shadow reference in rad/s
// @Field: Meas: AHRS primary filtered gyro in rad/s
// @Field: Err: Reference minus measured rate in rad/s
// @Field: P: Proportional term normalized
// @Field: I: Integral term normalized
// @Field: D: Measurement derivative term normalized
// @Field: Raw: P plus I plus D normalized
// @Field: Out: Clamped normalized PID output
// @Field: AP: Captured AP body-rate target including SYSID in rad/s
// @Field: Val: Current controller or mixer result validity
// @Field: Sat: Bits 0 positive residual, 1 negative residual, 2 previous-step integration gate blocked
struct PACKED log_UTBR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t mode;
    uint8_t axis;
    float desired;
    float measured;
    float error;
    float p;
    float i;
    float d;
    float raw;
    float output;
    float ap_target;
    uint8_t valid;
    uint8_t saturation;
};

// @LoggerMessage: UTBM
// @Description: Hypothetical BF_X normalized motor thrusts; never ESC outputs
// @Field: TimeUS: Shadow sample timestamp in microseconds since boot
// @Field: Seq: Shadow cycle sequence number
// @Field: Md: ArduPilot flight mode number
// @Field: R: Normalized R in logical BF_X motor or body axis order
// @Field: P: Proportional term normalized
// @Field: Y: Normalized Y in logical BF_X motor or body axis order
// @Field: T: Normalized T in logical BF_X motor or body axis order
// @Field: M1: Normalized M1 in logical BF_X motor or body axis order
// @Field: M2: Normalized M2 in logical BF_X motor or body axis order
// @Field: M3: Normalized M3 in logical BF_X motor or body axis order
// @Field: M4: Normalized M4 in logical BF_X motor or body axis order
// @Field: S: Common moment scale
// @Field: Shift: Collective displacement or shifted flag according to packet
// @Field: Val: Current controller or mixer result validity
// @Field: Lo: Motor lower-bound bitmask M1 through M4
// @Field: Hi: Motor upper-bound bitmask M1 through M4
struct PACKED log_UTBM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
    float thrust;
    float m1;
    float m2;
    float m3;
    float m4;
    float scale;
    float shift;
    uint8_t valid;
    uint8_t lower;
    uint8_t upper;
};

// @LoggerMessage: UTBA
// @Description: Achieved normalized allocation and directional masks; no AP limits
// @Field: TimeUS: Shadow sample timestamp in microseconds since boot
// @Field: Seq: Shadow cycle sequence number
// @Field: Md: ArduPilot flight mode number
// @Field: R: Achieved normalized R
// @Field: P: Achieved normalized P
// @Field: Y: Achieved normalized Y
// @Field: T: Achieved normalized T
// @Field: Pos: Positive directional limitations, bits R P Y collective
// @Field: Neg: Negative directional limitations, bits R P Y collective
// @Field: Scale: Moments scaled flag
// @Field: Shift: Collective displacement or shifted flag according to packet
struct PACKED log_UTBA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
    float thrust;
    uint8_t positive;
    uint8_t negative;
    uint8_t scaled;
    uint8_t shifted;
};

// @LoggerMessage: UTBT
// @Description: Shadow sample alignment, timing and diagnostic loss counters
// @Field: TimeUS: Shadow sample timestamp in microseconds since boot
// @Field: Seq: Shadow cycle sequence number
// @Field: Md: ArduPilot flight mode number
// @Field: APUS: Read-only AP target capture timestamp in microseconds
// @Field: APS: AP target capture sequence
// @Field: APM: AP flight mode at capture
// @Field: Cap: Capture validity including sequence match
// @Field: RCms: Last accepted RC input timestamp in milliseconds, not a new sample per loop
// @Field: Dt: Actual scheduler loop interval in seconds
// @Field: Ex: Shadow execution time in microseconds; SITL stopped clock may give zero
// @Field: Max: Maximum observed shadow execution time in microseconds
// @Field: Req: Requested diagnostic sets per second
// @Field: Eff: Successfully enqueued complete sets per elapsed experiment second, not persisted rate
// @Field: Drop: Diagnostic sets dropped on bounded queue overflow
// @Field: Miss: Sets with at least one backend write failure
struct PACKED log_UTBT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t mode;
    uint64_t ap_time_us;
    uint32_t ap_seq;
    uint8_t ap_mode;
    uint8_t capture_valid;
    uint32_t rc_input_ms;
    float dt;
    uint32_t execution_us;
    uint32_t max_us;
    uint16_t requested;
    float effective;
    uint32_t drops;
    uint32_t missing;
};

// @LoggerMessage: UTBQ
// @Description: Cumulative diagnostic pipeline counters; written means backend enqueue, not persistence
// @Field: TimeUS: Consumer timestamp in microseconds since boot
// @Field: Seq: Latest shadow sample sequence
// @Field: ReqN: Requested diagnostic sets
// @Field: EnqN: Sets accepted by the bounded queue
// @Field: WrN: Complete sets accepted by the first logger backend
// @Field: Drop: Producer sets dropped on full queue or lock contention
// @Field: Miss: Sets with a failed backend enqueue
// @Field: HWM: Queue high-water mark since boot
// @Field: Depth: Observed queue depth
// @Field: Epoch: Experiment generation used to reject obsolete queued samples
// @Field: Gy: Primary gyro index in the status sample, 255 when no state sampled
// @Field: Gone: Obsolete or disabled experiment sets discarded by consumer
struct PACKED log_UTBQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint32_t requested;
    uint32_t enqueued;
    uint32_t written;
    uint32_t dropped;
    uint32_t missing;
    uint8_t high_water;
    uint8_t depth;
    uint32_t epoch;
    uint8_t primary_gyro;
    uint32_t discarded;
};

#define UTB_LOG_STRUCTURES \
    { LOG_UTBQ_MSG, sizeof(log_UTBQ), "UTBQ", "QIIIIIIBBIBI", "TimeUS,Seq,ReqN,EnqN,WrN,Drop,Miss,HWM,Depth,Epoch,Gy,Gone", "s-----------", "F-----------" }, \
    { LOG_UTBS_MSG, sizeof(log_UTBS), "UTBS", "QIBHBBBfIIHfIIII", "TimeUS,Seq,Md,Flg,Calc,Obs,Pol,Dt,Ex,Max,Req,Eff,Dr,Miss,Ov,LM", "s------ssszz---s", "F-------FF-----F" }, \
    { LOG_UTBR_MSG, sizeof(log_UTBR), "UTBR", "QIBBfffffffffBB", "TimeUS,Seq,Md,Ax,Des,Meas,Err,P,I,D,Raw,Out,AP,Val,Sat", "s---EEE-----E--", "F--------------" }, \
    { LOG_UTBM_MSG, sizeof(log_UTBM), "UTBM", "QIBffffffffffBBB", "TimeUS,Seq,Md,R,P,Y,T,M1,M2,M3,M4,S,Shift,Val,Lo,Hi", "s---------------", "F---------------" }, \
    { LOG_UTBA_MSG, sizeof(log_UTBA), "UTBA", "QIBffffBBBB", "TimeUS,Seq,Md,R,P,Y,T,Pos,Neg,Scale,Shift", "s----------", "F----------" }, \
    { LOG_UTBT_MSG, sizeof(log_UTBT), "UTBT", "QIBQIBBIfIIHfII", "TimeUS,Seq,Md,APUS,APS,APM,Cap,RCms,Dt,Ex,Max,Req,Eff,Drop,Miss", "s--s---sssszz--", "F--F---C-FF----" },
#endif // AP_UTB_ENABLED && HAL_LOGGING_ENABLED
