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
#if AP_UTB_RATE_SITL_AVAILABLE && HAL_LOGGING_ENABLED
#include <AP_Logger/LogStructure.h>
// @LoggerMessage: UARX
// @Description: SITL-only actual rate-owner sample before the single motor output
// @Field: TimeUS: INS/control sample time since boot
// @Field: Seq: Mode-local scheduler cycle
// @Field: Ax: Body axis 0 roll 1 pitch 2 yaw
// @Field: Own: Fixed owner 1 UTB 2 AP comparison
// @Field: Flt: Latched AP_UTB_SITLRate fault
// @Field: Arm: Simulator armed state
// @Field: Spl: AP_Motors spool state
// @Field: Dt: Actual virtual scheduler interval seconds
// @Field: Ref: RATE-only raw previous-mode-update reference; attitude mode selected body rate target (angles in UAAT)
// @Field: Tar: Body rate target used this sample; AP shaping only in RATE-only or explicit AP attitude comparison
// @Field: Gyr: Actual AHRS primary gyro radians per second
// @Field: P: Actual selected controller proportional term
// @Field: I: Actual selected controller integral term
// @Field: D: Actual selected controller derivative term
// @Field: Out: Actual normalized request handed to AP_Motors
struct PACKED log_UARX {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t axis, owner, fault, armed, spool;
    float dt, reference, target, gyro, p, i, d, output;
};
// @LoggerMessage: UAMT
// @Description: SITL-only outputs read after the single existing AP_Motors chain
// @Field: TimeUS: Matching UARX control sample time
// @Field: Seq: Matching UARX sequence
// @Field: Ph: 0 waiting 1 running 2 complete 3 latched fault
// @Field: Flt: Latched AP_UTB_SITLRate fault
// @Field: C1: Actual simulated channel 1 PWM
// @Field: C2: Actual simulated channel 2 PWM
// @Field: C3: Actual simulated channel 3 PWM
// @Field: C4: Actual simulated channel 4 PWM
// @Field: M1: Raw normalized AP_Motors actuator 1
// @Field: M2: Raw normalized AP_Motors actuator 2
// @Field: M3: Raw normalized AP_Motors actuator 3
// @Field: M4: Raw normalized AP_Motors actuator 4
// @Field: APN: AP rate PID invocation count in this mode
// @Field: UN: UTB rate PID invocation count in this mode
// @Field: MN: Existing motor output callback count in this mode
// @Field: Ex: Rate slot host steady-clock duration microseconds, not virtual dt
struct PACKED log_UAMT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t phase, fault;
    uint16_t pwm[4];
    float actuator[4];
    uint32_t ap_calls, utb_calls, motor_calls, execution_us;
};
// @LoggerMessage: UAFB
// @Description: SITL verified normalized allocation, not physical torque
// @Field: TimeUS: Matching UARX control time
// @Field: Seq: Output cycle sequence
// @Field: CSeq: PID cycle issuing the transported raw command
// @Field: FBUS: Post-output capture timestamp, uint32 virtual microseconds
// @Field: Ax: Body axis R/P/Y 0/1/2
// @Field: Av: Allocation reconstruction valid
// @Field: PV: Transported PID sample valid
// @Field: Lim: Actual AP allocation limit flags R/P/Y/lower/upper
// @Field: Blk: Current UTB integration gate bits R/P/Y
// @Field: G: Actual compensation gain
// @Field: Raw: UTB transported PID before clipping; AP owner records its applied normalized request
// @Field: Cmd: Actual transported request handed to allocator
// @Field: Alc: Actual normalized allocated component
// @Field: Res: Raw minus allocated component
struct PACKED log_UAFB {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq, command_seq, capture_us;
    uint8_t axis, valid, pid_valid, limits, blocked;
    float gain, raw, command, allocated, residual;
};
// @LoggerMessage: UALT
// @Description: SITL actual pre-linearization allocation and inverse actuator commands
// @Field: TimeUS: Matching UARX/UAFB sample time
// @Field: Seq: Output cycle sequence
// @Field: G: Compensation gain
// @Field: T1: Actual pre-linearization motor 1 command
// @Field: T2: Actual pre-linearization motor 2 command
// @Field: T3: Actual pre-linearization motor 3 command
// @Field: T4: Actual pre-linearization motor 4 command
// @Field: P1: Inverse-linearized compensated motor 1 command
// @Field: P2: Inverse-linearized compensated motor 2 command
// @Field: P3: Inverse-linearized compensated motor 3 command
// @Field: P4: Inverse-linearized compensated motor 4 command
// @Field: Av: Reconstruction valid
// @Field: Lim: Actual limit flags
struct PACKED log_UALT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    float gain, thrust[4], post_thrust[4];
    uint8_t valid, limits;
};
// @LoggerMessage: UAAT
// @Description: SITL reduced attitude loop, radians and body rates radians per second
// @Field: TimeUS: Matching rate sample timestamp
// @Field: Seq: Matching rate cycle sequence
// @Field: Src: Outer source 1 UTB 2 AP comparison
// @Field: Flt: UTB attitude fault
// @Field: RRef: Requested Euler roll radians
// @Field: PRef: Requested Euler pitch radians
// @Field: Roll: Sampled actual Euler roll radians
// @Field: Pitch: Sampled actual Euler pitch radians
// @Field: Yaw: Sampled actual heading radians
// @Field: ER: UTB reduced rotation body X error radians, zero for AP source
// @Field: EP: UTB reduced rotation body Y error radians, zero for AP source
// @Field: RR: Selected body roll rate radians per second
// @Field: PR: Selected body pitch rate radians per second
// @Field: YR: Selected body yaw rate radians per second
// @Field: Lim: Outer roll/pitch rate norm clipped
struct PACKED log_UAAT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t source, fault;
    float roll_ref, pitch_ref, roll, pitch, yaw, error_roll, error_pitch, roll_rate, pitch_rate, yaw_rate;
    uint8_t limited;
};
// @LoggerMessage: UAST
// @Description: SITL outer authority counts and vertical envelope, no altitude control
// @Field: TimeUS: Matching rate sample timestamp
// @Field: Seq: Matching rate cycle sequence
// @Field: UN: UTB attitude invocations
// @Field: AN: AP attitude invocations
// @Field: Alt: AHRS height above EKF origin meters
// @Field: VZ: AHRS vertical velocity positive up meters per second
// @Field: Thr: Applied manual normalized throttle
struct PACKED log_UAST {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq, utb_calls, ap_calls;
    float altitude, vertical_speed, throttle;
};
#define UTB_SITL_LOG_STRUCTURES \
    { LOG_UARX_MSG, sizeof(log_UARX), "UARX", "QIBBBBBffffffff", \
      "TimeUS,Seq,Ax,Own,Flt,Arm,Spl,Dt,Ref,Tar,Gyr,P,I,D,Out", "s------sEEE----", "F------0000----" }, \
    { LOG_UAMT_MSG, sizeof(log_UAMT), "UAMT", "QIBBHHHHffffIIII", \
      "TimeUS,Seq,Ph,Flt,C1,C2,C3,C4,M1,M2,M3,M4,APN,UN,MN,Ex", "s---YYYY-------s", "F--------------F" }, \
    { LOG_UAFB_MSG, sizeof(log_UAFB), "UAFB", "QIIIBBBBBfffff", \
      "TimeUS,Seq,CSeq,FBUS,Ax,Av,PV,Lim,Blk,G,Raw,Cmd,Alc,Res", "s--s----------", "F--F----------" }, \
    { LOG_UALT_MSG, sizeof(log_UALT), "UALT", "QIfffffffffBB", \
      "TimeUS,Seq,G,T1,T2,T3,T4,P1,P2,P3,P4,Av,Lim", "s------------", "F------------" }, \
    { LOG_UAAT_MSG, sizeof(log_UAAT), "UAAT", "QIBBffffffffffB", \
      "TimeUS,Seq,Src,Flt,RRef,PRef,Roll,Pitch,Yaw,ER,EP,RR,PR,YR,Lim", "s---rrrrrrrEEE-", "F---0000000000-" }, \
    { LOG_UAST_MSG, sizeof(log_UAST), "UAST", "QIIIfff", \
      "TimeUS,Seq,UN,AN,Alt,VZ,Thr", "s---mn-", "F---00-" },
#endif
