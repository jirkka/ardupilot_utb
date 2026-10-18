//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file utb_angle_log.h
/// @brief Pevná RC a režimová evidence pilotního ANGLE experimentu, pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-17
/// @date 2026-10-17
//--------------------------------------------------------------------------------------------------------------------
#pragma once
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
//====================================================================================================================
// Include Files
//--------------------------------------------------------------------------------------------------------------------
#include <AP_UTB/AP_UTB_config.h>
#if AP_UTB_RATE_SITL_AVAILABLE && HAL_LOGGING_ENABLED
#include <AP_Logger/LogStructure.h>
// End Include Files
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// @LoggerMessage: UARC
// @Description: SITL pilot ANGLE/ACRO actual AP RC sample and manual throttle provenance
// @Field: TimeUS: Matching controller and motor sample timestamp in microseconds
// @Field: Seq: Matching ANGLE scheduler sequence
// @Field: RCms: AP RC sample timestamp in milliseconds, captured in preceding mode run
// @Field: NR: Actual AP calibrated and reversed normalized roll with deadband
// @Field: NP: Actual AP calibrated and reversed normalized pitch with deadband
// @Field: NY: Actual AP calibrated and reversed normalized yaw with deadband
// @Field: Thr: AP manual throttle map output from the same pilot sample
// @Field: Out: Manual throttle handed to AP after spool and fault guards
// @Field: V: Pilot snapshot valid
// @Field: RCF: RC reason 0 none 1 config 2 signal 3 stale 4 calibration 5 range 6 throttle
// @Field: Stop: Latched mode reason 0 none 1 config 2 RC 3 state 4 attitude 5 rate 6 vertical 7 failsafe
// @Field: Md: Actual pilot mode number 42
// @Field: Age: Age of the sampled AP RC data in milliseconds
// @Field: Mix: Explicit AP manual throttle versus attitude headroom mix
// @Field: Log: Previous post-output capture and log callback host duration in microseconds
// @Field: Run: Last pilot mode run host duration in microseconds
struct PACKED log_UARC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq, rc_ms;
    float normalized_roll, normalized_pitch, normalized_yaw, throttle, applied;
    uint8_t valid, rc_fault, stop, mode;
    uint32_t age;
    float mix;
    uint32_t log_us, run_us;
};
#define UTB_ANGLE_LOG_STRUCTURES \
    { LOG_UARC_MSG, sizeof(log_UARC), "UARC", "QIIfffffBBBBIfII", \
      "TimeUS,Seq,RCms,NR,NP,NY,Thr,Out,V,RCF,Stop,Md,Age,Mix,Log,Run", \
      "s-s---------s-ss", "F-C---------C-FF" },
// End Data type, constant, macro definition
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
// None
// End Static data declarations
//====================================================================================================================
// Private function prototypes
//--------------------------------------------------------------------------------------------------------------------
// None
// End Private function prototypes
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
// None
// End Public functions
//====================================================================================================================
// Private functions
//--------------------------------------------------------------------------------------------------------------------
// None
// End Private functions
#endif // AP_UTB_RATE_SITL_AVAILABLE && HAL_LOGGING_ENABLED
