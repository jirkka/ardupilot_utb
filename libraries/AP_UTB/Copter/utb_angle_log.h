//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file utb_angle_log.h
/// @brief Pevná RC a režimová evidence pilotního ANGLE experimentu, pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-19
/// @date 2026-10-19
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
// @Field: Out: Manual throttle before native ANGLE angle boost, after spool and internal validation
// @Field: V: Pilot snapshot valid
// @Field: RCF: Legacy RC reason; UTB3A 0 valid 1 AP RC input unavailable
// @Field: Stop: Legacy stop code; UTB3A 0 valid 255 current state/output invalid, not a latched stop
// @Field: Md: Actual pilot mode number 42 ANGLE or 43 ACRO
// @Field: Age: Age of the sampled AP RC data in milliseconds
// @Field: Mix: Native AP throttle versus attitude headroom mix
// @Field: Log: Legacy post-output callback host duration microseconds; UTB3A reserved zero
// @Field: Run: Legacy mode-run host duration microseconds; UTB3A reserved zero
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
