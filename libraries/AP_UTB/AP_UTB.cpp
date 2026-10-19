//--------------------------------------------------------------------------------------------------------------------
//  NOTICE: UTB project extension; existing licenses and copyright notices are retained.
//--------------------------------------------------------------------------------------------------------------------
///  @file       AP_UTB.cpp
///  @brief      Manager SHADOW, parametry, reset, oddělená matematická/logging policy, bounded queue a epochy.
///  @author     Jiri Kovar
///  @version    2026-10-18
///  @date       2026-10-18
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
#include "AP_UTB.h"
#if AP_UTB_ENABLED
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Include Files
//====================================================================================================================
// Static data declarations
//--------------------------------------------------------------------------------------------------------------------
constexpr uint8_t AP_UTB::Queue::CAPACITY;
static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_CHAR_LOCK_FREE == 2 && ATOMIC_BOOL_LOCK_FREE == 2, "UTB feedback atomics must not lock");
const AP_Param::GroupInfo AP_UTB::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: UTB diagnostic enable
    // @Description: Enables disarmed-only UTB mode and optional shadow diagnostics after reboot. Never grants motor authority or arming permission.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_UTB, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SHADOW
    // @DisplayName: UTB shadow experiment
    // @Description: Runs diagnostic rate PID and BF_X allocation beside AP control. Requires logging to start. No actuator output. Off resets history and time budget latch.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SHADOW", 2, AP_UTB, _shadow, 0),

    // @Param: LOG_RATE
    // @DisplayName: UTB diagnostic log rate
    // @Description: Requested diagnostic sets per second, independent of controller rate. 400 Hz is experimental. Hardware CPU and logger limits remain unverified.
    // @Range: 1 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("LOG_RATE", 3, AP_UTB, _log_rate, 200),

    // @Param: RAT_RLL_P
    // @DisplayName: UTB shadow RLL P
    // @Description: Diagnostic RLL P, units s/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_P", 4, AP_UTB, _p[0], 0),

    // @Param: RAT_RLL_I
    // @DisplayName: UTB shadow RLL I
    // @Description: Diagnostic RLL I, units 1/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_I", 5, AP_UTB, _i[0], 0),

    // @Param: RAT_RLL_D
    // @DisplayName: UTB shadow RLL D
    // @Description: Diagnostic RLL D, units s^2/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_D", 6, AP_UTB, _d[0], 0),

    // @Param: RAT_RLL_IMAX
    // @DisplayName: UTB shadow RLL IMAX
    // @Description: Diagnostic RLL IMAX, units dimensionless. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_IMAX", 7, AP_UTB, _imax[0], 0),

    // @Param: RAT_RLL_D_HZ
    // @DisplayName: UTB shadow RLL D_HZ
    // @Description: Diagnostic RLL D_HZ, units Hz. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RAT_RLL_D_HZ", 8, AP_UTB, _d_hz[0], 0),

    // @Param: RAT_PIT_P
    // @DisplayName: UTB shadow PIT P
    // @Description: Diagnostic PIT P, units s/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_P", 9, AP_UTB, _p[1], 0),

    // @Param: RAT_PIT_I
    // @DisplayName: UTB shadow PIT I
    // @Description: Diagnostic PIT I, units 1/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_I", 10, AP_UTB, _i[1], 0),

    // @Param: RAT_PIT_D
    // @DisplayName: UTB shadow PIT D
    // @Description: Diagnostic PIT D, units s^2/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_D", 11, AP_UTB, _d[1], 0),

    // @Param: RAT_PIT_IMAX
    // @DisplayName: UTB shadow PIT IMAX
    // @Description: Diagnostic PIT IMAX, units dimensionless. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_IMAX", 12, AP_UTB, _imax[1], 0),

    // @Param: RAT_PIT_D_HZ
    // @DisplayName: UTB shadow PIT D_HZ
    // @Description: Diagnostic PIT D_HZ, units Hz. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RAT_PIT_D_HZ", 13, AP_UTB, _d_hz[1], 0),

    // @Param: RAT_YAW_P
    // @DisplayName: UTB shadow YAW P
    // @Description: Diagnostic YAW P, units s/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_P", 14, AP_UTB, _p[2], 0),

    // @Param: RAT_YAW_I
    // @DisplayName: UTB shadow YAW I
    // @Description: Diagnostic YAW I, units 1/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_I", 15, AP_UTB, _i[2], 0),

    // @Param: RAT_YAW_D
    // @DisplayName: UTB shadow YAW D
    // @Description: Diagnostic YAW D, units s^2/rad. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_D", 16, AP_UTB, _d[2], 0),

    // @Param: RAT_YAW_IMAX
    // @DisplayName: UTB shadow YAW IMAX
    // @Description: Diagnostic YAW IMAX, units dimensionless. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_IMAX", 17, AP_UTB, _imax[2], 0),

    // @Param: RAT_YAW_D_HZ
    // @DisplayName: UTB shadow YAW D_HZ
    // @Description: Diagnostic YAW D_HZ, units Hz. Zero default is not a flight tune. Positive D requires positive cutoff no greater than 10 percent of nominal controller rate. Changing gains resets history.
    // @Range: 0 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RAT_YAW_D_HZ", 18, AP_UTB, _d_hz[2], 0),

#if AP_UTB_BENCH_ENABLED
    // @Param: B_SYNTH
    // @DisplayName: Disarmed internal shadow reference
    // @Description: BENCH only. 400 zero-reference preroll cycles then a finite 8000-cycle sequence, amplitude 0.3 rad/s. Requires healthy real sensors, 400 Hz and logger availability; faults or completion latch off until set to zero. Profile 2 is rejected outside SITL. Never grants motor authority.
    // @Values: 0:RC reference,1:Small internal sequence,2:SITL saturation sequence
    // @User: Advanced
    AP_GROUPINFO("B_SYNTH", 22, AP_UTB, _synthetic_request, 0),

    // @Param: B_WAIT_US
    // @DisplayName: Benchmark diagnostic acquire threshold
    // @Description: Counts main BLOCK logger acquire durations above this threshold. Zero disables only the exceed counter. Not a flight safety limit.
    // @Range: 0 1000000
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("B_WAIT_US", 19, AP_UTB, _bench_wait, 0),
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Param: B_FAIL
    // @DisplayName: Benchmark simulated thread creation failure
    // @Description: SITL only: bypasses UTB worker creation with a false result without exhausting heap. Does not alter arming guards.
    // @Values: 0:Normal,1:Simulated failure
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("B_FAIL", 20, AP_UTB, _bench_fail, 0),
#endif
#endif
    // @Param: LOG_FMT
    // @DisplayName: UTB diagnostic log format
    // @Description: Full writes separate legacy messages. Packed v1 preserves every sample field in one UTBC packet; decode with utb_log_decode.py. Changes only worker serialization, never controller rate or motor authority.
    // @Values: 0:Full,1:Packed v1
    // @User: Advanced
    AP_GROUPINFO("LOG_FMT", 21, AP_UTB, _log_format, 0),

#if AP_UTB_RATE_SITL_AVAILABLE
    // @Param: S_OWNER
    // @DisplayName: Experimental SITL rate owner
    // @Description: SITL only. Enables explicit GCS entry into mode 41. Authority is fixed at entry; changes invalidate the experiment. No hardware output capability.
    // @Values: 0:Disabled,1:UTB PID,2:AP PID comparison
    // @User: Advanced
    AP_GROUPINFO("S_OWNER", 23, AP_UTB, _sitl_owner, 0),

    // @Param: S_PROFILE
    // @DisplayName: Finite SITL reference profile
    // @Description: SITL only. Rate profiles use 3200 cycles or 8000 for long yaw. Explicit S_ATT uses 5600 cycles and supports profiles 0/1/2/4/5/6. Fixed at entry. No altitude or position control.
    // @Values: 0:Zero,1:Roll,2:Pitch,3:Yaw,4:Combined,5:Roll sine,6:Roll sign changes,7:Long yaw
    // @User: Advanced
    AP_GROUPINFO("S_PROFILE", 24, AP_UTB, _sitl_profile, 0),

    // @Param: S_RUN
    // @DisplayName: Start finite SITL reference
    // @Description: SITL only. Start after virtual arming and full spool. Completed sequence stays at zero reference; mode exit is required before another run. Does not arm or change modes.
    // @Values: 0:Waiting,1:Start
    // @User: Advanced
    AP_GROUPINFO("S_RUN", 25, AP_UTB, _sitl_run, 0),

    // @Param: S_AMP
    // @DisplayName: SITL rate amplitude
    // @Description: SITL only. Rate-only amplitude, fixed at entry. Attitude experiments use S_ANG instead. Not a flight tune or hardware safety limit.
    // @Range: 0 0.3
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("S_AMP", 26, AP_UTB, _sitl_amplitude, 0),

    // @Param: S_DLY
    // @DisplayName: SITL normalized RPY command delay
    // @Description: SITL only. One 400Hz cycle of explicit adapter transport delay for both comparison owners. Fixed at entry, reset on inactivity. Not ESC or hardware delay calibration.
    // @Values: 0:No delay,1:One cycle
    // @User: Advanced
    AP_GROUPINFO("S_DLY", 27, AP_UTB, _sitl_delay, 0),

    // @Param: S_AW
    // @DisplayName: Experimental SITL allocation feedback
    // @Description: SITL only. Uses verified BF_X_REV pre-linearization allocation for directional integration gating. Unknown, stale, boost or slew-mismatched feedback stops. Requires scripting disabled. Default retains saturation stop. No physical torque feedback.
    // @Values: 0:Saturation stop,1:Verified allocation gating
    // @User: Advanced
    AP_GROUPINFO("S_AW", 28, AP_UTB, _sitl_antiwindup, 0),

    // @Param: S_ATT
    // @DisplayName: SITL attitude source
    // @Description: SITL only. Explicit experimental outer loop, fixed at entry. Zero preserves rate-only experiments. Available only on the simulator.
    // @Values: 0:Rate only,1:UTB reduced rotation P,2:AP attitude comparison
    // @User: Advanced
    AP_GROUPINFO("S_ATT", 29, AP_UTB, _sitl_attitude, 0),

    // @Param: S_ANG
    // @DisplayName: SITL angle amplitude
    // @Description: SITL only. Finite reference amplitude. Combined roll/pitch vector must remain within five degrees. Not a hardware tune.
    // @Range: 0 5
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("S_ANG", 30, AP_UTB, _sitl_angle, 0),

    // @Param: S_ARP
    // @DisplayName: SITL roll angle P
    // @Description: SITL only. Reduced rotation roll gain, fixed at entry. Simulator only.
    // @Range: 1 6
    // @User: Advanced
    AP_GROUPINFO("S_ARP", 31, AP_UTB, _sitl_angle_roll_p, 2),

    // @Param: S_APP
    // @DisplayName: SITL pitch angle P
    // @Description: SITL only. Reduced rotation pitch gain, fixed at entry. Simulator only.
    // @Range: 1 6
    // @User: Advanced
    AP_GROUPINFO("S_APP", 32, AP_UTB, _sitl_angle_pitch_p, 2),

    // @Param: S_RMAX
    // @DisplayName: SITL tilt rate limit
    // @Description: SITL only. Common roll/pitch rate norm limit. Fixed at entry, simulator only.
    // @Range: 0.05 0.3
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("S_RMAX", 33, AP_UTB, _sitl_angle_rate_max, 0.3),

    // @Param: S_YRATE
    // @DisplayName: SITL attitude yaw rate
    // @Description: SITL only. Body yaw rate for the UTB outer loop, no heading hold. AP comparison uses its original Euler yaw rate semantics.
    // @Range: -0.1 0.1
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("S_YRATE", 34, AP_UTB, _sitl_yaw_rate, 0),

    // @Param: S_TOFF
    // @DisplayName: SITL throttle perturbation
    // @Description: SITL only. Constant normalized throttle offset for bounded attitude experiments. No altitude controller. Fixed at entry.
    // @Range: -0.001 0.001
    // @User: Advanced
    AP_GROUPINFO("S_TOFF", 35, AP_UTB, _sitl_throttle_offset, 0),

    // @Param: A_ENABLE
    // @DisplayName: SITL pilot ANGLE mode enable
    // @Description: Enable AP-derived UTB ANGLE mode42 in active SITL builds. Controller settings use UT3_; unavailable on hardware.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("A_ENABLE", 36, AP_UTB, _angle_enable, 0),

    // @Param: A_ANGLE
    // @DisplayName: SITL ANGLE maximum tilt reference
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Circular roll/pitch reference norm limit. Limited to the previously validated 5 degree attitude envelope. Changes while in ANGLE cause a latched stop.
    // @Units: deg
    // @Range: 0.1 5
    // @User: Advanced
    AP_GROUPINFO("A_ANGLE", 37, AP_UTB, _angle_max, 5),

    // @Param: A_YMAX
    // @DisplayName: SITL ANGLE maximum body yaw rate
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Maximum absolute pilot body yaw rate. Center stick commands zero rate, not heading hold. Changes while in ANGLE cause a latched stop.
    // @Units: rad/s
    // @Range: 0.01 0.1
    // @User: Advanced
    AP_GROUPINFO("A_YMAX", 38, AP_UTB, _angle_yaw_max, 0.1),

    // @Param: A_AW
    // @DisplayName: SITL ANGLE allocation feedback opt in
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Zero keeps the existing saturation-stop policy. One enables the existing validated normalized BF_X_REV allocation feedback. Changes while in ANGLE cause a latched stop.
    // @Values: 0:Saturation stop,1:Allocation feedback
    // @User: Advanced
    AP_GROUPINFO("A_AW", 39, AP_UTB, _angle_aw, 0),

    // @Param: C_ENABLE
    // @DisplayName: SITL UTB C_ENABLE
    // @Description: Enable AP-derived UTB ACRO mode43 in active SITL builds. Native AP ACRO maps and UT3_ controller settings apply; unavailable on hardware.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("C_ENABLE", 40, AP_UTB, _acro_enable, 0),

    // @Param: C_RMAX
    // @DisplayName: SITL UTB C_RMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Maximum absolute pilot roll body rate. Frozen at entry; hard cap 0.15 rad/s. Change limits outside the pilot mode. Hardware has no such parameter.
    // @Units: rad/s
    // @Range: 0.01 0.15
    // @User: Advanced
    AP_GROUPINFO("C_RMAX", 41, AP_UTB, _acro_roll_max, 0.15f),

    // @Param: C_PMAX
    // @DisplayName: SITL UTB C_PMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Maximum absolute pilot pitch body rate. Frozen at entry; hard cap 0.15 rad/s. Change limits outside the pilot mode. Hardware has no such parameter.
    // @Units: rad/s
    // @Range: 0.01 0.15
    // @User: Advanced
    AP_GROUPINFO("C_PMAX", 42, AP_UTB, _acro_pitch_max, 0.15f),

    // @Param: C_YMAX
    // @DisplayName: SITL UTB C_YMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Maximum absolute pilot yaw body rate. Frozen at entry; hard cap 0.05 rad/s. Change limits outside the pilot mode. Hardware has no such parameter.
    // @Units: rad/s
    // @Range: 0.01 0.05
    // @User: Advanced
    AP_GROUPINFO("C_YMAX", 43, AP_UTB, _acro_yaw_max, 0.05f),

    // @Param: C_AW
    // @DisplayName: SITL UTB C_AW
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Existing allocation feedback opt in for pilot ACRO. Frozen at entry. Change limits outside the pilot mode. Hardware has no such parameter.
    // @Values: 0:Saturation stop,1:Allocation feedback
    // @User: Advanced
    AP_GROUPINFO("C_AW", 44, AP_UTB, _acro_aw, 0),

    // @Param: A_RMAX
    // @DisplayName: SITL UTB A_RMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Maximum norm of ANGLE roll/pitch body rate target. Frozen at entry; existing hard cap 0.3 rad/s. Change limits outside the pilot mode. Hardware has no such parameter.
    // @Units: rad/s
    // @Range: 0.05 0.3
    // @User: Advanced
    AP_GROUPINFO("A_RMAX", 45, AP_UTB, _angle_rate_max, 0.3f),

    // @Param: P_RLL_P
    // @DisplayName: SITL pilot RLL P
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.3
    // @User: Advanced
    AP_GROUPINFO("P_RLL_P", 46, AP_UTB, _pilot_p[0], 0.095f),

    // @Param: P_RLL_I
    // @DisplayName: SITL pilot RLL I
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.6
    // @User: Advanced
    AP_GROUPINFO("P_RLL_I", 47, AP_UTB, _pilot_i[0], 0.24f),

    // @Param: P_RLL_D
    // @DisplayName: SITL pilot RLL D
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.004
    // @User: Advanced
    AP_GROUPINFO("P_RLL_D", 48, AP_UTB, _pilot_d[0], 0.00075f),

    // @Param: P_RLL_IMAX
    // @DisplayName: SITL pilot RLL IMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.19
    // @User: Advanced
    AP_GROUPINFO("P_RLL_IMAX", 49, AP_UTB, _pilot_imax[0], 0.19f),

    // @Param: P_RLL_D_HZ
    // @DisplayName: SITL pilot RLL D_HZ
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 40
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("P_RLL_D_HZ", 50, AP_UTB, _pilot_d_hz[0], 25.0f),

    // @Param: P_PIT_P
    // @DisplayName: SITL pilot PIT P
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.3
    // @User: Advanced
    AP_GROUPINFO("P_PIT_P", 51, AP_UTB, _pilot_p[1], 0.1425f),

    // @Param: P_PIT_I
    // @DisplayName: SITL pilot PIT I
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.6
    // @User: Advanced
    AP_GROUPINFO("P_PIT_I", 52, AP_UTB, _pilot_i[1], 0.24f),

    // @Param: P_PIT_D
    // @DisplayName: SITL pilot PIT D
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.004
    // @User: Advanced
    AP_GROUPINFO("P_PIT_D", 53, AP_UTB, _pilot_d[1], 0.0015f),

    // @Param: P_PIT_IMAX
    // @DisplayName: SITL pilot PIT IMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.19
    // @User: Advanced
    AP_GROUPINFO("P_PIT_IMAX", 54, AP_UTB, _pilot_imax[1], 0.19f),

    // @Param: P_PIT_D_HZ
    // @DisplayName: SITL pilot PIT D_HZ
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 40
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("P_PIT_D_HZ", 55, AP_UTB, _pilot_d_hz[1], 35.0f),

    // @Param: P_YAW_P
    // @DisplayName: SITL pilot YAW P
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.3
    // @User: Advanced
    AP_GROUPINFO("P_YAW_P", 56, AP_UTB, _pilot_p[2], 0.3f),

    // @Param: P_YAW_I
    // @DisplayName: SITL pilot YAW I
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.6
    // @User: Advanced
    AP_GROUPINFO("P_YAW_I", 57, AP_UTB, _pilot_i[2], 0.6f),

    // @Param: P_YAW_D
    // @DisplayName: SITL pilot YAW D
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.004
    // @User: Advanced
    AP_GROUPINFO("P_YAW_D", 58, AP_UTB, _pilot_d[2], 0.0f),

    // @Param: P_YAW_IMAX
    // @DisplayName: SITL pilot YAW IMAX
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 0.19
    // @User: Advanced
    AP_GROUPINFO("P_YAW_IMAX", 59, AP_UTB, _pilot_imax[2], 0.19f),

    // @Param: P_YAW_D_HZ
    // @DisplayName: SITL pilot YAW D_HZ
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Shared pilot ACRO and ANGLE rate gain; does not change mode41 or SHADOW. Frozen at entry; edit outside both pilot modes. D requires a positive cutoff.
    // @Range: 0 40
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("P_YAW_D_HZ", 60, AP_UTB, _pilot_d_hz[2], 20.0f),

    // @Param: P_ANG_R
    // @DisplayName: SITL pilot ANGLE R attitude P
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Outer attitude proportional gain for pilot ANGLE only. Frozen at entry; edit outside pilot modes. Existing angle and rate reference caps remain unchanged.
    // @Range: 1 6
    // @User: Advanced
    AP_GROUPINFO("P_ANG_R", 61, AP_UTB, _pilot_angle_r, 3.25f),

    // @Param: P_ANG_P
    // @DisplayName: SITL pilot ANGLE P attitude P
    // @Description: Legacy pre-3A pilot setting, ignored by current modes42/43; use UT3_ and native AP pilot parameters. Historical meaning: Outer attitude proportional gain for pilot ANGLE only. Frozen at entry; edit outside pilot modes. Existing angle and rate reference caps remain unchanged.
    // @Range: 1 6
    // @User: Advanced
    AP_GROUPINFO("P_ANG_P", 62, AP_UTB, _pilot_angle_p, 3.25f),

#endif
    AP_GROUPEND
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Static data declarations
//====================================================================================================================
// Public functions
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Initialize AP_Param defaults without starting a worker or enabling control.
///
///  @param[in]  None
///  @return     None (constructor/destructor).
///
///  @details    Public class member. Declaration/definition: AP_UTB::AP_UTB().
//--------------------------------------------------------------------------------------------------------------------
AP_UTB::AP_UTB()
{
    AP_Param::setup_object_defaults(this, var_info);
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Init.
///
///  @param[in]  view View.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::init(const AP_AHRS_View &view).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::init(const AP_AHRS_View &view)
{
    _enabled_at_boot = _enable.get() == 1;
    if (_enabled_at_boot) {
        _provider.init(view);
    }
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Update state.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::update_state().
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::update_state()
{
    if (_enabled_at_boot && !shadow_requested()) {
        _provider.sample(_state);
    }
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Publish.
///
///  @param[in]  value Value supplied to this operation.
///  @param[in]  enqueue Enqueue.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB::Queue::publish(const Snapshot &value, bool
///              enqueue).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB::Queue::publish(const Snapshot &value, bool enqueue)
{
    // Odhad obsazení diagnostické fronty po vložení.
    uint8_t depth;

    // The status mailbox is best-effort and independent of SPSC data publication.
    if (_sem.take_nonblocking()) {
        _status = value;
        _sem.give();
    }
    if (!enqueue) {
        return true;
    }
    if (_objects.space() == 0) {
        return false;
    }
    // Single producer: the consumer can only increase space after this check.
    ++_enqueued;
    depth = MIN(uint32_t(CAPACITY), _objects.available() + 1);
    _high_water.store(MAX(_high_water.load(), depth));
    return _objects.push(value);
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Pop.
///
///  @param[in]  value Value supplied to this operation.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB::Queue::pop(Snapshot &value).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB::Queue::pop(Snapshot &value)
{
    return _objects.pop(value);
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Try to copy the latest status mailbox without blocking.
///
///  @param[in]  value Value supplied to this operation.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB::Queue::read_status(Snapshot &value).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB::Queue::read_status(Snapshot &value)
{
    if (!_sem.take_nonblocking()) {
        return false;
    }
    value = _status;
    _sem.give();
    return true;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Reset shadow.
///
///  @param[in]  None
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::reset_shadow().
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::reset_shadow()
{
#if AP_UTB_BENCH_ENABLED
    if (synthetic_requested()) {
        _synthetic.abort();
    }
#endif
    _controller.reset();
    ++_epoch;
    _capture = {};
    _snapshot = {};
    _context_valid = false;
    _last_imu_us = 0;
    _last_gyro = 255;
    _experiment_started = false;
    _budget_latched = false;
    _consecutive_overruns = 0;
    _last_log_us = 0;
    _rate_start_us = 0;
    _rate_start_written = _written_sets.load();
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Capture the AP rate target together with its time, mode and sequence identity.
///
///  @param[in]  target Target.
///  @param[in]  time_us Sample/log timestamp in microseconds.
///  @param[in]  mode Mode.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::capture_target(const Vector3f &target, uint64_t
///              time_us, uint8_t mode).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::capture_target(const Vector3f &target, uint64_t time_us, uint8_t mode)
{
    if (!shadow_requested()) {
        return;
    }
    _capture.target = target;
    _capture.time_us = time_us;
    _capture.seq = _seq + 1;
    _capture.mode = mode;
    _capture.valid = !target.is_nan() && !target.is_inf();
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Evaluate SHADOW validity and controller/mixer mathematics from explicit sampled inputs.
///
///  @param[in]  input Explicit input snapshot.
///  @param[in]  gains Per-axis controller gains.
///  @param[out] s S.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::evaluate(const Inputs &input, const
///              AP_UTB_RateController::Gains (&gains)[3], Snapshot &s).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::evaluate(const Inputs &input, const AP_UTB_RateController::Gains (&gains)[3], Snapshot &s)
{
    // Příznak změny třídy nebo geometrie rámu.
    bool geometry_changed;
    // Počet sad odmítnutých logger backendem.
    uint32_t missing;
    // Příznak odmítnuté diagnostické sady v logger backendu.
    bool backend_failed;
    // Příznak změny primárního gyroskopu.
    bool gyro_changed;
    // Platnost nominální periody regulátoru.
    bool nominal_valid;
    // Platnost skutečného časového kroku vůči nominální periodě.
    bool dt_valid;
    // Povolené stáří stavových dat v mikrosekundách.
    uint32_t age_us;
    // Platnost stavu včetně stáří a nové INS časové značky.
    bool state_valid;

    s = {};
    s.input = input;
    s.log_format = _log_format.get() == 1 ? 1 : 0;
    s.seq = ++_seq;
    geometry_changed = _context_valid &&
                       (input.frame_class != _last_class || input.frame_type != _last_type);
    if (geometry_changed) {
        // Old queued snapshots retain their epoch and cannot be attributed to the new geometry.
        ++_epoch;
    }
    s.epoch = _epoch.load();
    s.flags = ENABLED | SHADOW_REQUESTED;
    if (input.logging_available) {
        s.flags |= LOGGING_AVAILABLE | SHADOW_OBSERVABLE;
    } else {
        s.observation = ObservationReason::LOGGING_UNAVAILABLE;
    }
    missing = _missing.load();
    backend_failed = _backend_failed.load() || missing != _last_observed_missing;
    _last_observed_missing = missing;
    if (backend_failed) {
        s.flags &= ~SHADOW_OBSERVABLE;
        s.observation = ObservationReason::WRITE_FAILED;
    }
    gyro_changed = _context_valid && input.state.primary_gyro != _last_gyro;
    if (gyro_changed || !_context_valid || input.mode != _last_mode || input.armed != _last_armed ||
        input.frame_class != _last_class || input.frame_type != _last_type) {
        _controller.reset();
    }
    _context_valid = true;
    _last_mode = input.mode;
    _last_armed = input.armed;
    _last_class = input.frame_class;
    _last_type = input.frame_type;
    _last_gyro = input.state.primary_gyro;
    nominal_valid = isfinite(input.nominal_dt) && input.nominal_dt > 0;
    dt_valid = nominal_valid && isfinite(input.dt) && input.dt >= input.nominal_dt * 0.25f &&
               input.dt <= input.nominal_dt * 4;
    age_us = nominal_valid ? uint32_t(MIN(0.1f, input.nominal_dt * 4) * 1.0e6f) : 0;
    state_valid = nominal_valid && input.state.valid(input.validation_time_us, age_us) &&
                  input.state.imu_updated_us != _last_imu_us;
    _last_imu_us = input.state.imu_updated_us;
    if (state_valid) {
        s.flags |= STATE_VALID;
    }
    if (_budget_latched) {
        s.policy = PolicyReason::TIME_BUDGET;
        _controller.reset();
        return;
    }
    if (!_experiment_started && !input.logging_available) {
        s.policy = PolicyReason::EXPERIMENT_SUPPRESSED;
        _controller.reset();
        return;
    }
    _experiment_started = true;
    if (input.fast_rate) {
        s.calculation = CalculationReason::FAST_RATE_UNSUPPORTED;
    } else if (!AP_UTB_MotorMixer::supports_frame(input.frame_class, input.frame_type)) {
        s.calculation = CalculationReason::FRAME_MISMATCH;
    } else if (!input.acro_available) {
        s.calculation = CalculationReason::ACRO_UNAVAILABLE;
    } else if (!dt_valid) {
        s.calculation = CalculationReason::DT_INVALID;
    } else if (!state_valid) {
        s.calculation = CalculationReason::STATE_INVALID;
    } else if (!input.reference_valid || !isfinite(input.thrust) || input.thrust < 0 || input.thrust > 1) {
        s.calculation = CalculationReason::REFERENCE_INVALID;
    } else if (!_controller.configure(gains, input.nominal_dt)) {
        s.calculation = CalculationReason::CONFIG_INVALID;
    }
    if (s.calculation != CalculationReason::NONE) {
        _controller.reset();
        return;
    }
    s.flags |= CONTROLLER_EVALUATED;
    s.control = _controller.update(input.state, input.reference, input.dt);
    if (!s.control.valid) {
        s.calculation = s.control.priming ? (gyro_changed ? CalculationReason::PRIMARY_GYRO_CHANGED : CalculationReason::PRIMING) : CalculationReason::CONTROLLER_INVALID;
        return;
    }
    s.flags |= CONTROLLER_VALID | MIXER_EVALUATED;
    s.mixer = _mixer.mix(s.control, input.thrust, input.frame_class, input.frame_type);
    if (!s.mixer.valid) {
        s.calculation = CalculationReason::MIXER_INVALID;
        _controller.reset();
        return;
    }
    s.flags |= MIXER_VALID | SHADOW_HEALTHY;
    _controller.feedback(s.control, s.mixer.achieved);
    if (input.logging_available && !backend_failed) {
        s.flags |= SHADOW_OBSERVABLE;
    }
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Update.
///
///  @param[in]  input Explicit input snapshot.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::update(const Inputs &input).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::update(const Inputs &input)
{
    // Počáteční čas nebo index vyhodnocovaného intervalu.
    uint32_t start;
    // Požadovaná frekvence ukládání diagnostiky v Hz.
    int16_t rate;
    // Doba od počátku vyhodnocovaného intervalu.
    uint64_t elapsed;
    // Určuje, zda se tento skutečný vzorek předá k zalogování.
    bool selected;
    // Index průchodu pevnou kolekcí.
    uint8_t j;

    _logging_requested.store(shadow_requested());
    if (!shadow_requested()) {
#if AP_UTB_BENCH_ENABLED
        if (!synthetic_requested()) {
            _synthetic.update(0, false, false, false, input.mode);
            _last_synthetic_request = 0;
        }
#endif
        if (_was_requested) {
            reset_shadow();
        }
        _was_requested = false;
        _snapshot.input.mode = input.mode;
        _snapshot.input.time_us = input.time_us;
        _queue.publish(_snapshot, false);
        return;
    }
    _was_requested = true;
    start = input.started_us;
    // Pole vzniká až po SHADOW guardu, vypnutá cesta je nesestavuje.
    AP_UTB_RateController::Gains gains[3];
    for (j = 0; j < 3; j++) {
        gains[j] = {_p[j].get(), _i[j].get(), _d[j].get(), _imax[j].get(), _d_hz[j].get()};
    }
    evaluate(input, gains, _snapshot);
#if AP_UTB_BENCH_ENABLED
    if ((input.synthetic_status == uint8_t(AP_UTB_SyntheticReference::Status::RUNNING) ||
         input.synthetic_status == uint8_t(AP_UTB_SyntheticReference::Status::PREROLL)) &&
        (_snapshot.policy != PolicyReason::NONE ||
         (_snapshot.calculation != CalculationReason::NONE && _snapshot.calculation != CalculationReason::PRIMING))) {
        _synthetic.abort();
        _snapshot.input.synthetic_status = uint8_t(AP_UTB_SyntheticReference::Status::ABORTED);
    }
#endif
    _state = input.state;
    if (!(_snapshot.flags & STATE_VALID)) {
        _snapshot.input.state.rates_rads.zero();
    }
    if (!input.reference_valid) {
        _snapshot.input.reference.zero();
    }
    if (!_snapshot.input.ap_target.valid) {
        _snapshot.input.ap_target.target.zero();
    }
    if (!isfinite(_snapshot.input.dt)) {
        _snapshot.input.dt = 0;
    }
    if (!isfinite(_snapshot.input.thrust)) {
        _snapshot.input.thrust = 0;
    }
    rate = _log_rate.get();
    if (rate < 1 || rate > 400 || (_shadow.get() != 0 && _shadow.get() != 1)) {
        _snapshot.control = {};
        _snapshot.mixer = {};
        _snapshot.flags &= ~(CONTROLLER_VALID | MIXER_VALID | SHADOW_HEALTHY);
        _snapshot.calculation = CalculationReason::CONFIG_INVALID;
        _controller.reset();
    }
    _snapshot.requested_log_rate = rate >= 1 && rate <= 400 ? rate : 0;
    if (_rate_start_us == 0) {
        _rate_start_us = input.time_us;
    }
    elapsed = input.time_us - _rate_start_us;
    _snapshot.effective_log_rate = elapsed > 0 ? (_written_sets.load() - _rate_start_written) * 1.0e6f / elapsed : 0;
    _snapshot.drops = _drops;
    _snapshot.missing = _missing;
    selected = select_log_sample(input.time_us, input.nominal_dt, rate);
    if (_snapshot.calculation == CalculationReason::PRIMARY_GYRO_CHANGED) {
        selected = true; // Preserve the one-cycle reset event when diagnostic capacity permits.
    }
    // Commit all metadata before publication; the consumer only sees complete copies.
    _snapshot.execution_us = AP_HAL::micros() - start;
    record_execution(_snapshot.execution_us, input.nominal_dt);
#if AP_UTB_BENCH_ENABLED
    if (_budget_latched && input.reference_source != 0) {
        _synthetic.abort();
        _controller.reset();
        _snapshot.control = {};
        _snapshot.mixer = {};
        _snapshot.flags &= ~(CONTROLLER_VALID | MIXER_VALID | SHADOW_HEALTHY);
        _snapshot.policy = PolicyReason::TIME_BUDGET;
        _snapshot.input.synthetic_status = uint8_t(AP_UTB_SyntheticReference::Status::ABORTED);
    }
#endif
    _snapshot.max_execution_us = _max_execution_us;
    _snapshot.overruns = _overruns;
    _snapshot.drops = _drops.load();
    _snapshot.missing = _missing.load();
    publish_diagnostics(_snapshot, selected);

}

#if AP_UTB_BENCH_ENABLED
//--------------------------------------------------------------------------------------------------------------------
///  @brief      Gate and advance the DISARMED internal diagnostic reference.
///
///  @param[in]  in Explicit input snapshot.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::prepare_synthetic(Inputs &in).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::prepare_synthetic(Inputs &in)
{
    // Splnění požadavků na periodu diagnostického experimentu.
    bool timing;
    // Splnění všech podmínek pro interní SHADOW referenci.
    bool eligible;

    if (_synthetic_request.get() != _last_synthetic_request) {
        _controller.reset();
        ++_epoch;
        _last_synthetic_request = _synthetic_request.get();
    }
    if (!synthetic_requested()) {
        _synthetic.update(0, false, false, false, in.mode);
        return;
    }
    timing = isfinite(in.nominal_dt) && in.nominal_dt <= 0.0025f && in.nominal_dt >= 0.0025f && isfinite(in.dt) &&
             in.dt >= in.nominal_dt * 0.25f && in.dt <= in.nominal_dt * 4;
    eligible = enabled() && shadow_requested() && timing && !in.fast_rate && in.acro_available && in.bench_safe &&
               AP_UTB_MotorMixer::supports_frame(in.frame_class, in.frame_type) &&
               in.state.valid(in.validation_time_us, 10000) &&
               in.state.imu_updated_us != _last_imu_us && in.logging_available && !_budget_latched;
    // Vzorek vzniká až po kontrolách; stavový automat se volá právě jednou.
    const AP_UTB_SyntheticReference::Sample sample = _synthetic.update(uint8_t(_synthetic_request.get()), eligible, in.armed, in.watchdog, in.mode);
    in.reference_source = synthetic_requested() ? 1 : 0;
    in.synthetic_profile = _synthetic_request.get();
    in.synthetic_cycle = sample.cycle;
    in.synthetic_phase = sample.phase;
    in.synthetic_status = uint8_t(sample.status);
    in.reference = sample.rates;
    in.reference_valid = sample.valid;
    in.thrust = sample.valid ? 0.5f : 0;
}
#endif

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Select a real controller sample at the requested logging cadence without backfilling.
///
///  @param[in]  time_us Sample/log timestamp in microseconds.
///  @param[in]  nominal_dt Nominal controller interval in seconds.
///  @param[in]  rate Rate.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB::select_log_sample(uint64_t time_us, float
///              nominal_dt, int16_t rate).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB::select_log_sample(uint64_t time_us, float nominal_dt, int16_t rate)
{
    // Rozestup ukládaných vzorků v mikrosekundách.
    uint64_t period_us;

    if (rate != _last_log_rate) {
        _last_log_us = 0;
        _last_log_rate = rate;
    }
    if (rate < 1 || rate > 400) {
        return false;
    }
    // At the nominal controller rate, capture every cycle despite timer jitter.
    // Slower log rates retain the existing wall-time decimation.
    if (isfinite(nominal_dt) && nominal_dt > 0 && rate * nominal_dt >= 1.0f) {
        _last_log_us = 0;
        return true;
    }
    if (_last_log_us == 0 || time_us >= _last_log_us) {
        period_us = uint64_t(1.0e6f / rate);
        _last_log_us = (_last_log_us == 0 ? time_us : _last_log_us) + period_us;
        if (_last_log_us <= time_us) {
            _last_log_us = time_us + period_us;
        }
        return true;
    }
    return false;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Publish fixed diagnostic data and update queue/drop accounting.
///
///  @param[in]  snapshot Snapshot.
///  @param[in]  selected Selected.
///  @return     bool.
///
///  @details    Public class member. Declaration/definition: bool AP_UTB::publish_diagnostics(Snapshot &snapshot, bool
///              selected).
//--------------------------------------------------------------------------------------------------------------------
bool AP_UTB::publish_diagnostics(Snapshot &snapshot, bool selected)
{
    if (selected) {
        ++_requested_sets;
    }
    if (!_queue.publish(snapshot, selected)) {
        if (selected) {
            ++_drops;
            snapshot.flags &= ~SHADOW_OBSERVABLE;
            snapshot.observation = ObservationReason::QUEUE_FULL;
            snapshot.drops = _drops.load();
            // One best-effort status publication, never retry the dropped data set.
            _queue.publish(snapshot, false);
        }
        return false;
    }
    return true;
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Record producer wall time and apply the existing time-budget policy.
///
///  @param[in]  elapsed_us Elapsed us.
///  @param[in]  nominal_dt Nominal controller interval in seconds.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::record_execution(uint32_t elapsed_us, float
///              nominal_dt).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::record_execution(uint32_t elapsed_us, float nominal_dt)
{
    _max_execution_us = MAX(_max_execution_us, elapsed_us);
    if (isfinite(nominal_dt) && nominal_dt > 0 && elapsed_us > nominal_dt * 5.0e4f) {
        _overruns++;
        if (_consecutive_overruns < 3 && ++_consecutive_overruns >= 3) {
            _budget_latched = true;
        }
    } else {
        _consecutive_overruns = 0;
    }
}

//--------------------------------------------------------------------------------------------------------------------
///  @brief      Record successful complete-set admission or backend failure.
///
///  @param[in]  success Success.
///  @return     None.
///
///  @details    Public class member. Declaration/definition: void AP_UTB::log_result(bool success).
//--------------------------------------------------------------------------------------------------------------------
void AP_UTB::log_result(bool success)
{
    _backend_failed.store(!success);
    if (success) {
        ++_written_sets;
    } else {
        ++_missing;
    }
}
#endif // AP_UTB_ENABLED

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Public functions
//====================================================================================================================
// Data type, constant, macro definition
//--------------------------------------------------------------------------------------------------------------------
// None
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End Data type, constant, macro definition
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
