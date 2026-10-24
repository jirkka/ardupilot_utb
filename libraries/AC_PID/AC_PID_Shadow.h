// GNU GPL v3 or later; AP-derived algorithms retain their original copyright.
/// @file AC_PID_Shadow.h
/// @brief Guarded value-only PID diagnostic state; no mutable AP interface.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-21
/// @date 2026-10-21
#pragma once
#ifndef AP_UTB_NATIVE_SHADOW_ENABLED
#define AP_UTB_NATIVE_SHADOW_ENABLED 0
#endif
#if AP_UTB_NATIVE_SHADOW_ENABLED
#include "AP_PIDInfo.h"
// Value-only projection: all state affecting PID output with SMAX=0 and no notch.
// Disabled slew telemetry has its own wall-clock history and is deliberately excluded.
struct AC_PID_Shadow {
    enum ConfigIndex { P, I, D, FF, IMAX, PDMAX, FLTT, FLTE, FLTD, SMAX, DFF, SRT, CONFIG_COUNT };
    float config[CONFIG_COUNT] {};
    float integrator = 0, target = 0, error = 0, derivative = 0, target_derivative = 0;
    AP_PIDInfo info {};
    int8_t slew_scale = 0;
    bool reset_filter = false, i_set = false, notch = false;
};
#endif
