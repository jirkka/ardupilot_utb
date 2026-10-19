#pragma once
// GNU GPL v3 or later; original controller algorithms: ArduPilot contributors.
/// @file AP_UTB_NativeBench.h
/// @brief Separate DISARMED numerical experiment. Never owns the vehicle motors.
/// @author Jiri Kovar
/// @version 2026-10-19
#include "AP_UTB_config.h"
#if AP_UTB_NATIVE_BENCH_AVAILABLE
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
class AP_AHRS_View;
class UTB_NativeBenchTestAccess;
class AP_UTB_NativeBench
{
public:
    AP_UTB_NativeBench();
    void init(AP_AHRS_View &view);
    void update(bool armed);
    static const AP_Param::GroupInfo var_info[];
private:
    friend class UTB_NativeBenchTestAccess;
    struct Session;
    // Only init() can bind the independent controller pair to RAM-only sinks.
    Session *_session = nullptr;
    AP_Int8 _run;
    bool _running = false;
    uint32_t _seq = 0;
    uint32_t _last_us = 0;
    uint32_t _last_log_us = 0;
    bool compute();
    void emit();
    float max_difference() const;
};
#endif
