// GNU GPL v3 or later; original ArduPilot algorithms retain their copyright.
/// @file AC_AttitudeControl_Shadow.h
/// @brief Value-only attitude diagnostic interface; absent when disabled.
/// @author Jiri Kovar
/// @version 2026-10-23
#pragma once
#ifndef AP_UTB_ATTITUDE_SHADOW_ENABLED
#define AP_UTB_ATTITUDE_SHADOW_ENABLED 0
#endif
#if AP_UTB_ATTITUDE_SHADOW_ENABLED && AP_UTB_ENABLED
#include <AP_Math/AP_Math.h>
namespace UTBAttitudeCapture
{
enum class Command : uint8_t { STABILIZE, ACRO, RATE_ONLY, RESET, RESET_YAW, RELAX, INERTIAL };
struct History {
    Quaternion target, error;
    Vector3f euler, euler_rate, velocity, acceleration, body;
    float thrust = 0, thrust_error = 0, feedforward = 1;
};
struct Config {
    Vector3f angle_p, accel, rate_max, angle_scale;
    float dt = 0, input_tc = 0, rp_tc = 0, yaw_tc = 0, yaw_p = 0, trim = 0;
    bool feedforward = false, sqrt_controller = false;
};
struct Sensors {
    Quaternion attitude;
    Vector3f gyro;
    float yaw = 0;
};
struct Frame {
    Config config;
    Sensors sensors;
    History before, after;
    Vector3f command;
    Command kind = Command::STABILIZE;
    bool reset_rate = false;
    uint32_t seq = 0, time_us = 0, loop = 0;
    uint8_t mode = 0;
};
// Hooks accept values only. No native controller or motor pointer crosses this boundary.
bool enabled();
void before(Command kind, const Vector3f &command, bool reset_rate,
            const Config &config, const Sensors &sensors, const History &history);
void after(const History &history);
void audit(bool changed);
void capture_time(bool post, uint32_t elapsed_us);
}
#endif
