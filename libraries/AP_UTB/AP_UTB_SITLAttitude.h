/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "AP_UTB_State.h"

// Reduced rotation P controller. No heading integrator or actuator dependencies.
class AP_UTB_SITLAttitude
{
public:
    struct Config {
        Config(float roll = 2, float pitch = 2, float maximum = 0.3f) :
            roll_p(roll), pitch_p(pitch), rate_max(maximum) {}
        float roll_p;
        float pitch_p;
        float rate_max;
        bool operator==(const Config &other) const;
    };
    enum class Fault : uint8_t { NONE, CONFIG, STATE, DT, REFERENCE, ENVELOPE, IMU };
    struct Result {
        Vector3f euler;
        Vector3f error;
        Vector3f rates;
        bool limited = false;
        bool valid = false;
    };
    bool configure(const Config &config);
    bool configuration_matches(const Config &config) const
    {
        return _configured && config == _config;
    }
    Result update(const AP_UTB_State &state, const Vector3f &reference, float dt, uint32_t now_us);
    void reset();
    Fault fault() const
    {
        return _fault;
    }
    uint32_t calls() const
    {
        return _calls;
    }
    static Vector3f reference(uint8_t profile, uint32_t cycle, float angle_rad, float yaw_rate);
    static constexpr uint32_t DURATION_CYCLES = 5600;
private:
    void invalidate(Fault fault);
    Config _config;
    Fault _fault = Fault::NONE;
    uint32_t _last_imu = 0;
    uint32_t _calls = 0;
    uint8_t _gyro = 255;
    bool _configured = false;
    bool _context = false;
};
#endif // AP_UTB_RATE_SITL_AVAILABLE
