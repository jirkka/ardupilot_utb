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
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "AP_UTB_RateController.h"
#include "AP_UTB_SITLAllocation.h"

// Experimental unsaturated adapter policy; has no motor or logger dependencies.
class AP_UTB_SITLRate
{
public:
    enum class Fault : uint8_t { NONE, STATE, DT, FRAME, FAST_RATE, CONFIG, SATURATION, ENVELOPE, IMU, FEEDBACK, ATTITUDE, VERTICAL };
    struct Inputs {
        AP_UTB_State state;
        Vector3f target;
        uint32_t now_us = 0;
        uint32_t seq = 0;
        float dt = 0;
        float nominal_dt = 0;
        uint8_t frame_class = 0;
        uint8_t frame_type = 0;
        bool fast_rate = false;
        bool armed = false;
        bool full_spool = false;
        bool allocation_limited = false;
    };
    bool configure(const AP_UTB_RateController::Gains (&gains)[3], bool allocation_feedback = false, bool delayed = false);
    void capture_allocation(const AP_UTB_SITLAllocation::Result &feedback)
    {
        _feedback = feedback;
    }
    bool configuration_matches(const AP_UTB_RateController::Gains (&gains)[3]) const;
    AP_UTB_RateController::Result step(const Inputs &in, bool utb_owner);
    void reset();
    void invalidate(Fault reason);
    Fault fault() const
    {
        return _fault;
    }
    uint32_t calls() const
    {
        return _calls;
    }
    Vector3f transport(const Vector3f &command, bool delayed, bool enabled);
    static Vector3f reference(uint8_t profile, uint32_t cycle, float amplitude);
    static constexpr uint32_t DURATION_CYCLES = 3200;
    static uint32_t duration_cycles(uint8_t profile)
    {
        return profile == 7 ? 8000 : DURATION_CYCLES;
    }
private:
    AP_UTB_RateController _controller;
    AP_UTB_RateController::Gains _gains[3];
    Fault _fault = Fault::NONE;
    uint32_t _calls = 0;
    uint32_t _last_imu = 0;
    uint8_t _gyro = 255;
    bool _configured = false;
    bool _context = false;
    bool _armed = false;
    bool _full_spool = false;
    Vector3f _previous_command;
    AP_UTB_SITLAllocation::Result _feedback;
    bool _allocation_feedback = false;
    bool _delayed = false;
};
#endif // AP_UTB_RATE_SITL_AVAILABLE
