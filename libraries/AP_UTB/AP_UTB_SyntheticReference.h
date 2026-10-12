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
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_ENABLED && AP_UTB_BENCH_ENABLED
#include <AP_Math/AP_Math.h>

// Reference only. No sensor substitution, actuator or logger interfaces.
class AP_UTB_SyntheticReference
{
public:
    enum class Status : uint8_t { OFF, WAITING, RUNNING, COMPLETE, ABORTED, PREROLL };
    struct Sample {
        Vector3f rates;
        uint32_t cycle = 0;
        uint8_t phase = 0;
        Status status = Status::OFF;
        bool valid = false;
    };
    Sample update(uint8_t request, bool eligible, bool armed, bool watchdog, uint8_t mode);
    void abort();
    static Vector3f signal(uint32_t cycle, uint8_t profile);
    static uint8_t phase(uint32_t cycle);
    static constexpr uint32_t CYCLES = 8000;
private:
    uint32_t _cycle = 0;
    uint16_t _preroll = 0;
    uint8_t _mode = 0;
    uint8_t _profile = 0;
    Status _status = Status::OFF;
};
#endif
