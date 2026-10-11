/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once

#include "AP_UTB.h"
#if AP_UTB_ENABLED

namespace UTBLogPacket
{

static constexpr uint8_t VERSION = 1;
static constexpr uint16_t WORDS = 96;

struct PACKED Axis {
    float desired;
    float measured;
    float error;
    float p;
    float i;
    float d;
    float raw;
    float output;
    float ap_target;
    uint8_t valid;
    uint8_t saturation;
};

// Versioned wire payload, transported losslessly by three logger int16[32] arrays.
struct PACKED Payload {
    Axis axis[3];
    float command[3];
    float collective;
    float motor[4];
    float scale;
    float shift;
    uint8_t mixer_valid;
    uint8_t lower;
    uint8_t upper;
    float achieved[3];
    float achieved_collective;
    uint8_t positive;
    uint8_t negative;
    uint8_t scaled;
    uint8_t shifted;
    float dt;
    uint32_t execution_us;
    uint16_t requested;
    uint32_t max_execution_us;
    uint8_t version;
};

static_assert(sizeof(float) == 4 && sizeof(Axis) == 38 && sizeof(Payload) == 192,
              "UTBC v1 wire layout changed");
void encode(const AP_UTB::Snapshot &snapshot, void *words);

}
#endif // AP_UTB_ENABLED
