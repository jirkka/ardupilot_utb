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
#include "AP_UTB_State.h"
#if AP_UTB_ENABLED
class AP_UTB_RateController
{
public:
    struct Gains {
        Gains() = default;
        Gains(float p_gain, float i_gain, float d_gain, float i_max, float cutoff_hz) :
            p(p_gain), i(i_gain), d(d_gain), imax(i_max), d_hz(cutoff_hz) {}

        float p = 0;
        float i = 0;
        float d = 0;
        float imax = 0;
        float d_hz = 0;
        bool operator==(const Gains &other) const;
        bool valid(float nominal_dt) const;
    };
    struct AxisResult {
        float error = 0;
        float p = 0;
        float i = 0;
        float d = 0;
        float raw = 0;
        float output = 0;
        float acceleration = 0;
        bool integration_blocked = false;
    };
    struct Result {
        AxisResult axis[3];
        Vector3f command;
        bool valid = false;
        bool priming = false;
    };
    bool configure(const Gains (&gains)[3], float nominal_dt);
    void reset();
    Result update(const AP_UTB_State &state, const Vector3f &target_rads, float dt_s);
    // SHADOW ONLY: feedback from the previous completed allocation, never AP limits.
    void feedback(const Result &result, const Vector3f &achieved);
private:
    Gains _gains[3];
    float _nominal_dt = 0;
    float _previous[3] {};
    float _acceleration[3] {};
    float _integral[3] {};
    float _residual[3] {};
    bool _configured = false;
    bool _primed = false;
};
#endif // AP_UTB_ENABLED
