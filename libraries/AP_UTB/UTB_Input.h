
#pragma once
/// @file UTB_Input.h
/// @author Jiri Kovar; AP algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL version 3 or later. Derived from Copter 4.7.1, b25e828b; see COPYING.txt.
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include <AP_Math/AP_Math.h>
// Public functions: inputs are already calibrated/deadzoned/reversed by AP RC_Channel.
class UTB_Input
{
public:
    static float expo(float input, float value);
    static Vector3f acro(Vector3f sticks, float rp_rate_deg, float yaw_rate_deg, float rp_expo, float yaw_expo);
    static Vector2f angle(float roll, float pitch, float maximum_rad);
    static float throttle(int16_t control, int16_t midpoint, float hover);
};

#endif // AP_UTB_RATE_SITL_AVAILABLE
