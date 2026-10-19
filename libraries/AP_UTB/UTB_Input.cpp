/// @file UTB_Input.cpp
/// @author Jiri Kovar; AP algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL version 3 or later. Derived from Copter 4.7.1, b25e828b; see COPYING.txt.
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "UTB_Input.h"
// AP_Math/control.cpp: input_expo; units and rational map are unchanged.
float UTB_Input::expo(float input, float value)
{
    input = constrain_float(input, -1.0, 1.0);
    if (value < .95f) {
        return (1 - value) * input / (1 - value * fabsf(input));
    }
    return input;
}
// mode_acro.cpp: circular RP limit and static AP ACRO maps; trainer intentionally omitted.
Vector3f UTB_Input::acro(Vector3f sticks, float rp_rate_deg, float yaw_rate_deg, float rp_expo, float yaw_expo)
{
    float length; // Combined roll/pitch stick length before the original circular limit.
    float ratio;  // Common scale preserves the roll/pitch input direction.
    length = norm(sticks.x, sticks.y);
    if (length > 1) {
        ratio = 1.0f / length;
        sticks.x *= ratio;
        sticks.y *= ratio;
    }
    return {radians(rp_rate_deg) * expo(sticks.x, rp_expo),
            radians(rp_rate_deg) * expo(sticks.y, rp_expo),
            radians(yaw_rate_deg) * expo(sticks.z, yaw_expo)};
}
// AP_Math/control.cpp: rc_input_to_roll_pitch_rad, including thrust-vector limiting.
Vector2f UTB_Input::angle(float roll, float pitch, float maximum_rad)
{
    Vector2f thrust;
    Vector2f result;
    maximum_rad = MIN(maximum_rad, radians(85.0));
    thrust.x = -tanf(maximum_rad * pitch);
    thrust.y = tanf(maximum_rad * roll);
    thrust.limit_length(tanf(constrain_float(maximum_rad, radians(10.0), maximum_rad)));
    result.y = -atanf(thrust.x);
    result.x = atanf(cosf(result.y) * thrust.y);
    return result;
}
// Mode::get_pilot_desired_throttle: midpoint correction and native hover-derived cubic map.
float UTB_Input::throttle(int16_t control, int16_t midpoint, float hover)
{
    float input;
    float shaping;
    if (midpoint <= 0) {
        midpoint = 500;
    }
    control = constrain_int16(control, 0, 1000);
    if (control < midpoint) {
        input = float(control) * .5f / float(midpoint);
    } else {
        input = .5f + float(control - midpoint) * .5f / float(1000 - midpoint);
    }
    shaping = constrain_float(-(hover - .5f) / .375f, -.5f, 1.0f);
    return input * (1 - shaping) + shaping * input * input * input;
}

#endif // AP_UTB_RATE_SITL_AVAILABLE
