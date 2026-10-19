
#pragma once
/// @file UTB_Core.h
/// @author Jiri Kovar; AP algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL version 3 or later. Derived from Copter 4.7.1, b25e828b; see COPYING.txt.
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include "UTB_State.h"
#include "UTB_Rate.h"
#include <AC_AttitudeControl/AC_AttitudeControl.h>
// One state source and one common Rate/PID instance for both pilot modes.
class UTB_Core
{
public:
    UTB_Core(const AP_AHRS_View &view, AP_MotorsMulticopter &motors) : state(view), rate(state, motors), _motors(motors) {}
    bool enter(uint8_t mode, AC_AttitudeControl &native);
    void leave(uint8_t mode);
    bool update_rate(float dt);
    void prepare_native(AC_AttitudeControl &native);
    void log_cycle();
    void update_mix(bool armed, bool landed, bool throttle_zero, bool air_mode);
    Vector3f sticks;
    Vector3f reference;
    float manual_throttle = 0;
    uint32_t rc_ms = 0;
    bool rc_valid = false;
    bool healthy() const
    {
        return _healthy;
    }
    UTB_State state;
    UTB_Rate rate;
private:
    // Current pilot owner; zero means the normal AP controller owns the rate slot.
    AP_MotorsMulticopter &_motors;
    uint32_t _seq = 0, _motor_calls = 0;
    uint64_t _time_us = 0;
    float _dt = 0;
    uint8_t _mode = 0;
    bool _healthy = true;
};

#endif // AP_UTB_RATE_SITL_AVAILABLE
