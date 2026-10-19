/// @file UTB_Core.cpp
/// @brief Copter-only serialization of the common controller's existing diagnostics.
/// @author Jiri Kovar
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL version 3 or later.
#include "../../../ArduCopter/Copter.h"
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
#include "utb_sitl_log.h"
#include "utb_angle_log.h"
// Existing diagnostic IDs; Own=3 distinguishes the AP-derived generation from legacy mode41.
void UTB_Core::log_cycle()
{
    ++_motor_calls;
#if HAL_LOGGING_ENABLED
    float actuator;
    float outputs[3] = {_motors.get_roll(), _motors.get_pitch(), _motors.get_yaw()};
    Vector3f attitude;
    Quaternion quaternion;
    AP_PIDInfo info;
    Vector3f target = rate.rate_bf_targets();
    log_UAMT motor_row = {LOG_PACKET_HEADER_INIT(LOG_UAMT_MSG), _time_us, _seq,
                          uint8_t(_healthy ? (_motors.armed() ? 1 : 0) : 3), uint8_t(_healthy ? 0 : 255), {}, {},
                          0, _seq, _motor_calls, 0
                         };
    for (uint8_t axis = 0; axis < 3; axis++) {
        info = axis == 0 ? rate.get_rate_roll_pid().get_pid_info() :
               axis == 1 ? rate.get_rate_pitch_pid().get_pid_info() : rate.get_rate_yaw_pid().get_pid_info();
        const log_UARX row = {LOG_PACKET_HEADER_INIT(LOG_UARX_MSG), _time_us, _seq,
                              axis, 3, uint8_t(_healthy ? 0 : 255), uint8_t(_motors.armed()), uint8_t(_motors.get_spool_state()),
                              _dt, target[axis], target[axis], info.actual, info.P, info.I, info.D, outputs[axis]
                             };
        AP::logger().WriteBlock_first_succeed(&row, sizeof(row));
    }
    for (uint8_t motor = 0; motor < 4; motor++) {
        motor_row.pwm[motor] = AP_HAL::get_HAL().rcout->read(motor);
        actuator = 0;
        _motors.get_raw_motor_throttle(motor, actuator);
        motor_row.actuator[motor] = actuator;
    }
    AP::logger().WriteBlock_first_succeed(&motor_row, sizeof(motor_row));
    state.get_quat_body_to_ned(quaternion);
    quaternion.to_euler(attitude);
    const log_UAAT angle = {LOG_PACKET_HEADER_INIT(LOG_UAAT_MSG), _time_us, _seq,
                            uint8_t(_mode == 42), 0, _mode == 42 ? reference.x : 0, _mode == 42 ? reference.y : 0,
                            attitude.x, attitude.y, attitude.z, 0, 0, target.x, target.y, target.z, 0
                           };
    const log_UARC rc = {LOG_PACKET_HEADER_INIT(LOG_UARC_MSG), _time_us, _seq, rc_ms,
                         sticks.x, sticks.y, sticks.z, manual_throttle, manual_throttle, uint8_t(rc_valid),
                         uint8_t(rc_valid ? 0 : 1), uint8_t(_healthy ? 0 : 255), _mode,
                         uint32_t(AP_HAL::millis() - rc_ms), rate.get_throttle_mix(), 0, 0
                        };
    AP::logger().WriteBlock_first_succeed(&angle, sizeof(angle));
    AP::logger().WriteBlock_first_succeed(&rc, sizeof(rc));
#endif
}

#endif
