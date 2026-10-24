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
#if HAL_LOGGING_ENABLED
// Keep dynamic metadata inside the native DataFlash FMT bounds.
static constexpr char chain_labels[] = "TimeUS,Seq,Src,APN,Wr,Bad,Md,Lim,TarR,TarP,TarY";
static constexpr char chain_format[] = "QIIIBIBBfff";
static_assert(sizeof(chain_labels) <= sizeof(log_Format::labels), "Chain labels exceed FMT capacity");
static_assert(sizeof(chain_format) <= sizeof(log_Format::format), "Chain format exceeds FMT capacity");
#endif
// Called immediately after the own Attitude calculation, later than the motor-output slot.
void UTB_Core::note_attitude()
{
    _attitude_seq = _seq;
#if HAL_LOGGING_ENABLED
    const Vector3f target = rate.rate_bf_targets();
    AP::logger().WriteStreaming("UGA1", "TimeUS,Seq,Md,R,P,Y,Acc,Vel", "QIBfffff",
                                AP_HAL::micros64(), _seq, _mode,
                                double(target.x), double(target.y), double(target.z),
                                double(rate.get_attitude_target_ang_accel().length()),
                                double(rate.get_attitude_target_ang_vel().length()));
#endif
}
// Existing diagnostic IDs; Own=3 distinguishes the AP-derived generation from legacy mode41.
void UTB_Core::log_cycle()
{
    ++_motor_calls;
    // The native motor-output slot has completed; verify that own requests survived unchanged.
    const Vector3f actual{_motors.get_roll(), _motors.get_pitch(), _motors.get_yaw()};
    const Vector3f ff{_motors.get_roll_ff(), _motors.get_pitch_ff(), _motors.get_yaw_ff()};
    if (memcmp(&actual, &rate.output_control(), sizeof(actual)) != 0 ||
        memcmp(&ff, &rate.output_feedforward(), sizeof(ff)) != 0) {
        _audit_faults |= 2U;
    }
    if (rate.output_writes() != 6) {
        _audit_faults |= 4U;
    }
#if HAL_LOGGING_ENABLED
    float actuator;
    float outputs[3] = {_motors.get_roll(), _motors.get_pitch(), _motors.get_yaw()};
    Vector3f attitude;
    Quaternion quaternion;
    AP_PIDInfo info;
    Vector3f target = _consumed_target;
    const Vector3f next_target = _consumed_target; // Actual input to this Rate step, not a future Attitude target.
    log_UAMT motor_row = {LOG_PACKET_HEADER_INIT(LOG_UAMT_MSG), _time_us, _seq,
                          uint8_t(_healthy ? (_motors.armed() ? 1 : 0) : 3), uint8_t(_healthy ? 0 : 255), {}, {},
                          _native_calls, _seq, _motor_calls, 0
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
    // Dynamic message IDs use the existing logger allocator; this code is absent on H743.
    AP::logger().WriteStreaming("UGC1", chain_labels, chain_format, _time_us, _seq, _source_seq, _native_calls,
                                rate.output_writes(), _audit_faults, _mode,
                                uint8_t(_motors.limit.roll | (_motors.limit.pitch << 1) | (_motors.limit.yaw << 2)),
                                double(next_target.x), double(next_target.y), double(next_target.z));
#endif
}

#endif
