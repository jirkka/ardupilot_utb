/// @file UTB_Angle.cpp
/// @brief AP-derived Angle; native mode manager, arming and motor output remain authoritative.
/// @author Jiri Kovar; original algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL v3 or later; original mode source at b25e828b.
#include "../../../ArduCopter/Copter.h"
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
#include <AP_UTB/UTB_Input.h>
bool UTB_Angle::init(bool ignore_checks)
{
    char failure[128];
    if (!copter.utb.enabled() || copter.utb.angle_enable() != 1 ||
        copter.utb3_core == nullptr || copter.using_rate_thread || copter.g2.att_enable.get() != 0 ||
        !copter.utb3_control->pre_arm_checks("UT3", failure, sizeof(failure))) {
        return false;
    }
    if (!copter.utb3_core->enter(42, *attitude_control)) {
        return false;
    }
    copter.utb3_control->set_yaw_rate_tc(copter.g2.command_model_pilot_y.get_rate_tc());
    return copter.utb3_core->healthy();
}
bool UTB_Angle::allows_arming(AP_Arming::Method method) const
{
    char failure[128];
    return copter.utb.enabled() && copter.utb.angle_enable() == 1 &&
           copter.utb3_core != nullptr && copter.utb3_core->healthy() && rc().has_valid_input() &&
           copter.utb3_control->pre_arm_checks("UT3", failure, sizeof(failure));
}
void UTB_Angle::exit()
{
    copter.utb3_core->leave(42);
}
void UTB_Angle::prepare_mode_change()
{
    copter.utb3_core->prepare_native(*attitude_control);
}
Vector3f UTB_Angle::monitoring_attitude_target() const
{
    return copter.utb3_control->get_att_target_euler_rad();
}
float UTB_Angle::monitoring_attitude_error() const
{
    return copter.utb3_control->get_att_error_angle_deg();
}
void UTB_Angle::monitoring_yaw_pid(float &ki, float &imax, float &integrator) const
{
    ki = copter.utb3_control->get_rate_yaw_pid().kI();
    imax = copter.utb3_control->get_rate_yaw_pid().imax();
    integrator = copter.utb3_control->get_rate_yaw_pid().get_pid_info().I;
}
void UTB_Angle::rate_control(float dt)
{
    copter.utb3_core->update_rate(dt);
}
void UTB_Angle::log_cycle()
{
    copter.utb3_core->log_cycle();
}
void UTB_Angle::run()
{
    UTB_Core &core = *copter.utb3_core;
    UTB_Rate &controller = core.rate;
    Vector3f sticks;
    Vector3f reference;
    Vector2f lean;
    float throttle;
    core.rc_valid = rc().has_valid_input();
    core.rc_ms = copter.g2.rc_channels.last_input_ms();
    // Native AP RC/failsafe code owns stale-input policy; validity is recorded independently.
    sticks = Vector3f(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz());
    core.sticks = sticks;
    apply_simple_mode(sticks.x, sticks.y);
    lean = UTB_Input::angle(sticks.x, sticks.y, controller.lean_angle_max_rad());
    reference = {lean.x, lean.y, radians(copter.g2.command_model_pilot_y.get_rate()) *
                 UTB_Input::expo(sticks.z, copter.g2.command_model_pilot_y.get_expo())
                };
    core.reference = reference;
    throttle = UTB_Input::throttle(channel_throttle->get_control_in(), copter.get_throttle_mid(), throttle_hover());
    motors->set_desired_spool_state(copter.ap.throttle_zero ? AP_Motors::DesiredSpoolState::GROUND_IDLE : AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        controller.reset_yaw_target_and_rate(true);
        controller.reset_rate_controller_I_terms();
        throttle = 0;
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        controller.reset_yaw_target_and_rate();
        controller.reset_rate_controller_I_terms_smoothly();
        throttle = 0;
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }
    controller.input_euler_angle_roll_pitch_euler_rate_yaw_rad(reference.x, reference.y, reference.z);
    core.manual_throttle = throttle;
    controller.set_throttle_out(throttle, true, copter.g.throttle_filt);

}

#endif
