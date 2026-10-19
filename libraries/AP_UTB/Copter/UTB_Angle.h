/// @file UTB_Angle.h
/// @brief Native AP-derived Angle mode, one UTB rate owner, SITL only.
/// @author Jiri Kovar; original mode algorithms: ArduPilot contributors
/// @version 2026-10-19
/// @date 2026-10-19
// GNU GPL v3 or later; source: ArduCopter/mode_stabilize.cpp at b25e828b.
#pragma once
#if AP_UTB_RATE_SITL_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
class UTB_Angle : public Mode
{
public:
    using Mode::Mode;
    Number mode_number() const override
    {
        return Number::UTB_ANGLE;
    }
    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;
    bool allows_arming(AP_Arming::Method method) const override;
    bool requires_position() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool is_autopilot() const override
    {
        return false;
    }
    bool allows_entry_in_rc_failsafe() const override
    {
        return false;
    }
    bool crash_check_enabled() const override
    {
        return true;
    }
    bool allows_save_trim() const override
    {
        return true;
    }
    bool allows_auto_trim() const override
    {
        return true;
    }
    void prepare_mode_change() override;
    Vector3f monitoring_attitude_target() const override;
    float monitoring_attitude_error() const override;
    void monitoring_yaw_pid(float &ki, float &imax, float &integrator) const override;
    void rate_control(float dt);
    void log_cycle();
protected:
    const char *name() const override
    {
        return "UTB Angle";
    }
    const char *name4() const override
    {
        return "UANG";
    }
};
#endif
