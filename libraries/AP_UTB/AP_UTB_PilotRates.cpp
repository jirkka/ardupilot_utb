//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file AP_UTB_PilotRates.cpp
/// @brief Actual Rates a samostatný experimentální pilotní profil; pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-18
/// @date 2026-10-18
//--------------------------------------------------------------------------------------------------------------------
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
// Include Files
#include "AP_UTB_PilotRates.h"
#if AP_UTB_RATE_SITL_AVAILABLE
// End Include Files
// Static data declarations
const AP_Param::GroupInfo AP_UTB_PilotRates::var_info[] = {
    // @Param: TYPE
    // @DisplayName: SITL pilot TYPE
    // @Description: Select Actual Rates and expanded pilot envelopes in SITL only. Edit outside pilot modes. Legacy C/A limits remain stored and unchanged.
    // @Range: 0 1
    // @Values: 0:Legacy,1:Actual experimental
    // @User: Advanced
    AP_GROUPINFO("TYPE", 1, AP_UTB_PilotRates, _type, 0),
    // @Param: R_C
    // @DisplayName: SITL pilot R_C
    // @Description: Actual Rates center slope.
    // @Range: 0 670
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("R_C", 2, AP_UTB_PilotRates, _center[0], 160),
    // @Param: R_M
    // @DisplayName: SITL pilot R_M
    // @Description: Actual Rates full-stick request; must be at least center. Not a validated achieved rate.
    // @Range: 1 670
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("R_M", 3, AP_UTB_PilotRates, _maximum[0], 670),
    // @Param: R_E
    // @DisplayName: SITL pilot R_E
    // @Description: Actual Rates expo fraction; 0.3 corresponds to 30 percent.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("R_E", 4, AP_UTB_PilotRates, _expo[0], 0.3),
    // @Param: P_C
    // @DisplayName: SITL pilot P_C
    // @Description: Actual Rates center slope.
    // @Range: 0 670
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("P_C", 5, AP_UTB_PilotRates, _center[1], 160),
    // @Param: P_M
    // @DisplayName: SITL pilot P_M
    // @Description: Actual Rates full-stick request; must be at least center. Not a validated achieved rate.
    // @Range: 1 670
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("P_M", 6, AP_UTB_PilotRates, _maximum[1], 670),
    // @Param: P_E
    // @DisplayName: SITL pilot P_E
    // @Description: Actual Rates expo fraction; 0.3 corresponds to 30 percent.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("P_E", 7, AP_UTB_PilotRates, _expo[1], 0.3),
    // @Param: Y_C
    // @DisplayName: SITL pilot Y_C
    // @Description: Actual Rates center slope.
    // @Range: 0 670
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("Y_C", 8, AP_UTB_PilotRates, _center[2], 160),
    // @Param: Y_M
    // @DisplayName: SITL pilot Y_M
    // @Description: Actual Rates full-stick request; must be at least center. Not a validated achieved rate.
    // @Range: 1 670
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("Y_M", 9, AP_UTB_PilotRates, _maximum[2], 500),
    // @Param: Y_E
    // @DisplayName: SITL pilot Y_E
    // @Description: Actual Rates expo fraction; 0.3 corresponds to 30 percent.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("Y_E", 10, AP_UTB_PilotRates, _expo[2], 0.3),
    // @Param: ANGLE
    // @DisplayName: SITL pilot ANGLE
    // @Description: Experimental ANGLE circular reference limit. Vertical and attitude guards still apply.
    // @Range: 1 60
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("ANGLE", 11, AP_UTB_PilotRates, _angle, 45),
    // @Param: ARATE
    // @DisplayName: SITL pilot ARATE
    // @Description: ANGLE RP body-rate norm cap; independent of maximum angle and ACRO rates.
    // @Range: 1 180
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("ARATE", 12, AP_UTB_PilotRates, _angle_rate, 90),
    AP_GROUPEND
};
// End Static data declarations
// Public functions
/// @brief Set AP parameter defaults for the subgroup.
/// @param[in] None
/// @return None.
/// @details Constructor; no controller history is modified.
AP_UTB_PilotRates::AP_UTB_PilotRates()
{
    AP_Param::setup_object_defaults(this, var_info);
}
/// @brief Read all fixed-size AP fields.
/// @param[in] None
/// @return Current profile.
/// @details Used before entry and for frozen configuration checks.
AP_UTB_PilotRates::Config AP_UTB_PilotRates::snapshot() const
{
    // Explicit snapshot returned by value; axis indexes all three fixed arrays.
    Config out;
    uint8_t axis;
    out.type = _type.get();
    for (axis = 0; axis < 3; axis++) {
        out.center[axis] = _center[axis].get();
        out.maximum[axis] = _maximum[axis].get();
        out.expo[axis] = _expo[axis].get();
    }
    out.angle = _angle.get();
    out.angle_rate = _angle_rate.get();
    return out;
}
/// @brief Check the supported finite mathematical domain.
/// @param[in] None
/// @return Validity, independently of physical performance.
/// @details Legacy type ignores dormant experimental fields.
bool AP_UTB_PilotRates::Config::valid() const
{
    // Index of the independently configured body axis.
    uint8_t axis;
    if (type == 0) {
        return true;
    }
    if (type != 1 || !isfinite(angle) || angle < 1 || angle > 60 ||
        !isfinite(angle_rate) || angle_rate < 1 || angle_rate > 180) {
        return false;
    }
    for (axis = 0; axis < 3; axis++) {
        if (!isfinite(AP_UTB_PilotRates::actual(0, center[axis], maximum[axis], expo[axis]))) {
            return false;
        }
    }
    return true;
}
/// @brief Compare complete frozen profiles.
/// @param[in] other Runtime profile.
/// @return Exact field equality, false for NaN.
/// @details Avoids silent in-flight parameter changes.
bool AP_UTB_PilotRates::Config::operator==(const Config &other) const
{
    // Every axis is compared without an epsilon deadband.
    uint8_t axis;
    if (type != other.type || !(angle <= other.angle && angle >= other.angle &&
                                angle_rate <= other.angle_rate && angle_rate >= other.angle_rate)) {
        return false;
    }
    for (axis = 0; axis < 3; axis++) {
        if (!(center[axis] <= other.center[axis] && center[axis] >= other.center[axis] &&
              maximum[axis] <= other.maximum[axis] && maximum[axis] >= other.maximum[axis] &&
              expo[axis] <= other.expo[axis] && expo[axis] >= other.expo[axis])) {
            return false;
        }
    }
    return true;
}
/// @brief Compute the Actual Rates curve in explicit physical units.
/// @param[in] stick Normalized input.
/// @param[in] center Center slope in deg/s.
/// @param[in] maximum Endpoint in deg/s.
/// @param[in] expo Expo fraction.
/// @return Rate in rad/s, or NaN for invalid arguments.
/// @details Independent expression of BF 4.5.0 applyActualRates, GPL-3.0-or-later.
float AP_UTB_PilotRates::actual(float stick, float center, float maximum, float expo)
{
    // Signed quintic term and unsigned stick displacement from center.
    float fifth;
    float displacement;
    if (!isfinite(stick) || fabsf(stick) > 1 || !isfinite(center) || center < 0 ||
        !isfinite(maximum) || maximum < 1 || maximum > 670 || center > maximum ||
        !isfinite(expo) || expo < 0 || expo > 1) {
        return NAN;
    }
    fifth = stick * stick * stick * stick * stick;
    displacement = fabsf(stick);
    return radians(center * stick + (maximum - center) * displacement *
                   ((1 - expo) * stick + expo * fifth));
}
// End Public functions
#endif // AP_UTB_RATE_SITL_AVAILABLE
