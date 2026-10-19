//--------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB project extension; original licenses remain unchanged.
//--------------------------------------------------------------------------------------------------------------------
/// @file AP_UTB_PilotRates.h
/// @brief Actual Rates a samostatný experimentální pilotní profil; pouze SITL.
/// @author Jiri Kovar
/// @version 2026-10-18
/// @date 2026-10-18
//--------------------------------------------------------------------------------------------------------------------
#pragma once
// GNU General Public License version 3 or later; see COPYING.txt.
// Include Files
#include "AP_UTB_config.h"
#if AP_UTB_RATE_SITL_AVAILABLE
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
// End Include Files
// Data type, constant, macro definition
class AP_UTB_PilotRates
{
public:
    struct Config {
        // 0 preserves the legacy pilot reference and all legacy envelope limits.
        int8_t type = 0;
        // Per-axis center slope and full-stick rate in degrees/second; expo in 0..1.
        Vector3f center{160, 160, 160};
        Vector3f maximum{670, 670, 500};
        Vector3f expo{0.3f, 0.3f, 0.3f};
        // ANGLE reference norm in degrees, independent RP body-rate cap in degrees/second.
        float angle = 45;
        float angle_rate = 90;
        /// @brief Validate a finite experimental profile without claiming flight qualification.
        /// @param[in] None
        /// @return True for supported mathematical parameter domains.
        /// @details 670 deg/s is a configuration bound, not a demonstrated plant capability.
        bool valid() const;
        /// @brief Compare every field of the frozen profile, rejecting NaN.
        /// @param[in] other Current AP parameter snapshot.
        /// @return Exact equality.
        /// @details A live edit latches the existing mode CONFIG fault.
        bool operator==(const Config &other) const;
    };
    // Public functions
    /// @brief Initialise the fixed AP parameter subgroup.
    /// @param[in] None
    /// @return None.
    /// @details No allocation in the control path.
    AP_UTB_PilotRates();
    /// @brief Read one explicit profile snapshot.
    /// @param[in] None
    /// @return Current configuration.
    /// @details The mode freezes this before entering control.
    Config snapshot() const;
    /// @brief Evaluate the Betaflight 4.5.0 Actual Rates mathematical map.
    /// @param[in] stick Normalized calibrated input in -1..1.
    /// @param[in] center Center slope in degrees/second.
    /// @param[in] maximum Full-stick rate in degrees/second, at least center.
    /// @param[in] expo Expo fraction in 0..1.
    /// @return Body rate in rad/s, NaN for invalid inputs.
    /// @details Formula source: Betaflight 4.5.0 src/main/fc/rc.c applyActualRates; no RC smoothing.
    static float actual(float stick, float center, float maximum, float expo);
    static const AP_Param::GroupInfo var_info[];
    // End Public functions
private:
    // Member data declarations (AP parameter storage, no dynamic buffers).
    AP_Int8 _type;
    AP_Float _center[3], _maximum[3], _expo[3];
    AP_Float _angle, _angle_rate;
    // End Member data declarations
};
// End Data type, constant, macro definition
#endif // AP_UTB_RATE_SITL_AVAILABLE
