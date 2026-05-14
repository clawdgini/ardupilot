#include "Rover.h"

// FOILBORNE_HOLD: hold ride height, heading, and forward speed via AR_FoilControl.
// PR5: outer cascade is live — h_cmd, V_cmd, ψ_cmd feed the height/speed/yaw
// loops; pitch/roll attitude wrappers run inside AR_FoilControl::update_inner.
// Targets per 04-control-law.md §3.1 / §1.4 / §1.5:
//   h_cmd = FBHD_H_REF  (default 0.15 m for the 1 m rig; mode-level param so
//                        SITL step-response harness can override at runtime
//                        without recompiling)
//   V_cmd = V_TO + 0.30 = 1.90 m/s  (V_TO 1.6 m/s by spec; FOIL_VTOFF default 1.8)
//   ψ_cmd = heading at mode entry (heading-hold)
//   φ_cmd = 0        (closed inside AR_FoilControl outer; no v0 differential main)

const AP_Param::GroupInfo ModeFoilborneHold::var_info[] = {
    // @Param: _H_REF
    // @DisplayName: Foilborne ride-height setpoint
    // @Description: Target ride height (m) above the hull-borne datum while in FOILBORNE_HOLD. Default 0.15 m matches the 1 m flight-dev rig spec (04-control-law.md §1.4). The mode reads this every tick, so a runtime param-set steps the height command for SITL closed-loop characterisation (W5).
    // @Units: m
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_H_REF",   1, ModeFoilborneHold, _h_ref_m,         0.15f),

    // @Param: _TH_OV
    // @DisplayName: Foilborne pitch override
    // @Description: When non-zero, bypasses the height->pitch cascade and forces theta_cmd = FBHD_TH_OV (radians). Used by the W5 step-response harness to characterise the inner pitch loop independent of the outer height loop. Set 0 (default) for normal cascaded operation.
    // @Units: rad
    // @Range: -0.262 0.262
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("_TH_OV",   2, ModeFoilborneHold, _th_override_rad, 0.0f),

    AP_GROUPEND
};

ModeFoilborneHold::ModeFoilborneHold() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeFoilborneHold::_enter()
{
    rover.g2.foil_control.set_height_target(_h_ref_m);
    rover.g2.foil_control.set_speed_target(1.9f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    if (!is_zero(_th_override_rad)) {
        rover.g2.foil_control.set_pitch_target_rad(_th_override_rad);
    } else {
        rover.g2.foil_control.clear_pitch_target_override();
    }
    // PR6 §5: reset the failsafe-gate's mode-settle timer so the 2 s arm
    // delay starts from this boundary, not from boot.
    rover.g2.foil_control.notify_mode_change();
    return true;
}

void ModeFoilborneHold::update()
{
    // PR7a D1: failsafe-descend handover.  AR_FoilControl raises this latch
    // when the duty-cycle saturation trigger has held under the armed gate
    // (foilborne + settled + above V_TO) for FOIL_FAIL_DWELL ms.  Swap to
    // AUTO_DESCEND immediately; foil_control's notify_mode_change() (called
    // from AutoDescend::_enter) wipes the latch + dwell + duty buffer so the
    // failsafe state machine starts fresh in the descent mode.
    if (rover.g2.foil_control.should_failsafe_descend()) {
        rover.set_mode(rover.mode_auto_descend, ModeReason::FAILSAFE);
        return;
    }

    // PR7b: setpoints sourced from mode-level params so a runtime param-set
    // steps the command (used by the W5 step-response harness).  Default
    // FBHD_H_REF = 0.15 matches the legacy hardcoded value, so existing
    // flights see no behaviour change.
    rover.g2.foil_control.set_height_target(_h_ref_m);
    rover.g2.foil_control.set_speed_target(1.9f);
    // Pitch override: when FBHD_TH_OV is non-zero, bypass the height->pitch
    // cascade and force theta_cmd directly.  When zero, restore normal
    // cascaded operation (idempotent if already cleared).
    if (!is_zero(_th_override_rad)) {
        rover.g2.foil_control.set_pitch_target_rad(_th_override_rad);
    } else {
        rover.g2.foil_control.clear_pitch_target_override();
    }
    // Heading target is NOT re-captured every tick — leave it at the entry
    // value so the boat actually holds heading instead of always pointing
    // wherever it just yawed to.

    // Pipe the speed-loop throttle (0..1) into AP_MotorsUGV (-100..+100).
    g2.motors.set_throttle(rover.g2.foil_control.get_throttle_cmd() * 100.0f);
    // _steering is left zero so AP_MotorsUGV::output_regular() doesn't write
    // k_steering — the foilborne rudder comes from AR_FoilControl on k_foilcontrol_rudder.
    g2.motors.set_steering(0.0f);
}
