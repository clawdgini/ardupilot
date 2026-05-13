#include "Rover.h"

// FOILBORNE_HOLD: hold ride height, heading, and forward speed via AR_FoilControl.
// PR5: outer cascade is live — h_cmd, V_cmd, ψ_cmd feed the height/speed/yaw
// loops; pitch/roll attitude wrappers run inside AR_FoilControl::update_inner.
// Targets per 04-control-law.md §3.1 / §1.4 / §1.5:
//   h_cmd = 0.15 m   (target ride height for the 1 m rig)
//   V_cmd = V_TO + 0.30 = 1.90 m/s  (V_TO 1.6 m/s by spec; FOIL_VTOFF default 1.8)
//   ψ_cmd = heading at mode entry (heading-hold)
//   φ_cmd = 0        (closed inside AR_FoilControl outer; no v0 differential main)

bool ModeFoilborneHold::_enter()
{
    rover.g2.foil_control.set_height_target(0.15f);
    rover.g2.foil_control.set_speed_target(1.9f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    rover.g2.foil_control.clear_pitch_target_override();
    // PR6 §5: reset the failsafe-gate's mode-settle timer so the 2 s arm
    // delay starts from this boundary, not from boot.
    rover.g2.foil_control.notify_mode_change();
    return true;
}

void ModeFoilborneHold::update()
{
    // Setpoints are sticky after _enter(); no per-tick churn unless the pilot
    // adjusts the stick (TODO: PR6 hooks stick-driven heading/speed nudge).
    rover.g2.foil_control.set_height_target(0.15f);
    rover.g2.foil_control.set_speed_target(1.9f);
    // Heading target is NOT re-captured every tick — leave it at the entry
    // value so the boat actually holds heading instead of always pointing
    // wherever it just yawed to.

    // Pipe the speed-loop throttle (0..1) into AP_MotorsUGV (-100..+100).
    g2.motors.set_throttle(rover.g2.foil_control.get_throttle_cmd() * 100.0f);
    // _steering is left zero so AP_MotorsUGV::output_regular() doesn't write
    // k_steering — the foilborne rudder comes from AR_FoilControl on k_foilcontrol_rudder.
    g2.motors.set_steering(0.0f);
}
