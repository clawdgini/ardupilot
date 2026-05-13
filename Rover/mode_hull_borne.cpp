#include "Rover.h"

// HULL_BORNE: displacement-mode taxi.
// PR5: taking the "simpler alternative" path from the PR5 brief — HULL_BORNE
// keeps is_foilborne_mode() == false, so the AR_FoilControl cascade is gated
// out and surfaces still passthrough via the normal Rover steering/servo path
// (no controller-driven canard pre-load on the v0 rig). The §4 pre-foilborne
// trim schedule (canard 0°→+6°, main 0°→+2° linear in V/V_TO) is deferred to
// PR6 as a feedforward injection in AR_FoilControl::output_to_servos().
// TODO(PR6): wire §4 trim schedule + add foilborne-up transition detection.

bool ModeHullBorne::_enter()
{
    // default targets: drift speed, heading = current yaw, no height target.
    rover.g2.foil_control.set_speed_target(0.0f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    rover.g2.foil_control.notify_mode_change();
    return true;
}

void ModeHullBorne::update()
{
    // PR5: still passthrough — set targets so a manual mode switch into a
    // foilborne mode picks up sane defaults, but don't drive any servos here.
    rover.g2.foil_control.set_speed_target(0.0f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    // No height target — boat is hull-borne.
}
