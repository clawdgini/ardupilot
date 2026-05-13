#include "Rover.h"

// HULL_BORNE: displacement-mode taxi.
// PR3 scaffolding: set targets on AR_FoilControl; surfaces still go through
// the usual Rover::set_servos() passthrough path because is_foilborne_mode()
// returns false for this mode. PR4+ will add the foilborne servo branch.

bool ModeHullBorne::_enter()
{
    // default targets: drift speed, heading = current yaw, no height target.
    rover.g2.foil_control.set_speed_target(0.0f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    return true;
}

void ModeHullBorne::update()
{
    // PR3: targets only — actual servo writes still passthrough.
    rover.g2.foil_control.set_speed_target(0.0f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    // No height target — boat is hull-borne.
}
