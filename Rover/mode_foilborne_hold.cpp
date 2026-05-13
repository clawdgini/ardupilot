#include "Rover.h"

// FOILBORNE_HOLD: hold ride height, heading, and forward speed via AR_FoilControl.
// PR3 scaffolding only: sets setpoint targets; surface PIDs land in PR4/PR5.

bool ModeFoilborneHold::_enter()
{
    // default targets on entry: 100 mm ride height, 2 m/s cruise placeholder,
    // current yaw as heading hold.
    rover.g2.foil_control.set_height_target(0.10f);
    rover.g2.foil_control.set_speed_target(2.0f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    return true;
}

void ModeFoilborneHold::update()
{
    // PR3: targets only — PR4/PR5 will run the cascaded controllers.
    rover.g2.foil_control.set_height_target(0.10f);  // 100 mm default ride height
    rover.g2.foil_control.set_speed_target(2.0f);    // cruise placeholder
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
}
