#include "Rover.h"

// AUTO_DESCEND: monotone-down ride-height schedule, transitions to HULL_BORNE
// once height-above-water touches the threshold. PR6 implements the ramp +
// touch-detection state machine; PR3 just stubs the targets.

bool ModeAutoDescend::_enter()
{
    // default targets on entry: command zero ride height, slow forward speed,
    // hold current heading.
    rover.g2.foil_control.set_height_target(0.0f);
    rover.g2.foil_control.set_speed_target(0.5f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    return true;
}

void ModeAutoDescend::update()
{
    // PR6 will replace this stub with a time-parametrised descend schedule.
    rover.g2.foil_control.set_height_target(0.0f);
    rover.g2.foil_control.set_speed_target(0.5f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
}
