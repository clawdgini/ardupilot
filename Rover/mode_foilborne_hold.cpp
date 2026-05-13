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
    // PR3 setpoint scaffolding — PR5 will close the height/heading/speed outer loops.
    rover.g2.foil_control.set_height_target(0.10f);  // 100 mm default ride height
    rover.g2.foil_control.set_speed_target(2.0f);    // cruise placeholder
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());

    // PR4: pilot stick throttle passthrough on SERVO3 (k_throttle). The speed
    // PID lands in PR5; until then the pilot drives forward speed directly so
    // the inner body-rate loop has aerodynamic authority to fight the unstable
    // pitch plant. Steering/rudder is owned by the inner loop via output_to_servos().
    float desired_steering;
    float desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    g2.motors.set_throttle(desired_throttle);
    // _steering is left zero so AP_MotorsUGV::output_regular() doesn't write
    // k_steering — the foilborne rudder comes from AR_FoilControl on k_foilcontrol_rudder.
    g2.motors.set_steering(0.0f);
}
