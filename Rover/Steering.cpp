#include "Rover.h"

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // Foilboat PR4: if the current mode is foilborne, overlay the three foil
    // surfaces (canard / main / rudder) with the inner-loop outputs BEFORE
    // motors.output() runs, so SRV_Channels::calc_pwm() inside motors.output()
    // picks up our scaled values on the same cycle. Throttle stays on the
    // rover's normal k_throttle path so manual stick / future PR5 speed PID
    // still works; g2.motors.output() does not touch k_foilcontrol_* channels.
    if (control_mode != nullptr && control_mode->is_foilborne_mode()) {
        g2.foil_control.output_to_servos();
    }

    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        // get ground speed
        float speed = 0.0f;
        g2.attitude_control.get_forward_speed(speed);

        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
