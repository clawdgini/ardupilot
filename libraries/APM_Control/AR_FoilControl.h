#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AC_PID/AC_PID.h>

/*
  AR_FoilControl — foilboat cascaded controller (rover sibling to AR_AttitudeControl).

  PR4 scope (per research/flight-dev-rig/05-ardufoil-integration.md §8):
    - Body-rate inner loop (p, q, r) via three AC_PIDs at 400 Hz.
    - Mixer per 04-control-law.md §2: canard = +1.0·q_out, main = -0.40·q_out, rudder = r_out.
    - Saturate each channel to ±0.436 rad (±25°).
    - output_to_servos() writes k_foilcontrol_{canard,main,rudder} (SRV enums 190/191/192).
    - Rover::set_servos() routes foilborne modes through us; throttle stays on the normal path.
  Outer loops (height/roll/heading -> setpoints), speed PID, and failsafe SM remain deferred.
  AP_SUBGROUPINFO indices 4 and 5 are reserved for PR5 (_phi_rate_pid, _v_pid).
*/
class AR_FoilControl {
public:

    // constructor
    AR_FoilControl();

    // do not allow copying
    CLASS_NO_COPY(AR_FoilControl);

    static AR_FoilControl *get_singleton() { return _singleton; }

    // One-shot: configure SRV_Channel angle ranges for the three foil surfaces.
    // Safe to call repeatedly; ensures channels read ±2500 centidegree range = ±25° mechanical.
    void init();

    // outer loop: height + roll + heading -> body-rate targets (called at 100 Hz).
    // No-arg signature is required by AP_Scheduler::SCHED_TASK_CLASS (FUNCTOR_BIND
    // binds `void()`). PR4+ will compute dt internally from AP_HAL::millis().
    void update_outer();

    // inner loop: body rates -> surface deflections via SRV_Channels (called at 400 Hz)
    void update_inner();

    // throttle / forward-speed loop (called at 50 Hz)
    void update_throttle();

    // failsafe state machine (called at 10 Hz)
    void update_failsafe();

    // Write the latest inner-loop surface commands to k_foilcontrol_{canard,main,rudder}.
    // Called from Rover::set_servos() when control_mode->is_foilborne_mode() is true.
    void output_to_servos();

    // setpoint setters used by Mode classes (PR3 will use these)
    void set_height_target(float h_m)        { _height_target_m   = h_m; }
    void set_speed_target(float v_ms)        { _speed_target_ms   = v_ms; }
    void set_heading_target_rad(float yaw)   { _heading_target_rad = yaw; }

    // Returns height above water in metres, or NaN if rangefinder is unhealthy / dropped.
    // Reads from the downward-facing instance configured via RNGFND1_ORIENT = PITCH_270.
    float get_height_above_water() const;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_FoilControl *_singleton;

    // Body-rate inner-loop PIDs (PR4). Outputs are radians of flap-equivalent
    // deflection per radian/sec of rate error; mixer scales to physical channels.
    AC_PID _p_rate_pid;   // roll-rate p  (axis-x)  — output goes to differential main (deferred to PR5)
    AC_PID _q_rate_pid;   // pitch-rate q (axis-y)  — output drives canard & main via §2 mixer
    AC_PID _r_rate_pid;   // yaw-rate r  (axis-z)   — output drives rudder

    // outer-loop gains (height -> pitch-rate, roll -> roll-rate, heading -> yaw-rate)
    // TODO(PR5): wire into update_outer().
    AP_Float _h_p;
    AP_Float _h_i;
    AP_Float _h_d;

    // scalar thresholds / setpoint defaults
    AP_Float _h_target;             // FOIL_HGT_TGT - default ride height (m)
    AP_Float _v_takeoff;            // FOIL_VTOFF   - foilborne takeoff speed (m/s)
    AP_Float _h_foilborne_thresh;   // FOIL_HGT_FB  - height threshold for foilborne mode (m)

    // setpoints (volatile state, not params)
    float _height_target_m;
    float _speed_target_ms;
    float _heading_target_rad;

    // last inner-loop commands in radians of flap deflection (post-mixer, post-saturation).
    // output_to_servos() converts these to centidegree and writes via SRV_Channels.
    float _canard_cmd_rad;
    float _main_cmd_rad;
    float _rudder_cmd_rad;

    // dt bookkeeping for update_inner()
    uint32_t _last_inner_us;

    // init() bookkeeping
    bool _servo_ranges_set;
};
