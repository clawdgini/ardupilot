#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
// TODO(PR4): include AC_PID once inner-loop body-rate PIDs wire in.
// #include <AC_PID/AC_PID.h>

/*
  AR_FoilControl — foilboat cascaded controller (rover sibling to AR_AttitudeControl).

  PR1 scope (per research/flight-dev-rig/05-ardufoil-integration.md §8):
    - skeleton singleton, empty update_* method bodies
    - AP_Param table exposes the scalar thresholds (HGT_TGT, VTOFF, HGT_FB)
    - AC_PID subgroups are stubbed (TODO PR4)
    - behaviour unchanged; outputs still passthrough via existing SERVOn_FUNCTION
*/
class AR_FoilControl {
public:

    // constructor
    AR_FoilControl();

    // do not allow copying
    CLASS_NO_COPY(AR_FoilControl);

    static AR_FoilControl *get_singleton() { return _singleton; }

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

    // outer-loop gains (height -> pitch-rate, roll -> roll-rate, heading -> yaw-rate)
    // TODO(PR4): wire into update_outer().
    AP_Float _h_p;
    AP_Float _h_i;
    AP_Float _h_d;

    // scalar thresholds / setpoint defaults
    AP_Float _h_target;             // FOIL_HGT_TGT - default ride height (m)
    AP_Float _v_takeoff;            // FOIL_VTOFF   - foilborne takeoff speed (m/s)
    AP_Float _h_foilborne_thresh;   // FOIL_HGT_FB  - height threshold for foilborne mode (m)

    // TODO(PR4): AC_PID members for body-rate inner loop, roll-rate, speed loop.
    //   _p_rate_pid, _q_rate_pid, _r_rate_pid, _phi_rate_pid, _v_pid
    // See research/flight-dev-rig/05-ardufoil-integration.md §1 and §5.

    // setpoints (volatile state, not params)
    float _height_target_m;
    float _speed_target_ms;
    float _heading_target_rad;
};
