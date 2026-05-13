#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AC_PID/AC_PID.h>

/*
  AR_FoilControl — foilboat cascaded controller (rover sibling to AR_AttitudeControl).

  PR5 scope (per research/flight-dev-rig/05-ardufoil-integration.md §8 and
  04-control-law.md §1–§5):
    - Attitude wrappers (theta/phi/psi) feeding the PR4 body-rate PIDs.
    - Outer loops: height -> theta_cmd (100 Hz), speed -> throttle (50 Hz),
      heading -> r_setpoint (in attitude/outer composite at 100 Hz).
    - V^2 gain scheduling on pitch/roll attitude + all three rate PIDs.
    - Cascade-level anti-windup: when pitch_mix_cmd saturates for > 0.2 s,
      the height-loop integrator is frozen (§5.3).
    - Mixer refinements: roll_diff_cmd clamped to ±0.262 rad (±15°),
      pitch_mix_cmd clamped to ±0.436 rad (±25°), per §5.4.
    - Throttle output routed through SRV_Channels::k_throttle via the
      Rover motors path (g2.motors.set_throttle).
  Deferred to PR6: failsafe state machine, persistent-saturation latch to
  AUTO_DESCEND (§5.5), lift-trim integrator (§2.2), HULL_BORNE pre-foilborne
  trim (§4 — taking the "simpler alternative": HULL_BORNE keeps PR4 routing
  via is_foilborne_mode()==false, so the v0 cascade is bypassed in HULL_BORNE).
*/
class AR_FoilControl {
public:

    // constructor
    AR_FoilControl();

    // do not allow copying
    CLASS_NO_COPY(AR_FoilControl);

    static AR_FoilControl *get_singleton() { return _singleton; }

    // One-shot init: configure SRV_Channel angle ranges + capture nominal PID
    // gains for V^2 scheduling.  Idempotent; safe to call repeatedly.
    void init();

    // outer loop: height + roll + heading -> body-rate / attitude targets (100 Hz).
    void update_outer();

    // inner loop: attitude wrappers + body-rate PIDs -> surface deflections (400 Hz).
    void update_inner();

    // throttle / forward-speed loop (called at 50 Hz)
    void update_throttle();

    // failsafe state machine (called at 10 Hz)
    void update_failsafe();

    // Write the latest inner-loop surface commands to k_foilcontrol_{canard,main,rudder}.
    // Called from Rover::set_servos() when control_mode->is_foilborne_mode() is true.
    void output_to_servos();

    // setpoint setters used by Mode classes
    void set_height_target(float h_m)        { _height_target_m   = h_m; }
    void set_speed_target(float v_ms)        { _speed_target_ms   = v_ms; }
    void set_heading_target_rad(float yaw)   { _heading_target_rad = yaw; }
    void set_pitch_target_rad(float th)      { _pitch_target_override_rad = th; _pitch_target_override = true; }
    void clear_pitch_target_override()       { _pitch_target_override = false; }

    // Returns height above water in metres, or NaN if rangefinder is unhealthy / dropped.
    // Reads from the downward-facing instance configured via RNGFND1_ORIENT = PITCH_270.
    float get_height_above_water() const;

    // True when the pitch inner-loop mixer output has been saturated for > 0.2 s.
    // Outer-loop integrator(s) should freeze while this holds (§5.3).
    bool pitch_saturated() const { return _pitch_saturated; }

    // Latest throttle command from the speed PID (0..1).  Modes that close
    // the speed loop read this each tick and pass it to AP_MotorsUGV.
    float get_throttle_cmd() const { return _throttle_cmd; }

    // Soft-engage gain (0..1) — ramps up over 1 s after the boat crosses the
    // foilborne threshold, then stays at 1.  Multiplied into all attitude /
    // body-rate outputs to prevent the cascade slamming at transition.
    float engage_factor() const { return _engage_factor; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_FoilControl *_singleton;

    // --- inner (body-rate) PIDs (PR4) -------------------------------------
    AC_PID _p_rate_pid;   // roll-rate p  (axis-x) -> differential main flap
    AC_PID _q_rate_pid;   // pitch-rate q (axis-y) -> canard + main via §2 mixer
    AC_PID _r_rate_pid;   // yaw-rate r  (axis-z) -> rudder

    // --- outer attitude/speed PIDs (PR5) ----------------------------------
    // Roll attitude (phi_cmd -> p_setpoint), Kp=0.6 / Ki=0.05 / Kd=0.04 per §1.2.
    AC_PID _phi_pid;
    // Forward speed (V_cmd -> throttle), Kp=0.35 / Ki=0.10 / Kd=0.0 per §1.5.
    AC_PID _v_pid;
    // Pitch attitude (theta_cmd -> q_setpoint), Kp=4.0 / Ki=0.5 / Kd=0.19 per §1.1.
    // Kd is on q (body rate), not on dθ/dt: AC_PID's D path is disabled (Kd=0
    // in the controller) and we add a manual `-Kd_theta * q` term outside.
    AC_PID _theta_pid;
    // Heading PID (psi_cmd -> r_setpoint), Kp=1.0 / Ki=0 / Kd=0 first-cut.
    AC_PID _psi_pid;

    // --- height outer loop (manual PID — keeps the legacy AP_Float gains) -
    // Preserves FOIL_HGT_P / FOIL_HGT_I / FOIL_HGT_D parameter names from PR1
    // unchanged; the integrator + last-zdot live here as private state.
    AP_Float _h_p;
    AP_Float _h_i;
    AP_Float _h_d;
    float    _h_integrator;
    float    _h_last_theta_cmd;       // last good theta_cmd, held during LIDAR dropouts

    // --- scalar thresholds / setpoint defaults ----------------------------
    AP_Float _h_target;             // FOIL_HGT_TGT - default ride height (m)
    AP_Float _v_takeoff;            // FOIL_VTOFF   - foilborne takeoff speed (m/s)
    AP_Float _h_foilborne_thresh;   // FOIL_HGT_FB  - height threshold for foilborne mode (m)
    AP_Float _vcruise;              // FOIL_VCRUISE  - V_cruise for V^2 scheduling (m/s)
    AP_Float _vmin_sched;           // FOIL_VMINSCHD - lower clamp on V before V^2 explodes (m/s)
    AP_Float _kd_theta;             // FOIL_KD_THETA - pitch attitude D on q (Kd manual term)

    // --- volatile setpoints (not persisted) -------------------------------
    float _height_target_m;
    float _speed_target_ms;
    float _heading_target_rad;
    bool  _pitch_target_override;
    float _pitch_target_override_rad;

    // --- last inner-loop surface commands (post-mixer, post-saturation) ---
    float _canard_cmd_rad;
    float _main_cmd_rad;
    float _rudder_cmd_rad;
    float _throttle_cmd;              // latest speed-PID output [0..1]

    // --- last outer-loop intermediates (for cross-tick use + logging) -----
    float _theta_cmd_rad;             // from update_outer or pilot trim
    float _p_setpoint_rad_s;          // roll-rate target from phi PID
    float _q_setpoint_rad_s;          // pitch-rate target from theta PID
    float _r_setpoint_rad_s;          // yaw-rate target from psi PID

    // --- V^2 scheduling captured nominal gains ----------------------------
    // Captured in init() so we can rescale each tick without losing the
    // user-set EEPROM values.
    float _nom_p_rate_kp, _nom_p_rate_ki, _nom_p_rate_kd;
    float _nom_q_rate_kp, _nom_q_rate_ki, _nom_q_rate_kd;
    float _nom_r_rate_kp, _nom_r_rate_ki, _nom_r_rate_kd;
    float _nom_phi_kp,    _nom_phi_ki,    _nom_phi_kd;
    float _nom_theta_kp,  _nom_theta_ki,  _nom_theta_kd;
    bool  _nominal_gains_captured;

    // --- cascade-level pitch-saturation tracking (§5.3) -------------------
    float    _pitch_sat_dwell_s;      // time mixer pitch_mix_cmd has been clipped
    bool     _pitch_saturated;        // dwell > 0.2 s

    // --- foilborne-state latch (cross-loop signal) ------------------------
    // True when h_meas > FOIL_HGT_FB.  Set by update_outer() each tick,
    // consumed by update_inner() to gate attitude wrappers + integrators.
    // Replaces the per-tick mode-flag check (cheaper + cascade-internal).
    bool _foilborne_now;
    // Time (seconds since foilborne) since the boat last entered foilborne.
    // Used to soft-engage the height-PID output over a 1 s ramp so the
    // cascade doesn't slam at the threshold transition.  Reset to 0 each
    // time _foilborne_now goes false->true.
    float _foilborne_engage_s;
    // Cosine-ramp gain in [0,1], updated each outer tick.
    float _engage_factor;

    // --- dt bookkeeping per loop ------------------------------------------
    uint32_t _last_inner_us;
    uint32_t _last_outer_us;
    uint32_t _last_throttle_us;

    // init() bookkeeping
    bool _servo_ranges_set;

    // --- helpers ----------------------------------------------------------
    // Apply V^2 scheduling: multiply nominal gains by clamp((Vc/max(V,Vmin))^2).
    void apply_vsq_scheduling();
    // Capture nominal gains from each PID's current kP/kI/kD (one-shot).
    void capture_nominal_gains();
};
