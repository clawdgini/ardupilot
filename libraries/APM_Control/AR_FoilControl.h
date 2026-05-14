#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

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

  PR6 scope (this commit): three coordinated changes that fall out of the
  three expert reviews of PR5's SITL smoke-test failure (1.6 s to ±25° clamp):
    - AC_PID idiom refactor (codebase expert):
      * Drop the manual `-Kd_theta·q` workaround. AC_PID's D-on-error path
        is the canonical idiom; we fold Kd_θ into Kd_q on the inner pitch-rate
        PID so a single AC_PID owns the pitch derivative term.
      * Yaw outer wrapper switches from AC_PID to AC_P (Ki_ψ=Kd_ψ=0 per spec).
      * Pitch outer stays AC_PID so the §2.2 lift-trim integrator (Ki_θ=0.5)
        survives. Roll stays AC_PID (Kd_φ=0.04 is on dφ/dt — what AC_PID does).
    - Control-law refinements (flight-controls expert):
      * Canard pre-load: linear → quadratic in V/V_TO, peak +4° (was +6°)
        so canard α at V_TO is 10° (was 12°), keeping a 2° margin to α_stall.
      * Main pre-load: linear, peak +2°. Both bypass the pitch integrator
        and ramp down via (1 - engage_factor) once foilborne.
      * V² scheduling: piecewise — frozen (Vc/Vto)² below V_TO, V² in-band,
        0.25 floor above V_cruise. Replaces the 4× hard cap.
      * Canard lead compensator H(s) = (1+0.060s)/(1+0.015s) downstream of
        the mixer to restore PM ~35° at ω≈30 rad/s (servo τ=55 ms eats ~58°).
    - Failsafe gating (also controls expert):
      * Saturation triggers (5-s duty + 0.5-s continuous) armed only when
        foilborne + settled (V > V_TO+0.2, engage > 0.9, t_since_mode > 2 s).
      * AUTO_DESCEND actual mode-switch wiring deferred to PR7.

  Deferred to PR7: failsafe state-machine *action* (the latch into
  AUTO_DESCEND is logged as a TODO here), FOIL LogStructure extension
  for Preld/SchS/SatA (currently only WriteStreaming with the PR5 set of
  fields, which doesn't need a static LogStructure entry).
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

    // Notify the failsafe gate of a mode change.  Optional — the controller
    // also auto-detects mode changes via the singleton control-mode pointer
    // if the caller doesn't invoke this, but explicit calls let modes signal
    // the transition exactly on their _enter() boundary.
    // PR7a D3: also wipes the duty-cycle ring buffer so the trigger
    // re-accumulates from scratch after every mode change (edge-case guard
    // for the warm-up window).
    void notify_mode_change();

    // Returns height above water in metres, or NaN if rangefinder is unhealthy / dropped.
    // Reads from the downward-facing instance configured via RNGFND1_ORIENT = PITCH_270.
    float get_height_above_water() const;

    // True when the pitch inner-loop mixer output has been saturated for > 0.2 s.
    // Outer-loop integrator(s) should freeze while this holds (§5.3).
    bool pitch_saturated() const { return _pitch_saturated; }

    // Saturation duty cycle over the last FOIL_SAT_WIN_MS window (5 s default).
    // Range [0..1]; reads 0 until the rolling buffer is full (post-boot warm-up).
    float saturation_duty() const { return _sat_duty_cycle; }

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
    // Kd_φ is on dφ/dt, which is exactly what AC_PID's D-on-error path computes.
    AC_PID _phi_pid;
    // Forward speed (V_cmd -> throttle), Kp=0.35 / Ki=0.10 / Kd=0.0 per §1.5.
    AC_PID _v_pid;
    // Pitch attitude (theta_cmd -> q_setpoint), Kp=4.0 / Ki=0.5 / Kd=0 per §1.1
    // (PR6: Kd_θ folded into Kd_q on the inner pitch-rate PID — see _q_rate_pid).
    // Retained as AC_PID (not AC_P) to preserve the §2.2 lift-trim integrator.
    AC_PID _theta_pid;
    // Heading P controller (psi_cmd -> r_setpoint), Kp=1.0 per §1.3 first-cut.
    // PR6: switched from AC_PID to AC_P — spec has Ki_ψ=0, Kd_ψ=0 so AC_P is
    // the canonical wrapper (matches AR_AttitudeControl::_steer_angle_p).
    AC_P   _psi_p;

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
    AP_Float _vto;                  // FOIL_VTO      - take-off reference speed for pre-load schedule (m/s)
    AP_Float _pre_can;              // FOIL_PRE_CAN  - canard pre-load peak at V=V_TO (rad)
    AP_Float _pre_main;             // FOIL_PRE_MAIN - main pre-load peak at V=V_TO (rad)
    AP_Float _sched_floor;          // FOIL_SCHED_FL - lower bound on V² gain scale (above V_cruise)
    AP_Float _lead_tau_lead;        // FOIL_LEAD_LD  - canard lead compensator zero (s)
    AP_Float _lead_tau_lag;         // FOIL_LEAD_LG  - canard lead compensator pole (s)
    AP_Int8  _lead_en;              // FOIL_LEAD_EN  - bypass switch for the lead compensator (0/1)

    // PR7a D3: failsafe duty-cycle trigger param.
    AP_Float _sat_duty_thr;         // FOIL_SAT_DUTY_THR - duty-cycle threshold (0..1)

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
    // user-set EEPROM values.  Yaw is AC_P so only kP is captured.
    float _nom_p_rate_kp, _nom_p_rate_ki, _nom_p_rate_kd;
    float _nom_q_rate_kp, _nom_q_rate_ki, _nom_q_rate_kd;
    float _nom_r_rate_kp, _nom_r_rate_ki, _nom_r_rate_kd;
    float _nom_phi_kp,    _nom_phi_ki,    _nom_phi_kd;
    float _nom_theta_kp,  _nom_theta_ki,  _nom_theta_kd;
    float _nom_psi_kp;
    bool  _nominal_gains_captured;

    // --- last gain scale (for logging) ------------------------------------
    float _last_gain_scale;

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

    // --- pre-load feed-forward (PR6, §4) ----------------------------------
    // Last pre-load applied to the canard (rad). Stashed for logging.
    float _canard_preload_last;

    // --- canard lead compensator state (PR6) ------------------------------
    // Discrete-time Tustin state for H(s) = (1 + tau_lead*s) / (1 + tau_lag*s)
    // applied to the canard command downstream of the mixer.
    float _lead_x_prev;
    float _lead_y_prev;

    // --- failsafe gating (PR6, Part 5; extended PR7a D3 + D1) -------------
    // arming gate: foilborne + settled (V > V_TO + 0.2, engage > 0.9,
    // t_since_mode_change > 2 s). Required precondition for the saturation
    // trigger to fire.
    bool  _sat_armed;
    // PR7a D3: fires when arming gate AND duty cycle > FOIL_SAT_DUTY_THR AND
    // the rolling buffer has been fully populated since boot/mode-change.
    // This is the signal polled by Rover modes via should_failsafe_descend()
    // (after the FOIL_FAIL_DWELL dwell counter — see _failsafe_dwell_ms).
    bool  _sat_trigger_armed;
    float _continuous_sat_s;          // dwell time for continuous-saturation latch
    float _t_since_mode_change_s;     // ramps up each failsafe tick, reset on mode change
    uint8_t _last_mode_num_seen;      // for auto-detect of mode changes if Mode forgot to notify

    // --- PR7a D3: rolling saturation duty-cycle buffer --------------------
    // 5 s window at the inner-loop rate (400 Hz scheduler tick) = 2000 samples.
    // Compile-time fixed size; FOIL_SAT_WIN_MS is a documented constant, not a
    // tunable param (the buffer length can't be changed at runtime). Buffer is
    // a circular bit-array; we also keep a running sum to avoid an O(N) recount
    // each tick.
    static constexpr uint16_t SAT_WINDOW_SAMPLES = 2000;  // 5 s @ 400 Hz
    uint8_t  _sat_buf[SAT_WINDOW_SAMPLES];     // 0 or 1 per inner-loop tick
    uint16_t _sat_buf_idx;                     // next-write index
    uint16_t _sat_buf_fill;                    // samples written so far (clamps at SAT_WINDOW_SAMPLES)
    uint16_t _sat_buf_sum;                     // running sum of the buffer
    float    _sat_duty_cycle;                  // _sat_buf_sum / SAT_WINDOW_SAMPLES, 0 until full


    // --- dt bookkeeping per loop ------------------------------------------
    uint32_t _last_inner_us;
    uint32_t _last_outer_us;
    uint32_t _last_throttle_us;
    uint32_t _last_failsafe_us;
    float    _last_inner_dt;          // for the lead compensator at 400 Hz

    // init() bookkeeping
    bool _servo_ranges_set;

    // --- helpers ----------------------------------------------------------
    // Apply V^2 scheduling: multiply nominal gains by clamp((Vc/max(V,Vmin))^2).
    void apply_vsq_scheduling();
    // Piecewise V² gain scale: frozen below V_TO, V² in-band, FOIL_SCHED_FL floor.
    float gain_scale(float V) const;
    // Canard / main pre-load schedules (rad). V is forward speed (m/s).
    float canard_preload_rad(float V) const;
    float main_preload_rad(float V) const;
    // First-order lead compensator on the canard command (discrete-time Tustin).
    // Returns y[n] given x[n] and updates the internal state.
    float canard_lead(float x);
    // Capture nominal gains from each PID's current kP/kI/kD (one-shot).
    void capture_nominal_gains();
    // PR7a D3: push a single sample (any-flap-at-limit bit) into the rolling
    // duty-cycle buffer and update _sat_duty_cycle. Called once per inner tick.
    void push_sat_sample(bool flap_at_limit);
};
