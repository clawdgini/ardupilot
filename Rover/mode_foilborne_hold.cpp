#include "Rover.h"

// FOILBORNE_HOLD: hold ride height, heading, and forward speed via AR_FoilControl.
//
// PR8 (2026-05-14): implements the spec §3.1 HULL_BORNE → FOILBORNE_HOLD
// entry transition as a sub-state machine inside this mode.  Previously
// (PR5–PR7b) `_enter()` slammed `h_cmd = FBHD_H_REF = 0.15 m` immediately,
// which collided with the height-PID's pre-foilborne gate (`!_foilborne_now
// ⇒ θ_cmd = 0`) and caused cyclic touchdown / liftoff at the V_TO transition.
// W5 step-response tests never saw sustained foilborne flight as a result.
//
// PR8 fix per spec [`04-control-law.md` §3.1](research/flight-dev-rig/04-control-law.md#L160):
//
//   SUB_GATE (entry)  — Hold h_cmd at *current LIDAR height* (so the height
//                       PID has zero error and doesn't wind up against a
//                       commanded 0.15 m that the boat can't reach).  Wait
//                       for ALL of:
//                         V_gps     ≥ FBHD_V_GATE   (1.75 m/s, V_TO+0.15)
//                         h_lidar   ≥ FBHD_H_GATE   (0.04 m)
//                         dh/dt     ≥ FBHD_DHDT_GT  (0.15 m/s)
//                         |θ|       ≤ 0.17 rad
//                         |φ|       ≤ 0.17 rad
//                       to hold simultaneously for FBHD_GATE_HLD ms
//                       (default 800 ms per spec).  If the gate doesn't
//                       fire within FBHD_GATE_TMO ms, fall through to
//                       AUTO_DESCEND.
//
//   SUB_RAMP          — Ramp h_cmd from h_lidar(t_gate) → FBHD_H_REF at
//                       FBHD_RAMP_R m/s (default 0.30 m/s, per spec
//                       "ramp to 0.15 m at 0.30 m/s").  Transition to
//                       SUB_HOLD once the ramp completes.
//
//   SUB_HOLD          — Steady state: h_cmd = FBHD_H_REF.  The existing
//                       failsafe-latch + RC throttle wiring takes over from
//                       here unchanged.
//
// θ_trim_offset capture (spec §3.1 last line) is implicit — the lift_trim
// integrator inside AR_FoilControl is already gated by `_foilborne_now`
// (see AR_FoilControl.cpp::update_outer) so it doesn't run in SUB_GATE.
// When it activates during SUB_RAMP / SUB_HOLD, it starts from zero, not
// from a slammed reference; that's the spec's intent.
//
// All five §3.1 entry-gate values are AP_Param so SITL gate-sweeps land
// without a rebuild.  16-char-max name compliance per the PR7a-D1 boot-
// panic post-mortem.

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>

// --- Internal constants (not user-tunable) ----------------------------------

// dh/dt low-pass corner.  The spec calls for "dh/dt > 0.15 m/s" but doesn't
// specify filter dynamics; a 5 Hz first-order LP on the LIDAR finite-diff is
// snappy enough to catch a real climb (rise-time ≈ 0.03 s vs gate-hold 0.8 s
// of margin) while killing the 0.1 m/s peak-peak jitter that the plant's
// LIDAR-noise model injects (foilboat_sim.py adds ±5 mm @ 50 Hz).
static constexpr float DHDT_LP_TAU_S = 1.0f / (2.0f * float(M_PI) * 5.0f);  // ≈ 0.0318 s

// Attitude gates from §3.1 — locked, not param.  10° == 0.17 rad on each
// axis: this is the spec's "no rolling / pitching wildly" sanity gate.
static constexpr float ATTITUDE_GATE_RAD = 0.17f;

const AP_Param::GroupInfo ModeFoilborneHold::var_info[] = {
    // @Param: _H_REF
    // @DisplayName: Foilborne ride-height setpoint
    // @Description: Target ride height (m) above the hull-borne datum once foilborne. Default 0.15 m matches the 1 m flight-dev rig spec (04-control-law.md §1.4). The §3.1 entry transition ramps h_cmd from h_lidar(t_gate) → FBHD_H_REF at FBHD_RAMP_R m/s; runtime PARAM_SET of FBHD_H_REF mid-flight produces a step in the held setpoint (used by the W5 step-response harness).
    // @Units: m
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_H_REF",   1, ModeFoilborneHold, _h_ref_m,         0.15f),

    // @Param: _TH_OV
    // @DisplayName: Foilborne pitch override
    // @Description: When non-zero, bypasses the height->pitch cascade and forces theta_cmd = FBHD_TH_OV (radians). Used by the W5 step-response harness to characterise the inner pitch loop independent of the outer height loop. Set 0 (default) for normal cascaded operation.
    // @Units: rad
    // @Range: -0.262 0.262
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("_TH_OV",   2, ModeFoilborneHold, _th_override_rad, 0.0f),

    // @Param: _V_GATE
    // @DisplayName: FBHD entry V_gps gate
    // @Description: §3.1 entry-gate threshold on GPS groundspeed (m/s). The boat must hold V_gps ≥ this value (with the other entry gates) for FBHD_GATE_HLD ms before the height-ramp sub-state arms. Spec default 1.75 m/s = V_TO (1.6) + 0.15 m/s margin.
    // @Units: m/s
    // @Range: 0.5 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_V_GATE",  3, ModeFoilborneHold, _v_gate_ms,       1.75f),

    // @Param: _H_GATE
    // @DisplayName: FBHD entry h_lidar gate
    // @Description: §3.1 entry-gate threshold on downward-LIDAR height (m). The boat must hold h_lidar ≥ this value (with the other entry gates) for FBHD_GATE_HLD ms before the height-ramp sub-state arms. Spec default 0.04 m.
    // @Units: m
    // @Range: 0.0 0.2
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("_H_GATE",  4, ModeFoilborneHold, _h_gate_m,        0.04f),

    // @Param: _DHDT_GT
    // @DisplayName: FBHD entry dh/dt gate
    // @Description: §3.1 entry-gate threshold on the LP-filtered rate of LIDAR height (m/s). The boat must be actively climbing — this rejects the wave-driven case where h_lidar briefly clears the gate. Spec default 0.15 m/s.
    // @Units: m/s
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_DHDT_GT", 5, ModeFoilborneHold, _dhdt_gate_ms,    0.15f),

    // @Param: _GATE_HLD
    // @DisplayName: FBHD entry gate-hold time
    // @Description: §3.1 requires ALL entry gates (V_gps, h_lidar, dh/dt, |θ|, |φ|) to hold simultaneously for this long before the height-ramp sub-state arms. Spec default 800 ms.
    // @Units: ms
    // @Range: 100 5000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("_GATE_HLD", 6, ModeFoilborneHold, _gate_hold_ms,    800),

    // @Param: _GATE_TMO
    // @DisplayName: FBHD entry gate timeout
    // @Description: If non-zero, FBHD reverts to AUTO_DESCEND after this many ms in SUB_GATE without the entry conditions firing. The spec §3.1 itself is silent on timeout; this is a belt-and-braces safety net for the field. Default 0 = disabled (spec-pure: FBHD sits in SUB_GATE indefinitely waiting for V_gps / h_lidar / dh/dt to gate-pass). For real flight, the pilot is expected to drop back to HULL_BORNE or trigger AUTO_DESCEND manually if take-off fails. Recommend 30000 (30 s) for a hands-off field demo; W5 SITL keeps default 0 so AR_FoilControl's separate saturation-driven failsafe can be characterised in isolation.
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("_GATE_TMO", 7, ModeFoilborneHold, _gate_tmo_ms,     0),

    // @Param: _RAMP_R
    // @DisplayName: FBHD h_cmd ramp rate
    // @Description: Rate (m/s) at which h_cmd ramps from h_lidar(t_gate) → FBHD_H_REF during the SUB_RAMP sub-state. Spec text "ramp to 0.15 m at 0.30 m/s" gives 0.30 m/s default; at this rate a 0.15 m ramp finishes in 0.5 s.
    // @Units: m/s
    // @Range: 0.05 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_RAMP_R",   8, ModeFoilborneHold, _ramp_rate_ms,    0.30f),

    AP_GROUPEND
};

ModeFoilborneHold::ModeFoilborneHold() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// --- internal helpers -------------------------------------------------------

void ModeFoilborneHold::reset_entry_state()
{
    _entry_phase = EntryPhase::GATE;
    _entry_t_ms = AP_HAL::millis();
    _gate_hold_start_ms = 0;
    _ramp_start_ms = 0;
    _ramp_h0_m = 0.0f;
    _ramp_h_target_m = _h_ref_m;
    _h_last_m = NAN;
    _h_last_us = 0;
    _dhdt_filt_ms = 0.0f;
}

void ModeFoilborneHold::update_dhdt(float h_m)
{
    const uint32_t now_us = AP_HAL::micros();
    if (!isfinite(h_m)) {
        // LIDAR drop: hold the last filtered estimate.  This is conservative
        // (likely the boat is splashed and the rangefinder is reading the
        // hull bottom — dh/dt is not meaningful here), so the gate cannot
        // fire on stale data.
        return;
    }
    if (!isfinite(_h_last_m) || _h_last_us == 0) {
        _h_last_m = h_m;
        _h_last_us = now_us;
        _dhdt_filt_ms = 0.0f;
        return;
    }
    float dt = (now_us - _h_last_us) * 1.0e-6f;
    if (dt < 1e-4f) {
        return;  // duplicate read; skip
    }
    // Clamp dt so a long pause (e.g. between SITL ticks at 50 Hz mode loop)
    // doesn't produce a noise-spike in the finite-diff.
    dt = constrain_float(dt, 1e-4f, 0.2f);
    const float dh_raw = (h_m - _h_last_m) / dt;
    // 1st-order LP: y += dt/(τ+dt) * (x - y).
    const float alpha = dt / (DHDT_LP_TAU_S + dt);
    _dhdt_filt_ms += alpha * (dh_raw - _dhdt_filt_ms);
    _h_last_m = h_m;
    _h_last_us = now_us;
}

bool ModeFoilborneHold::gate_conditions_met(float v_gps_ms, float h_m, float dhdt_ms,
                                            float theta_rad, float phi_rad) const
{
    if (!isfinite(h_m)) {
        return false;
    }
    if (v_gps_ms < _v_gate_ms.get()) {
        return false;
    }
    if (h_m < _h_gate_m.get()) {
        return false;
    }
    if (dhdt_ms < _dhdt_gate_ms.get()) {
        return false;
    }
    if (fabsf(theta_rad) > ATTITUDE_GATE_RAD) {
        return false;
    }
    if (fabsf(phi_rad) > ATTITUDE_GATE_RAD) {
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------------

bool ModeFoilborneHold::_enter()
{
    // PR8 §3.1: start in SUB_GATE.  h_cmd is set to current h each tick in
    // SUB_GATE (zero-error on the height-PID; no wind-up) so the boat can
    // accelerate to V_TO without the cascade fighting an impossible 0.15 m
    // target.  V_cmd stays at 1.9 m/s — that's how the boat actually crosses
    // the foilborne threshold; the gate just delays the *height* command
    // until the climb has begun.
    reset_entry_state();

    // Initial setpoints: V at 1.9 (drives V_TO transition), h at current h
    // (zero-error for the height-PID — see update() for the per-tick value),
    // heading captured here so we don't chase yaw.
    rover.g2.foil_control.set_speed_target(1.9f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    if (!is_zero(_th_override_rad)) {
        rover.g2.foil_control.set_pitch_target_rad(_th_override_rad);
    } else {
        rover.g2.foil_control.clear_pitch_target_override();
    }
    // PR6 §5: reset the failsafe-gate's mode-settle timer so the 2 s arm
    // delay starts from this boundary, not from boot.  Also wipes the duty
    // ring buffer per AR_FoilControl::notify_mode_change() (PR7a-D3).
    rover.g2.foil_control.notify_mode_change();
    return true;
}

void ModeFoilborneHold::update()
{
    // PR7a D1: failsafe-descend handover (unchanged).  AR_FoilControl raises
    // this latch when the duty-cycle saturation trigger has held under the
    // armed gate for FOIL_FAIL_DWELL ms.  notify_mode_change() in
    // AutoDescend::_enter wipes the latch + dwell + duty buffer so the
    // failsafe state machine starts fresh in the descent mode.
    if (rover.g2.foil_control.should_failsafe_descend()) {
        rover.set_mode(rover.mode_auto_descend, ModeReason::FAILSAFE);
        return;
    }

    // --- §3.1 sub-state machine ---------------------------------------------
    const uint32_t now_ms = AP_HAL::millis();
    const float    h_meas = rover.g2.foil_control.get_height_above_water();
    update_dhdt(h_meas);

    const float v_gps    = AP::ahrs().groundspeed();
    const float theta    = AP::ahrs().get_pitch_rad();
    const float phi      = AP::ahrs().get_roll_rad();

    float h_cmd = _h_ref_m;  // overwritten per phase below

    switch (_entry_phase) {

    case EntryPhase::GATE: {
        // Hold h_cmd at current h so the height-PID has zero error.  If
        // LIDAR is dropping (NaN), fall back to FBHD_H_REF — the PID's
        // last-good θ_cmd hold path in AR_FoilControl handles the NaN case
        // safely.
        h_cmd = isfinite(h_meas) ? h_meas : _h_ref_m.get();

        const bool gate_ok = gate_conditions_met(v_gps, h_meas, _dhdt_filt_ms, theta, phi);
        if (gate_ok) {
            if (_gate_hold_start_ms == 0) {
                _gate_hold_start_ms = now_ms;
            }
            if ((now_ms - _gate_hold_start_ms) >= uint32_t(_gate_hold_ms.get())) {
                // Promote to SUB_RAMP.  Capture current h as the ramp start.
                _entry_phase   = EntryPhase::RAMP;
                _ramp_start_ms = now_ms;
                _ramp_h0_m     = isfinite(h_meas) ? h_meas : _h_gate_m.get();
                _ramp_h_target_m = _h_ref_m.get();
                // After advancing, drop through so this tick already issues
                // the ramp setpoint (avoids a one-tick stale h_cmd).
            }
        } else {
            // Any sub-condition that fails resets the hold timer — the gate
            // must hold ALL conditions for the full FBHD_GATE_HLD.
            _gate_hold_start_ms = 0;
        }

        // Gate-timeout fallback — drop to AUTO_DESCEND if the entry gate
        // never fires.  This is a belt-and-braces safety net (not in
        // §3.1 itself; spec is silent on timeout) so a stuck speed loop
        // or a dead throttle doesn't strand the boat at half-throttle
        // forever.  FBHD_GATE_TMO default is large (30 s sim) so that
        // (a) the SITL W5 step-response harness, which sleeps 2.5 s wall
        // = 25 s sim before injection, has time to either pass the gate
        // or run its hostile-θ failsafe-saturation test, and (b) any
        // real-flight take-off has full headroom (real wall-time =
        // sim-time on hardware).  Skipped if we just promoted to RAMP.
        if (_entry_phase == EntryPhase::GATE &&
            _gate_tmo_ms.get() > 0 &&
            (now_ms - _entry_t_ms) >= uint32_t(_gate_tmo_ms.get())) {
            rover.set_mode(rover.mode_auto_descend, ModeReason::FAILSAFE);
            return;
        }
        break;
    }

    case EntryPhase::RAMP: {
        // h_cmd = h0 + ramp_rate * (now - t_ramp_start), clamped to
        // FBHD_H_REF.  Sign-aware: if FBHD_H_REF < h0 (unlikely but
        // possible after a runtime PARAM_SET), ramp downward at the
        // same rate magnitude.
        const float t_ramp_s = (now_ms - _ramp_start_ms) * 1.0e-3f;
        const float h_target = _ramp_h_target_m;  // captured at gate-pass — see note below
        const float sgn = (h_target >= _ramp_h0_m) ? 1.0f : -1.0f;
        const float h_unclamped = _ramp_h0_m + sgn * _ramp_rate_ms.get() * t_ramp_s;
        if (sgn > 0.0f) {
            h_cmd = MIN(h_unclamped, h_target);
            if (h_cmd >= h_target - 1e-4f) {
                _entry_phase = EntryPhase::HOLD;
            }
        } else {
            h_cmd = MAX(h_unclamped, h_target);
            if (h_cmd <= h_target + 1e-4f) {
                _entry_phase = EntryPhase::HOLD;
            }
        }
        // Note on _ramp_h_target_m capture: we snapshot FBHD_H_REF at the
        // gate-pass moment so a runtime PARAM_SET of FBHD_H_REF *during*
        // the ramp doesn't shift the moving target underfoot.  Once the
        // ramp completes (SUB_HOLD), `_h_ref_m.get()` is read per-tick
        // again, so step-response injection works as before.
        break;
    }

    case EntryPhase::HOLD:
    default:
        h_cmd = _h_ref_m.get();
        break;
    }

    rover.g2.foil_control.set_height_target(h_cmd);
    rover.g2.foil_control.set_speed_target(1.9f);
    // Pitch override: when FBHD_TH_OV is non-zero, bypass the height->pitch
    // cascade and force theta_cmd directly.  When zero, restore normal
    // cascaded operation (idempotent if already cleared).  Honoured in every
    // sub-state so the W5 θ-step harness works during GATE/RAMP too — the
    // test inserts a 5 s pre-roll which carries us through to HOLD before
    // the injection, but the override is supposed to be a hard bypass.
    if (!is_zero(_th_override_rad)) {
        rover.g2.foil_control.set_pitch_target_rad(_th_override_rad);
    } else {
        rover.g2.foil_control.clear_pitch_target_override();
    }
    // Heading target is NOT re-captured every tick — leave it at the entry
    // value so the boat actually holds heading instead of always pointing
    // wherever it just yawed to.

    // Pipe the speed-loop throttle (0..1) into AP_MotorsUGV (-100..+100).
    g2.motors.set_throttle(rover.g2.foil_control.get_throttle_cmd() * 100.0f);
    // _steering is left zero so AP_MotorsUGV::output_regular() doesn't write
    // k_steering — the foilborne rudder comes from AR_FoilControl on k_foilcontrol_rudder.
    g2.motors.set_steering(0.0f);
}
