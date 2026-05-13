#include "Rover.h"

// AUTO_DESCEND: monotone-down ride-height schedule, transitions to HULL_BORNE
// once height-above-water touches the threshold.  PR5 implements the height
// ramp (0.15 → 0 m over 3 s) and throttle taper per 04-control-law.md §3.3.

// Ramp config — linear, time-parameterised.  3 s nominal window (mid-range of
// the spec's 2–4 s).  Throttle is tapered to 0.10 over the same window, then
// 0 once h < 0.03 m (the latter check moves to the failsafe SM in PR6).
static constexpr float AUTO_DESCEND_RAMP_S       = 3.0f;
static constexpr float AUTO_DESCEND_H_START      = 0.15f;
static constexpr float AUTO_DESCEND_H_END        = 0.00f;
static constexpr float AUTO_DESCEND_THR_END      = 0.10f;
static constexpr float AUTO_DESCEND_H_CUTOFF_M   = 0.03f;

bool ModeAutoDescend::_enter()
{
    // Snapshot current targets and the start-of-descent throttle.
    rover.g2.foil_control.set_height_target(AUTO_DESCEND_H_START);
    rover.g2.foil_control.set_speed_target(0.5f);
    rover.g2.foil_control.set_heading_target_rad(AP::ahrs().get_yaw_rad());
    rover.g2.foil_control.clear_pitch_target_override();
    rover.g2.foil_control.notify_mode_change();
    return true;
}

void ModeAutoDescend::update()
{
    // Time-since-entry — Mode::enter() doesn't expose this, so derive it from
    // a static last-h target.  When the height target equals AUTO_DESCEND_H_END
    // we assume the ramp completed.  This is good enough for v0 SITL; a
    // proper entry-time member lands with the failsafe SM in PR6.
    static uint32_t entry_ms = 0;
    if (entry_ms == 0 || rover.g2.foil_control.get_throttle_cmd() <= 0.0f) {
        // (Re)init on first tick after entry.
        entry_ms = AP_HAL::millis();
    }
    const float t = (AP_HAL::millis() - entry_ms) * 1.0e-3f;
    const float u = constrain_float(t / AUTO_DESCEND_RAMP_S, 0.0f, 1.0f);

    const float h_target = AUTO_DESCEND_H_START + (AUTO_DESCEND_H_END - AUTO_DESCEND_H_START) * u;
    rover.g2.foil_control.set_height_target(h_target);
    // Speed-loop target rides the height down: V_TO * (1 - u) keeps the boat
    // bleeding speed monotonically so the foils unload cleanly.
    rover.g2.foil_control.set_speed_target(1.6f * (1.0f - u));

    // Throttle taper: linear from whatever_we_had → 0.10 over the same window,
    // then 0 once height has fallen below 3 cm.  We mix the speed-PID output
    // with the open-loop taper by taking the *minimum* — guarantees a
    // monotonic descent even if the speed loop tries to push back.
    const float h_meas = rover.g2.foil_control.get_height_above_water();
    float throttle_open_loop = AUTO_DESCEND_THR_END + (1.0f - AUTO_DESCEND_THR_END) * (1.0f - u);
    if (isfinite(h_meas) && h_meas < AUTO_DESCEND_H_CUTOFF_M) {
        throttle_open_loop = 0.0f;
    }
    const float throttle_pid = rover.g2.foil_control.get_throttle_cmd();
    const float throttle = MIN(throttle_pid, throttle_open_loop);
    g2.motors.set_throttle(throttle * 100.0f);
    g2.motors.set_steering(0.0f);
}
