/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AR_FoilControl.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/rotations.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <cmath>

// --- defaults ---------------------------------------------------------------

// Scalar thresholds (per 05-ardufoil-integration.md §5).
#define AR_FOILCONTROL_HGT_TARGET       0.10f
#define AR_FOILCONTROL_V_TAKEOFF        1.8f
#define AR_FOILCONTROL_HGT_FB_THRESH    0.06f

// Height outer loop defaults (04-control-law.md §1.4).
// Kp = 0.35, Ki = 0.08, Kd = 0.20  (Kd on vertical rate ż, not on dh/dt).
#define AR_FOILCONTROL_HGT_P            0.35f
#define AR_FOILCONTROL_HGT_I            0.08f
#define AR_FOILCONTROL_HGT_D            0.20f

// Inner body-rate PID defaults (PR4 baseline — V^2 scheduled at runtime).
//   q-axis (pitch rate): Kp=0.19, Ki=0.02, Kd=0.04  (PR6: Kd_θ folded in here)
//   p-axis (roll  rate): Kp=0.04, Ki=0.005, Kd=0
//   r-axis (yaw   rate): Kp=1.2,  Ki=0.3,  Kd=0
//
// Kd_q default (0.04) is the analytical estimate from closed-loop ω_n=30 rad/s,
// ζ=0.7 design intent for the pitch cascade (Kd_outer = 2ζω/G then mapped onto
// the rate loop's plant gain). The codebase expert flagged two derivations that
// disagree (0.009 vs 0.04); SITL closed-loop step-response sweep across
// {0.01, 0.02, 0.04, 0.08} is queued behind plant v0.2. Until that sweep
// runs, 0.04 is the conservative default — high enough to add damping, low
// enough that the rate loop stays causal at 400 Hz with a 20 Hz D-filter.
#define AR_FOILCONTROL_Q_RATE_P         0.19f
#define AR_FOILCONTROL_Q_RATE_I         0.02f
#define AR_FOILCONTROL_Q_RATE_D         0.04f
#define AR_FOILCONTROL_Q_RATE_FF        0.00f
#define AR_FOILCONTROL_Q_RATE_IMAX      0.20f

#define AR_FOILCONTROL_P_RATE_P         0.04f
#define AR_FOILCONTROL_P_RATE_I         0.005f
#define AR_FOILCONTROL_P_RATE_D         0.00f
#define AR_FOILCONTROL_P_RATE_FF        0.00f
#define AR_FOILCONTROL_P_RATE_IMAX      0.20f

#define AR_FOILCONTROL_R_RATE_P         1.20f
#define AR_FOILCONTROL_R_RATE_I         0.30f
#define AR_FOILCONTROL_R_RATE_D         0.00f
#define AR_FOILCONTROL_R_RATE_FF        0.00f
#define AR_FOILCONTROL_R_RATE_IMAX      0.40f

// Pitch attitude PID (§1.1).  PR6: Kd_θ moved onto the inner pitch-rate PID
// (canonical AC_PID idiom), so the outer pitch wrapper has Kd=0 here.
// Ki=0.5 stays in place as the §2.2 lift-trim integrator hook.
#define AR_FOILCONTROL_THETA_P          4.00f
#define AR_FOILCONTROL_THETA_I          0.50f
#define AR_FOILCONTROL_THETA_D          0.00f
#define AR_FOILCONTROL_THETA_FF         0.00f
#define AR_FOILCONTROL_THETA_IMAX       2.00f

// Roll attitude PID (§1.2).  Kd_φ=0.04 is on dφ/dt, which is what AC_PID's
// D-on-error path computes (target φ_cmd is constant in v0, so dφ/dt = -de/dt
// up to sign and AC_PID's D filter cleans it).
#define AR_FOILCONTROL_PHI_P            0.60f
#define AR_FOILCONTROL_PHI_I            0.05f
#define AR_FOILCONTROL_PHI_D            0.04f
#define AR_FOILCONTROL_PHI_FF           0.00f
#define AR_FOILCONTROL_PHI_IMAX         0.50f

// Heading P controller (§1.3 first-cut: Kp=1.0).
// PR6: AC_P (no I, no D) — matches AR_AttitudeControl::_steer_angle_p.
#define AR_FOILCONTROL_PSI_P            1.00f

// Speed PID (§1.5).  Output is in 0..1 throttle fraction.
#define AR_FOILCONTROL_V_P              0.35f
#define AR_FOILCONTROL_V_I              0.10f
#define AR_FOILCONTROL_V_D              0.00f
#define AR_FOILCONTROL_V_FF             0.00f
#define AR_FOILCONTROL_V_IMAX           1.00f

// Filter cutoffs.
#define AR_FOILCONTROL_RATE_FILT_T_HZ   0.00f
#define AR_FOILCONTROL_RATE_FILT_E_HZ   0.00f
#define AR_FOILCONTROL_RATE_FILT_D_HZ   20.0f
#define AR_FOILCONTROL_ATT_FILT_T_HZ    0.00f
#define AR_FOILCONTROL_ATT_FILT_E_HZ    0.00f
#define AR_FOILCONTROL_ATT_FILT_D_HZ    10.0f

// V^2 gain scheduling defaults (§1.1).
#define AR_FOILCONTROL_VCRUISE          3.7f
#define AR_FOILCONTROL_VMIN_SCHED       0.5f

// Pre-load schedule defaults (PR6, §4 revised by controls expert).
// V_TO 1.6 m/s, canard peak +4° (was +6°: leaves 2° margin to α_stall at
// V_TO when canard rigging +3° + θ at take-off +3° + pre-load 4° = 10° AoA),
// main peak +2°. Schedule shape: canard quadratic in V/V_TO, main linear.
#define AR_FOILCONTROL_VTO              1.6f
#define AR_FOILCONTROL_PRE_CAN_RAD      0.0698132f   // radians(4.0)
#define AR_FOILCONTROL_PRE_MAIN_RAD     0.0349066f   // radians(2.0)

// V² scheduling floor (PR6, §3 piecewise). Applied above V_cruise so the
// schedule doesn't drop below 0.25× the nominal gain — high-speed plant has
// plenty of authority but we still want a finite controller bandwidth.
#define AR_FOILCONTROL_SCHED_FLOOR      0.25f

// Canard lead compensator (PR6, §4). H(s) = (1 + 0.060 s) / (1 + 0.015 s).
// Zero at 16.7 rad/s (≈ servo pole 1/τ_servo at 55 ms = 18 rad/s),
// pole at 66.7 rad/s (≈ 4× the servo pole). Restores ~35° PM at ω∈[25,35]
// after the servo lag drops it from ~80° to ~22°.
#define AR_FOILCONTROL_LEAD_LD          0.060f
#define AR_FOILCONTROL_LEAD_LG          0.015f
#define AR_FOILCONTROL_LEAD_EN          1

// pilot stick scale: full deflection maps to ±2 rad/s body-rate setpoint.
// (Still used for HULL_BORNE / pilot inputs in lieu of an attitude command.)
#define AR_FOILCONTROL_PILOT_RATE_SCALE 2.0f

// Mixer constants per 04-control-law.md §2.1.
#define AR_FOILCONTROL_MIX_K_PC          1.00f
#define AR_FOILCONTROL_MIX_K_PM          0.40f

// §5.1 mechanical limit on each flap channel (±25° = ±0.436 rad).
#define AR_FOILCONTROL_FLAP_LIMIT_RAD    0.436f
// §5.4 differential roll headroom (±15° = ±0.262 rad).
#define AR_FOILCONTROL_ROLL_DIFF_LIMIT_RAD 0.262f
// Height-outer pitch-cmd clamp (±6° = ±0.105 rad) per §1.4.
#define AR_FOILCONTROL_THETA_CMD_LIMIT_RAD 0.105f
// §5.3 cascade dwell threshold before height integrator freezes.
#define AR_FOILCONTROL_PITCH_SAT_DWELL_S   0.20f
// PR6 §5: continuous-saturation latch dwell (must hold for this long with
// _sat_trigger_armed=true before AUTO_DESCEND is requested).
#define AR_FOILCONTROL_CONT_SAT_LATCH_S    0.50f
// PR6 §5: settle window after a mode change before failsafe arms.
#define AR_FOILCONTROL_MODE_SETTLE_S       2.0f

// PR7a D3: duty-cycle saturation trigger defaults.
// Threshold (0..1): >40% saturated samples over the 5 s window arms the trigger.
// Window length is the SAT_WINDOW_SAMPLES compile-time constant in the header
// (2000 samples = 5 s @ 400 Hz inner-loop tick).  FOIL_SAT_WIN_MS is documented
// here as a static value, not exposed as a param (changing it would require a
// buffer realloc that we don't do at runtime).
#define AR_FOILCONTROL_SAT_DUTY_THR        0.40f
#define AR_FOILCONTROL_SAT_WIN_MS          5000     // documented constant; matches SAT_WINDOW_SAMPLES @ 400 Hz
// Margin (rad) inside the mechanical flap limit at which we declare a surface
// "at limit" for duty-cycle accounting.  Anything within 0.5° (0.00873 rad)
// of the ±25° clamp counts as saturated.
#define AR_FOILCONTROL_SAT_EPSILON_RAD     0.00873f

// PR7a D1: failsafe dwell before AUTO_DESCEND is requested (ms).
// Counts up while _sat_trigger_armed is set inside update_failsafe(); when it
// exceeds this dwell, _failsafe_descend_request latches true and the next
// Rover mode tick swaps the mode via rover.set_mode(AUTO_DESCEND).
#define AR_FOILCONTROL_FAIL_DWELL_MS       500


// SRV_Channels::set_angle takes uint16_t centidegree half-range.
#define AR_FOILCONTROL_SRV_ANGLE_CD      2500

AR_FoilControl *AR_FoilControl::_singleton;

const AP_Param::GroupInfo AR_FoilControl::var_info[] = {

    // @Group: P_RAT_
    // @Path: ../AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(_p_rate_pid, "P_RAT_", 1, AR_FoilControl, AC_PID),

    // @Group: Q_RAT_
    // @Path: ../AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(_q_rate_pid, "Q_RAT_", 2, AR_FoilControl, AC_PID),

    // @Group: R_RAT_
    // @Path: ../AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(_r_rate_pid, "R_RAT_", 3, AR_FoilControl, AC_PID),

    // @Group: ROL_
    // @Path: ../AC_PID/AC_PID.cpp
    // Roll attitude PID (PR5) — phi_cmd → p_setpoint.
    AP_SUBGROUPINFO(_phi_pid, "ROL_", 4, AR_FoilControl, AC_PID),

    // @Group: SPD_
    // @Path: ../AC_PID/AC_PID.cpp
    // Forward-speed PID (PR5) — V_cmd → throttle [0..1].
    AP_SUBGROUPINFO(_v_pid, "SPD_", 5, AR_FoilControl, AC_PID),

    // @Param: HGT_P
    // @DisplayName: Foil height control P gain
    // @Description: Outer-loop height-error to pitch-cmd P gain (rad/m)
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_P",       6, AR_FoilControl, _h_p, AR_FOILCONTROL_HGT_P),

    // @Param: HGT_I
    // @DisplayName: Foil height control I gain
    // @Description: Outer-loop height-error to pitch-cmd I gain (rad/(m·s))
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_I",       7, AR_FoilControl, _h_i, AR_FOILCONTROL_HGT_I),

    // @Param: HGT_D
    // @DisplayName: Foil height control D gain
    // @Description: Outer-loop height-error to pitch-cmd D gain on vertical rate (rad·s/m)
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_D",       8, AR_FoilControl, _h_d, AR_FOILCONTROL_HGT_D),

    // @Param: HGT_TGT
    // @DisplayName: Foilborne target ride height
    // @Description: Desired height above water in foilborne hold (m)
    // @Units: m
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_TGT",     9, AR_FoilControl, _h_target, AR_FOILCONTROL_HGT_TARGET),

    // @Param: VTOFF
    // @DisplayName: Foilborne takeoff speed
    // @Description: Forward speed at which foilborne transition is permitted (m/s)
    // @Units: m/s
    // @Range: 0.0 20.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("VTOFF",      10, AR_FoilControl, _v_takeoff, AR_FOILCONTROL_V_TAKEOFF),

    // @Param: HGT_FB
    // @DisplayName: Foilborne height threshold
    // @Description: Minimum height above water to consider vehicle foilborne (m)
    // @Units: m
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_FB",     11, AR_FoilControl, _h_foilborne_thresh, AR_FOILCONTROL_HGT_FB_THRESH),

    // @Group: PIT_
    // @Path: ../AC_PID/AC_PID.cpp
    // Pitch attitude PID (PR5) — theta_cmd → q_setpoint.
    // PR6: Kd_θ moved to FOIL_Q_RAT_D on the inner pitch-rate PID (canonical
    // AC_PID idiom). FOIL_KD_THETA is removed; FOIL_PIT_D remains 0.
    AP_SUBGROUPINFO(_theta_pid, "PIT_", 12, AR_FoilControl, AC_PID),

    // @Group: YAW_
    // @Path: ../AC_PID/AC_P.cpp
    // Heading P controller (PR5/PR6) — psi_cmd → r_setpoint, first-cut Kp=1.0.
    // PR6: refactored from AC_PID to AC_P (Ki_ψ=Kd_ψ=0 per spec). Slot index
    // 13 reused so FOIL_YAW_P retains the same key; FOIL_YAW_{I,D,IMAX} are
    // dropped (they didn't have an effect in PR5 anyway — Ki/Kd were 0).
    AP_SUBGROUPINFO(_psi_p, "YAW_", 13, AR_FoilControl, AC_P),

    // @Param: VCRUISE
    // @DisplayName: V^2 scheduling cruise speed
    // @Description: Reference cruise speed for V^2 gain scheduling (m/s).
    // Pitch/roll attitude + rate-loop gains are multiplied by piecewise V^2
    // schedule centred on this speed (see FOIL_SCHED_FL for the floor).
    // @Units: m/s
    // @Range: 0.5 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VCRUISE",    14, AR_FoilControl, _vcruise, AR_FOILCONTROL_VCRUISE),

    // @Param: VMINSCHD
    // @DisplayName: V^2 scheduling minimum speed
    // @Description: Lower clamp on V used by the V^2 schedule (m/s). Prevents
    // 1/V^2 blow-up at rest. Must be > 0.
    // @Units: m/s
    // @Range: 0.1 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VMINSCHD",   15, AR_FoilControl, _vmin_sched, AR_FOILCONTROL_VMIN_SCHED),

    // Slot 16: previously FOIL_KD_THETA (PR5 manual D-on-q term).
    // PR6: removed — Kd_θ is now FOIL_Q_RAT_D (the inner pitch-rate PID's D
    // gain). Slot key intentionally left unused so a later PR can repurpose
    // it without colliding with cached EEPROM keys on existing field rigs.

    // @Param: VTO
    // @DisplayName: Foilborne take-off reference speed
    // @Description: V_TO used by the pre-load schedule (rad/(m/s)) — canard
    // pre-load peaks at V=VTO (quadratic), main at V=VTO (linear). Also used
    // as the freeze threshold for the V² gain schedule.
    // @Units: m/s
    // @Range: 0.5 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VTO",        17, AR_FoilControl, _vto, AR_FOILCONTROL_VTO),

    // @Param: PRE_CAN
    // @DisplayName: Canard pre-load peak
    // @Description: Canard flap deflection at V=V_TO before foilborne engage
    // (rad). Positive = trailing-edge-down = lift up. Quadratic in V/V_TO.
    // Default 0.0698 rad (4°) leaves 2° margin to α_stall at V_TO.
    // @Units: rad
    // @Range: 0.0 0.175
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PRE_CAN",    18, AR_FoilControl, _pre_can, AR_FOILCONTROL_PRE_CAN_RAD),

    // @Param: PRE_MAIN
    // @DisplayName: Main pre-load peak
    // @Description: Main flap deflection at V=V_TO before foilborne engage
    // (rad). Linear in V/V_TO. Default 0.0349 rad (2°).
    // @Units: rad
    // @Range: 0.0 0.175
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("PRE_MAIN",   19, AR_FoilControl, _pre_main, AR_FOILCONTROL_PRE_MAIN_RAD),

    // @Param: SCHED_FL
    // @DisplayName: V² schedule floor
    // @Description: Lower bound on the V² gain scale applied above V_cruise.
    // Without this, scale → 0 at very high V, dropping controller bandwidth
    // unacceptably. Default 0.25 (controller runs at 25% of nominal at high V).
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("SCHED_FL",   20, AR_FoilControl, _sched_floor, AR_FOILCONTROL_SCHED_FLOOR),

    // @Param: LEAD_LD
    // @DisplayName: Canard lead compensator lead time constant
    // @Description: Lead time constant of the canard lead-lag compensator
    // H(s)=(1+τ_lead·s)/(1+τ_lag·s) applied downstream of the mixer. Default
    // 0.060 s places the zero at ~17 rad/s (just below the servo pole).
    // @Units: s
    // @Range: 0.0 0.5
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("LEAD_LD",    21, AR_FoilControl, _lead_tau_lead, AR_FOILCONTROL_LEAD_LD),

    // @Param: LEAD_LG
    // @DisplayName: Canard lead compensator lag time constant
    // @Description: Lag time constant of the canard lead-lag compensator.
    // Default 0.015 s places the pole at ~67 rad/s (~4× the servo pole).
    // @Units: s
    // @Range: 0.0 0.5
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("LEAD_LG",    22, AR_FoilControl, _lead_tau_lag, AR_FOILCONTROL_LEAD_LG),

    // @Param: LEAD_EN
    // @DisplayName: Canard lead compensator enable
    // @Description: Enable (1) / bypass (0) the canard lead compensator. For
    // A/B testing the PM-recovery delta.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LEAD_EN",    23, AR_FoilControl, _lead_en, AR_FOILCONTROL_LEAD_EN),

    // @Param: SAT_DUTY
    // @DisplayName: Saturation duty-cycle trigger threshold
    // @Description: Fraction of inner-loop ticks (0..1) over a 5 s window in
    // which any flap surface is within ~0.5 deg of its mechanical limit. When
    // exceeded under the failsafe arming gate (foilborne + settled), the
    // duty-cycle trigger fires; sustained for FOIL_FAIL_DWELL ms it demands
    // AUTO_DESCEND.  Default 0.40 (40 %).
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Advanced
    // PR7b: shortened from SAT_DUTY_THR to SAT_DUTY — FOIL_SAT_DUTY_THR was
    // 17 chars total, exceeding the AP_Param 16-char name limit, panicking
    // ardurover at boot with "Bad parameter table" (ENABLE_DEBUG=1 message:
    // "suffix is too long in SAT_DUTY_THR (17 > 16)").  Renaming the suffix
    // avoids the boot panic; the AP_GROUPINFO slot index (24) is preserved
    // so any stored param value migrates cleanly across the rename.
    AP_GROUPINFO("SAT_DUTY", 24, AR_FoilControl, _sat_duty_thr, AR_FOILCONTROL_SAT_DUTY_THR),

    // @Param: FAIL_DWELL
    // @DisplayName: Failsafe AUTO_DESCEND dwell
    // @Description: Time the duty-cycle saturation trigger must hold under the
    // failsafe arming gate before _failsafe_descend_request latches true.
    // Rover modes poll should_failsafe_descend() and call
    // rover.set_mode(AUTO_DESCEND, FAILSAFE) on the next tick.
    // @Units: ms
    // @Range: 100 2000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("FAIL_DWELL",  25, AR_FoilControl, _fail_dwell_ms, AR_FOILCONTROL_FAIL_DWELL_MS),

    AP_GROUPEND
};

AR_FoilControl::AR_FoilControl() :
    _p_rate_pid(AR_FOILCONTROL_P_RATE_P, AR_FOILCONTROL_P_RATE_I, AR_FOILCONTROL_P_RATE_D,
                AR_FOILCONTROL_P_RATE_FF, AR_FOILCONTROL_P_RATE_IMAX,
                AR_FOILCONTROL_RATE_FILT_T_HZ, AR_FOILCONTROL_RATE_FILT_E_HZ, AR_FOILCONTROL_RATE_FILT_D_HZ),
    _q_rate_pid(AR_FOILCONTROL_Q_RATE_P, AR_FOILCONTROL_Q_RATE_I, AR_FOILCONTROL_Q_RATE_D,
                AR_FOILCONTROL_Q_RATE_FF, AR_FOILCONTROL_Q_RATE_IMAX,
                AR_FOILCONTROL_RATE_FILT_T_HZ, AR_FOILCONTROL_RATE_FILT_E_HZ, AR_FOILCONTROL_RATE_FILT_D_HZ),
    _r_rate_pid(AR_FOILCONTROL_R_RATE_P, AR_FOILCONTROL_R_RATE_I, AR_FOILCONTROL_R_RATE_D,
                AR_FOILCONTROL_R_RATE_FF, AR_FOILCONTROL_R_RATE_IMAX,
                AR_FOILCONTROL_RATE_FILT_T_HZ, AR_FOILCONTROL_RATE_FILT_E_HZ, AR_FOILCONTROL_RATE_FILT_D_HZ),
    _phi_pid(AR_FOILCONTROL_PHI_P, AR_FOILCONTROL_PHI_I, AR_FOILCONTROL_PHI_D,
             AR_FOILCONTROL_PHI_FF, AR_FOILCONTROL_PHI_IMAX,
             AR_FOILCONTROL_ATT_FILT_T_HZ, AR_FOILCONTROL_ATT_FILT_E_HZ, AR_FOILCONTROL_ATT_FILT_D_HZ),
    _v_pid(AR_FOILCONTROL_V_P, AR_FOILCONTROL_V_I, AR_FOILCONTROL_V_D,
           AR_FOILCONTROL_V_FF, AR_FOILCONTROL_V_IMAX,
           AR_FOILCONTROL_ATT_FILT_T_HZ, AR_FOILCONTROL_ATT_FILT_E_HZ, AR_FOILCONTROL_ATT_FILT_D_HZ),
    _theta_pid(AR_FOILCONTROL_THETA_P, AR_FOILCONTROL_THETA_I, AR_FOILCONTROL_THETA_D,
               AR_FOILCONTROL_THETA_FF, AR_FOILCONTROL_THETA_IMAX,
               AR_FOILCONTROL_ATT_FILT_T_HZ, AR_FOILCONTROL_ATT_FILT_E_HZ, AR_FOILCONTROL_ATT_FILT_D_HZ),
    _psi_p(AR_FOILCONTROL_PSI_P),
    _h_integrator(0.0f),
    _h_last_theta_cmd(0.0f),
    _height_target_m(AR_FOILCONTROL_HGT_TARGET),
    _speed_target_ms(0.0f),
    _heading_target_rad(0.0f),
    _pitch_target_override(false),
    _pitch_target_override_rad(0.0f),
    _canard_cmd_rad(0.0f),
    _main_cmd_rad(0.0f),
    _rudder_cmd_rad(0.0f),
    _throttle_cmd(0.0f),
    _theta_cmd_rad(0.0f),
    _p_setpoint_rad_s(0.0f),
    _q_setpoint_rad_s(0.0f),
    _r_setpoint_rad_s(0.0f),
    _nom_p_rate_kp(0), _nom_p_rate_ki(0), _nom_p_rate_kd(0),
    _nom_q_rate_kp(0), _nom_q_rate_ki(0), _nom_q_rate_kd(0),
    _nom_r_rate_kp(0), _nom_r_rate_ki(0), _nom_r_rate_kd(0),
    _nom_phi_kp(0),    _nom_phi_ki(0),    _nom_phi_kd(0),
    _nom_theta_kp(0),  _nom_theta_ki(0),  _nom_theta_kd(0),
    _nom_psi_kp(0),
    _nominal_gains_captured(false),
    _last_gain_scale(1.0f),
    _pitch_sat_dwell_s(0.0f),
    _pitch_saturated(false),
    _foilborne_now(false),
    _foilborne_engage_s(0.0f),
    _engage_factor(0.0f),
    _canard_preload_last(0.0f),
    _lead_x_prev(0.0f),
    _lead_y_prev(0.0f),
    _sat_armed(false),
    _sat_trigger_armed(false),
    _continuous_sat_s(0.0f),
    _t_since_mode_change_s(0.0f),
    _last_mode_num_seen(0xff),
    _sat_buf_idx(0),
    _sat_buf_fill(0),
    _sat_buf_sum(0),
    _sat_duty_cycle(0.0f),
    _failsafe_dwell_ms(0),
    _failsafe_descend_request(false),
    _failsafe_event_pending(false),
    _last_inner_us(0),
    _last_outer_us(0),
    _last_throttle_us(0),
    _last_failsafe_us(0),
    _last_inner_dt(0.0025f),
    // PR14a D3: matrix-detector snapshot state — zero-init for first tick.
    _last_height_err_m(0.0f),
    _att_err_lpf_rad(0.0f),
    _h_integrator_clamp_dwell_s(0.0f),
    _height_pid_winding(false),
    _servo_ranges_set(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    // PR7a D3: zero-init the 2000-sample duty-cycle ring buffer.
    for (uint16_t i = 0; i < SAT_WINDOW_SAMPLES; i++) {
        _sat_buf[i] = 0;
    }
}

// PR7a D3 + D1: explicit out-of-line notify_mode_change.  Wipes the
// duty-cycle ring buffer (D3) and drops the AUTO_DESCEND request latch +
// failsafe dwell (D1) on every mode entry.  Each Rover mode's _enter()
// already calls this; PR6 only needed it to reset _continuous_sat_s and
// _t_since_mode_change_s.
void AR_FoilControl::notify_mode_change()
{
    _continuous_sat_s = 0.0f;
    _t_since_mode_change_s = 0.0f;
    // Wipe the duty-cycle buffer so the trigger has to re-accumulate over a
    // fresh 5 s window after every mode change.  This is the "gate until
    // buffer is full" edge-case guard per the PR7a D3 spec.
    _sat_buf_idx = 0;
    _sat_buf_fill = 0;
    _sat_buf_sum = 0;
    _sat_duty_cycle = 0.0f;
    _sat_armed = false;
    _sat_trigger_armed = false;
    for (uint16_t i = 0; i < SAT_WINDOW_SAMPLES; i++) {
        _sat_buf[i] = 0;
    }
    // PR7a D1: drop the AUTO_DESCEND request latch + dwell.  Modes consume
    // the latch once on entry (e.g. ModeAutoDescend is the consumer) and we
    // don't want a stale latch re-triggering after the mode has swapped.
    _failsafe_descend_request = false;
    _failsafe_dwell_ms = 0;
    _failsafe_event_pending = false;
}

// PR7a D2: pull-and-clear gate for the FOI3 one-shot event.  Called by
// Rover/Log.cpp at 10 Hz.  Returns true exactly once per failsafe latch.
bool AR_FoilControl::consume_failsafe_event_log_pending()
{
    if (_failsafe_event_pending) {
        _failsafe_event_pending = false;
        return true;
    }
    return false;
}

// PR14a D3: matrix-detector getters (synthesis §3.1).  Return cached
// snapshots updated each tick by update_outer / update_inner.  Const,
// non-mutating, safe to call from a different scheduler task at any time.

bool AR_FoilControl::height_pid_winding() const
{
    return _height_pid_winding;
}

float AR_FoilControl::last_height_error_m() const
{
    return _last_height_err_m;
}

float AR_FoilControl::attitude_error_rad() const
{
    return _att_err_lpf_rad;
}

// PR7a D3: push a single flap-at-limit sample into the circular buffer and
// update the running sum + duty cycle.  Called once per inner-loop tick from
// update_inner() after the per-channel mechanical clamp has been applied.
void AR_FoilControl::push_sat_sample(bool flap_at_limit)
{
    const uint8_t new_bit = flap_at_limit ? 1 : 0;
    const uint8_t old_bit = _sat_buf[_sat_buf_idx];
    if (_sat_buf_fill < SAT_WINDOW_SAMPLES) {
        // Buffer not yet full: we're overwriting a slot that was zero-init'd
        // in the constructor (so old_bit is always 0 here), and we're adding
        // a fresh sample to the count.
        _sat_buf_sum += new_bit;
        _sat_buf_fill++;
    } else {
        // Full: subtract the evicted sample, add the new one.
        _sat_buf_sum = _sat_buf_sum + new_bit - old_bit;
    }
    _sat_buf[_sat_buf_idx] = new_bit;
    _sat_buf_idx++;
    if (_sat_buf_idx >= SAT_WINDOW_SAMPLES) {
        _sat_buf_idx = 0;
    }
    // Gate until the buffer is full (PR7a D3 edge-case: safer to under-trigger
    // during the first 5 s post-boot than to fire on a half-empty buffer).
    if (_sat_buf_fill < SAT_WINDOW_SAMPLES) {
        _sat_duty_cycle = 0.0f;
    } else {
        _sat_duty_cycle = (float)_sat_buf_sum / (float)SAT_WINDOW_SAMPLES;
    }
}

void AR_FoilControl::capture_nominal_gains()
{
    if (_nominal_gains_captured) {
        return;
    }
    _nom_p_rate_kp = _p_rate_pid.kP();
    _nom_p_rate_ki = _p_rate_pid.kI();
    _nom_p_rate_kd = _p_rate_pid.kD();
    _nom_q_rate_kp = _q_rate_pid.kP();
    _nom_q_rate_ki = _q_rate_pid.kI();
    _nom_q_rate_kd = _q_rate_pid.kD();
    _nom_r_rate_kp = _r_rate_pid.kP();
    _nom_r_rate_ki = _r_rate_pid.kI();
    _nom_r_rate_kd = _r_rate_pid.kD();
    _nom_phi_kp    = _phi_pid.kP();
    _nom_phi_ki    = _phi_pid.kI();
    _nom_phi_kd    = _phi_pid.kD();
    _nom_theta_kp  = _theta_pid.kP();
    _nom_theta_ki  = _theta_pid.kI();
    _nom_theta_kd  = _theta_pid.kD();
    _nom_psi_kp    = _psi_p.kP();
    _nominal_gains_captured = true;
}

// Piecewise V² gain scale (PR6, §3).
//   V ≤ V_TO            → frozen at (Vc/Vto)² (prevents 1/V² blow-up + replaces
//                         the PR5 4× hard cap with a value derived from spec).
//   V_TO < V ≤ V_cruise → (Vc/V)² (canonical V² schedule, peaks at 1.0 at Vc).
//   V > V_cruise        → max((Vc/V)², FOIL_SCHED_FL) (clamps the high-V tail).
float AR_FoilControl::gain_scale(float V) const
{
    const float Vc  = MAX(_vcruise.get(), 0.1f);
    const float Vto = MAX(_vto.get(),     0.1f);
    const float frozen = (Vc / Vto) * (Vc / Vto);
    if (V <= Vto) {
        return frozen;
    }
    if (V <= Vc) {
        const float r = Vc / V;
        return r * r;
    }
    const float r = Vc / V;
    float s = r * r;
    const float floor_s = _sched_floor.get();
    if (s < floor_s) {
        s = floor_s;
    }
    return s;
}

void AR_FoilControl::apply_vsq_scheduling()
{
    if (!_nominal_gains_captured) {
        capture_nominal_gains();
    }
    // Pre-foilborne: foils have no usable authority and the V² formula
    // returns the "frozen" multiplier (Vc/Vto)² which is large (~5.35) but
    // bounded by the AC_PID kP/kI/kD setters; we still rely on
    // _foilborne_now-gated integrators upstream to prevent windup.
    float scale = 1.0f;
    if (_foilborne_now) {
        const float v = AP::ahrs().groundspeed();
        scale = gain_scale(v);
    }
    _last_gain_scale = scale;

    // Rate loops + pitch/roll attitude wrappers.
    _p_rate_pid.set_kP(_nom_p_rate_kp * scale);
    _p_rate_pid.set_kI(_nom_p_rate_ki * scale);
    _p_rate_pid.set_kD(_nom_p_rate_kd * scale);
    _q_rate_pid.set_kP(_nom_q_rate_kp * scale);
    _q_rate_pid.set_kI(_nom_q_rate_ki * scale);
    _q_rate_pid.set_kD(_nom_q_rate_kd * scale);
    _r_rate_pid.set_kP(_nom_r_rate_kp * scale);
    _r_rate_pid.set_kI(_nom_r_rate_ki * scale);
    _r_rate_pid.set_kD(_nom_r_rate_kd * scale);
    _phi_pid.set_kP(_nom_phi_kp * scale);
    _phi_pid.set_kI(_nom_phi_ki * scale);
    _phi_pid.set_kD(_nom_phi_kd * scale);
    _theta_pid.set_kP(_nom_theta_kp * scale);
    _theta_pid.set_kI(_nom_theta_ki * scale);
    _theta_pid.set_kD(_nom_theta_kd * scale);
    // Yaw P controller — same scale.
    _psi_p.set_kP(_nom_psi_kp * scale);
}

// Canard pre-load (PR6, §4). Quadratic in V/V_TO so the canard sees small
// deflection during the speed-build where induced w from bow-down hull pitch
// drives α_canard up; deflection ramps in toward V_TO where the lift authority
// is needed.
float AR_FoilControl::canard_preload_rad(float V) const
{
    if (V <= 0.0f) return 0.0f;
    const float vto = MAX(_vto.get(), 0.1f);
    float v_norm = V / vto;
    if (v_norm > 1.0f) v_norm = 1.0f;
    return _pre_can.get() * v_norm * v_norm;
}

// Main pre-load (PR6, §4). Linear in V/V_TO — main foil is aft of CG and
// produces nose-down moment for positive deflection, so its quadratic-vs-
// linear shape is less critical than the canard's.
float AR_FoilControl::main_preload_rad(float V) const
{
    if (V <= 0.0f) return 0.0f;
    const float vto = MAX(_vto.get(), 0.1f);
    float v_norm = V / vto;
    if (v_norm > 1.0f) v_norm = 1.0f;
    return _pre_main.get() * v_norm;
}

// Tustin/bilinear discretisation of H(s) = (1 + τ_lead·s) / (1 + τ_lag·s).
// y[n] = b0·x[n] + b1·x[n-1] - a1·y[n-1], with
//   a = 2/T, denom = 1 + τ_lag·a
//   b0 = (1 + τ_lead·a) / denom
//   b1 = (1 - τ_lead·a) / denom
//   a1 = (1 - τ_lag·a)  / denom
float AR_FoilControl::canard_lead(float x)
{
    if (!_lead_en) {
        // Bypass: pass-through, also reset state so a re-enable later is bumpless.
        _lead_x_prev = x;
        _lead_y_prev = x;
        return x;
    }
    const float T = MAX(_last_inner_dt, 0.0005f);   // floor at 0.5 ms for safety
    const float a = 2.0f / T;
    const float tau_ld = _lead_tau_lead.get();
    const float tau_lg = _lead_tau_lag.get();
    const float denom = 1.0f + tau_lg * a;
    const float b0 = (1.0f + tau_ld * a) / denom;
    const float b1 = (1.0f - tau_ld * a) / denom;
    const float a1 = (1.0f - tau_lg * a) / denom;
    const float y = b0 * x + b1 * _lead_x_prev - a1 * _lead_y_prev;
    _lead_x_prev = x;
    _lead_y_prev = y;
    return y;
}

void AR_FoilControl::init()
{
    capture_nominal_gains();
    if (_servo_ranges_set) {
        return;
    }
    // Configure ±2500 centideg = ±25° mechanical half-range. Plant model uses
    // PWM 1500 = 0° and ±500 PWM = ±25° — set_angle with 2500 centideg encodes
    // exactly this mapping when the SERVOn_MIN/MAX/TRIM are at their defaults.
    SRV_Channels::set_angle(SRV_Channel::k_foilcontrol_canard, AR_FOILCONTROL_SRV_ANGLE_CD);
    SRV_Channels::set_angle(SRV_Channel::k_foilcontrol_main,   AR_FOILCONTROL_SRV_ANGLE_CD);
    SRV_Channels::set_angle(SRV_Channel::k_foilcontrol_rudder, AR_FOILCONTROL_SRV_ANGLE_CD);
    _servo_ranges_set = true;
}

// Read the downward-facing rangefinder. Returns NaN when no healthy instance.
float AR_FoilControl::get_height_above_water() const
{
#if AP_RANGEFINDER_ENABLED
    const RangeFinder *rf = AP::rangefinder();
    if (rf == nullptr) {
        return nanf("");
    }
    if (rf->status_orient(ROTATION_PITCH_270) != RangeFinder::Status::Good) {
        return nanf("");
    }
    return rf->distance_orient(ROTATION_PITCH_270);
#else
    return nanf("");
#endif
}

// -----------------------------------------------------------------------------
// Outer loop (100 Hz): height -> θ_cmd, heading -> r_setpoint, roll attitude
// kept ready for v1 dual-main-flap (φ_cmd is currently zero on the v0 rig per
// §6 open question 3).  θ_setpoint and r_setpoint are stashed for the 400 Hz
// inner loop to consume.
// -----------------------------------------------------------------------------
void AR_FoilControl::update_outer()
{
    const uint32_t now_us = AP_HAL::micros();
    float dt;
    if (_last_outer_us == 0) {
        dt = 0.01f;  // first call: 100 Hz nominal
        _theta_pid.reset_filter();
        _phi_pid.reset_filter();
        _theta_pid.reset_I();
        _phi_pid.reset_I();
        _h_integrator = 0.0f;
        _h_last_theta_cmd = 0.0f;
    } else {
        dt = (now_us - _last_outer_us) * 1.0e-6f;
        dt = constrain_float(dt, 0.002f, 0.05f);
    }
    _last_outer_us = now_us;

    // --- Height outer loop -------------------------------------------------
    // h_cmd -> θ_cmd, clamped to ±0.105 rad (±6°).  Kd term is on vertical
    // rate ż (NED-down convention: ż_up = -ż_NED).  When LIDAR is unhealthy
    // (NaN), hold the last good θ_cmd and freeze the integrator.
    //
    // Pre-foilborne gate (§5.3 / §6 q1 spirit): until the boat is actually
    // flying (h_meas > FOIL_HGT_FB, default 0.06 m), the height-PID does not
    // wind up — we issue θ_cmd = 0 and zero the integrator.  This prevents
    // the integrator from saturating during sub-V_TO acceleration in the
    // water, where the foils have no authority to track a non-zero θ_cmd.
    const float h_meas = get_height_above_water();
    float theta_cmd_rad;
    const float h_fb_thresh = _h_foilborne_thresh.get();
    const bool foilborne_now = isfinite(h_meas) && (h_meas > h_fb_thresh);
    if (foilborne_now && !_foilborne_now) {
        // Rising edge: reset the soft-engage timer.
        _foilborne_engage_s = 0.0f;
    }
    _foilborne_now = foilborne_now;
    if (foilborne_now) {
        _foilborne_engage_s += dt;
    } else {
        _foilborne_engage_s = 0.0f;
    }
    // Soft-engage ramp: 0..1 over the first 1.0 s of foilborne (cosine
    // half-period), then unity.  Multiplies the height-PID output (θ_cmd)
    // and the attitude-wrapper rate setpoints.  Prevents the gain schedule
    // from slamming the cascade at the transition.
    const float ENGAGE_RAMP_S = 1.0f;
    float engage = 1.0f;
    if (_foilborne_engage_s < ENGAGE_RAMP_S) {
        const float u = _foilborne_engage_s / ENGAGE_RAMP_S;
        engage = 0.5f * (1.0f - cosf(M_PI * u));
    }
    _engage_factor = engage;

    if (!isfinite(h_meas)) {
        // P4 LIDAR dropout: hold last good cmd, freeze integrator.
        theta_cmd_rad = _h_last_theta_cmd;
    } else if (!foilborne_now) {
        // Pre-foilborne / hull-borne: do not chase a height that isn't there.
        // Bleed the integrator toward zero (1 s time-constant) so that when we
        // do break foilborne, we start from a clean slate.
        _h_integrator *= MAX(0.0f, 1.0f - dt);
        theta_cmd_rad = 0.0f;
        _h_last_theta_cmd = 0.0f;
    } else {
        const float h_err = _height_target_m - h_meas;
        Vector3f vel_ned;
        float zdot_up = 0.0f;  // m/s upward
        if (AP::ahrs().get_velocity_NED(vel_ned)) {
            zdot_up = -vel_ned.z;
        }

        // Anti-windup: freeze integrator while pitch_mix_cmd has been
        // saturated for > 0.2 s (§5.3).  We always integrate the new sample
        // unless that flag is asserted.
        if (!_pitch_saturated) {
            _h_integrator += _h_i.get() * h_err * dt;
        }
        // Clamp integrator to the same ±0.105 rad envelope as the output
        // (so it can't store more than the loop can express).
        _h_integrator = constrain_float(_h_integrator,
                                        -AR_FOILCONTROL_THETA_CMD_LIMIT_RAD,
                                        AR_FOILCONTROL_THETA_CMD_LIMIT_RAD);

        // u = Kp*err + I  - Kd*ż_up
        const float u_p = _h_p.get() * h_err;
        const float u_d = -_h_d.get() * zdot_up;
        float u = u_p + _h_integrator + u_d;
        u = constrain_float(u,
                            -AR_FOILCONTROL_THETA_CMD_LIMIT_RAD,
                            AR_FOILCONTROL_THETA_CMD_LIMIT_RAD);
        u *= engage;
        theta_cmd_rad = u;
        _h_last_theta_cmd = u;
    }

    // Mode-level override (e.g. HULL_BORNE trim, currently unused — kept as a
    // hook so a future PR can inject a non-cascade θ_cmd without re-plumbing).
    if (_pitch_target_override) {
        theta_cmd_rad = _pitch_target_override_rad;
    }
    _theta_cmd_rad = theta_cmd_rad;

    // --- Heading P controller -> yaw-rate target --------------------------
    // PR6: yaw outer is now AC_P (no integrator, no derivative). Wrap heading
    // error on ±π so we always take the short way round.
    const float yaw_meas = AP::ahrs().get_yaw_rad();
    const float yaw_err = wrap_PI(_heading_target_rad - yaw_meas);
    // Pre-foilborne: hold rudder at zero (rudder T-foil has near-zero
    // authority below V_TO).
    if (!foilborne_now) {
        _r_setpoint_rad_s = 0.0f;
    } else {
        const float r_set = _psi_p.get_p(yaw_err) * engage;
        _r_setpoint_rad_s = constrain_float(r_set, -3.0f, 3.0f);
    }

    // --- Roll attitude PID -> roll-rate target ----------------------------
    // v0 mono single-main-flap rig: φ_cmd = 0 (open-loop roll, hull stiffness
    // does the work — §6 open question 3).  Run the PID anyway so the
    // integrator stays warm and a v1 differential-main pickup is one-line.
    const float phi_meas = AP::ahrs().get_roll_rad();
    const float phi_cmd  = 0.0f;
    if (!foilborne_now) {
        _phi_pid.reset_I();
        _p_setpoint_rad_s = 0.0f;
    } else {
        const float p_set = _phi_pid.update_all(phi_cmd, phi_meas, dt) * engage;
        _p_setpoint_rad_s = constrain_float(p_set, -3.0f, 3.0f);
    }

    // theta_pid is run inside update_inner() so it ticks at 400 Hz and tracks
    // the latest θ_cmd produced here.

    // PR7a D2: FOI/FOI2 emissions moved to Rover/Log.cpp so we can register
    // them as static LogStructure entries (gives MAVLink log-download clients
    // proper field metadata).  Snapshot values (_theta_cmd_rad, _p_setpoint_rad_s,
    // _r_setpoint_rad_s, _canard_preload_last, _last_gain_scale, _sat_trigger_armed,
    // _sat_duty_cycle) are exposed via const getters; Rover pulls them at
    // its own logging cadence.

    // PR14a D3: matrix-detector snapshots — purely additive bookkeeping for
    // the new const getters consumed by FoilboatFailsafe (PR14b/c).  No
    // control behaviour change: only read existing state into cache fields.
    //
    //   _last_height_err_m       — h_meas − h_cmd, NaN when LIDAR is invalid
    //                              so the matrix's H_AGL_DISAGREE row can
    //                              skip the tick rather than fire on stale.
    //   _h_integrator_clamp_dwell_s — seconds the height-PID integrator has
    //                              been within ε of its ±0.105 rad clamp.
    //   _height_pid_winding      — dwell > 1 s (K1 INTEGRATOR_WIND_UP row).
    if (isfinite(h_meas)) {
        _last_height_err_m = h_meas - _height_target_m;
    } else {
        _last_height_err_m = AP_Logger::quiet_nanf();
    }
    {
        const float clamp_eps = 1e-4f;
        const float clamp_lim = AR_FOILCONTROL_THETA_CMD_LIMIT_RAD;
        if (fabsf(_h_integrator) >= clamp_lim - clamp_eps) {
            _h_integrator_clamp_dwell_s += dt;
        } else {
            _h_integrator_clamp_dwell_s = 0.0f;
        }
        _height_pid_winding = (_h_integrator_clamp_dwell_s > 1.0f);
    }
}

// -----------------------------------------------------------------------------
// Inner loop (400 Hz): attitude wrappers + body-rate PIDs -> mixer -> surfaces.
// -----------------------------------------------------------------------------
void AR_FoilControl::update_inner()
{
    // Compute dt; clamp to [1 ms, 10 ms] (400 Hz nominal is 2.5 ms).
    const uint32_t now_us = AP_HAL::micros();
    float dt;
    if (_last_inner_us == 0) {
        dt = 0.0025f;
        _p_rate_pid.reset_filter();
        _q_rate_pid.reset_filter();
        _r_rate_pid.reset_filter();
        _p_rate_pid.reset_I();
        _q_rate_pid.reset_I();
        _r_rate_pid.reset_I();
    } else {
        dt = (now_us - _last_inner_us) * 1.0e-6f;
        dt = constrain_float(dt, 0.001f, 0.01f);
    }
    _last_inner_us = now_us;
    _last_inner_dt = dt;

    // V^2 gain schedule before each tick.
    apply_vsq_scheduling();

    // Body rates p, q, r in rad/s.
    const Vector3f gyro = AP::ahrs().get_gyro();
    const float q_meas  = gyro.y;

    // --- Pitch attitude wrapper (400 Hz) ----------------------------------
    // θ_cmd -> q_setpoint via P+I PID.  PR6: the manual `-Kd_θ·q` term is
    // gone — Kd_θ is now Kd on the inner pitch-rate PID (canonical AC_PID).
    // Pre-foilborne: zero the rate target and bleed the theta integrator.
    const float theta_meas = AP::ahrs().get_pitch_rad();
    if (!_foilborne_now) {
        _theta_pid.reset_I();
        _q_setpoint_rad_s = 0.0f;
    } else {
        float q_set       = _theta_pid.update_all(_theta_cmd_rad, theta_meas, dt);
        q_set            *= _engage_factor;                  // soft-engage at transition
        _q_setpoint_rad_s = constrain_float(q_set, -6.0f, 6.0f);
    }

    // --- Pilot stick fallback when no outer setpoint has been published ---
    if (_last_outer_us == 0 && rc().has_valid_input()) {
        const float p_stick = rc().get_roll_channel().norm_input_dz()  * AR_FOILCONTROL_PILOT_RATE_SCALE;
        const float q_stick = rc().get_pitch_channel().norm_input_dz() * AR_FOILCONTROL_PILOT_RATE_SCALE;
        const float r_stick = rc().get_yaw_channel().norm_input_dz()   * AR_FOILCONTROL_PILOT_RATE_SCALE;
        _p_setpoint_rad_s = p_stick;
        _q_setpoint_rad_s = q_stick;
        _r_setpoint_rad_s = r_stick;
    }

    // --- Body-rate PIDs ---------------------------------------------------
    const bool rate_limit = !_foilborne_now;
    const float p_out = _p_rate_pid.update_all(_p_setpoint_rad_s, gyro.x, dt, rate_limit);
    const float q_out = _q_rate_pid.update_all(_q_setpoint_rad_s, q_meas,  dt, rate_limit);
    const float r_out = _r_rate_pid.update_all(_r_setpoint_rad_s, gyro.z,  dt, rate_limit);

    // --- Mixer (§2.1) -----------------------------------------------------
    float pitch_mix_cmd = q_out;
    float roll_diff_cmd = p_out;
    float rudder_cmd_raw = r_out;

    // §5.4 differential roll headroom (±15°).
    roll_diff_cmd = constrain_float(roll_diff_cmd,
                                    -AR_FOILCONTROL_ROLL_DIFF_LIMIT_RAD,
                                    +AR_FOILCONTROL_ROLL_DIFF_LIMIT_RAD);
    // §5.1 mechanical clamp on pitch mix (±25°).
    const float pitch_mix_unclipped = pitch_mix_cmd;
    pitch_mix_cmd = constrain_float(pitch_mix_cmd,
                                    -AR_FOILCONTROL_FLAP_LIMIT_RAD,
                                    +AR_FOILCONTROL_FLAP_LIMIT_RAD);

    // §5.3 cascade-level pitch-saturation tracking: dwell timer + flag.
    if (fabsf(pitch_mix_unclipped) > AR_FOILCONTROL_FLAP_LIMIT_RAD) {
        _pitch_sat_dwell_s += dt;
        if (_pitch_sat_dwell_s > AR_FOILCONTROL_PITCH_SAT_DWELL_S) {
            _pitch_saturated = true;
        }
    } else {
        _pitch_sat_dwell_s = 0.0f;
        _pitch_saturated = false;
    }

    // --- Mixer outputs (§2.1) --------------------------------------------
    //   canard      = +k_pc · pitch_mix_cmd
    //   main        = -k_pm · pitch_mix_cmd
    //   rudder      = rudder_cmd
    float canard_cmd = +AR_FOILCONTROL_MIX_K_PC * pitch_mix_cmd;
    float main_cmd   = -AR_FOILCONTROL_MIX_K_PM * pitch_mix_cmd;
    float rudder_cmd =  rudder_cmd_raw;

    // --- PR6 §4: pre-load feed-forward + foilborne engage taper -----------
    // Pre-load bypasses the pitch integrator so it acts as a feed-forward
    // bias the rate-loop integrator never sees. Schedule:
    //   HULL_BORNE: pre-load is the ONLY canard/main command (no PID).
    //   FOILBORNE engage ramp: scale pre-load by (1 - engage_factor) so it
    //     fades out as the PID takes over.
    //   FOILBORNE steady: pre-load = 0.
    const float V = AP::ahrs().groundspeed();
    const float canard_pl = canard_preload_rad(V);
    const float main_pl   = main_preload_rad(V);
    _canard_preload_last = canard_pl;
    if (!_foilborne_now) {
        // Hull-borne: only the pre-load drives the surfaces; rate-PID outputs
        // (q_out etc.) ride on top of zero engage_factor on the rate setpoints
        // anyway, but be explicit here so the wiring is clear.
        canard_cmd = canard_pl;
        main_cmd   = main_pl;
    } else {
        // Foilborne: PID + (1 - engage)*pre-load. engage_factor ramps 0->1
        // over the first 1 s of foilborne, so this term smoothly fades out.
        const float fade = 1.0f - _engage_factor;
        canard_cmd += fade * canard_pl;
        main_cmd   += fade * main_pl;
    }

    // --- PR6 §4 canard lead compensator ----------------------------------
    // H(s) = (1 + 0.060·s) / (1 + 0.015·s).  Applied between mixer and the
    // ±25° clamp so the compensator sees the full mixer-shaped command but
    // the output is still hard-bounded by the mechanical limit.
    canard_cmd = canard_lead(canard_cmd);

    // Per-channel mechanical clamp (defence-in-depth).
    _canard_cmd_rad = constrain_float(canard_cmd, -AR_FOILCONTROL_FLAP_LIMIT_RAD, +AR_FOILCONTROL_FLAP_LIMIT_RAD);
    _main_cmd_rad   = constrain_float(main_cmd,   -AR_FOILCONTROL_FLAP_LIMIT_RAD, +AR_FOILCONTROL_FLAP_LIMIT_RAD);
    _rudder_cmd_rad = constrain_float(rudder_cmd, -AR_FOILCONTROL_FLAP_LIMIT_RAD, +AR_FOILCONTROL_FLAP_LIMIT_RAD);

    // --- PR7a D3: rolling saturation duty-cycle sample -------------------
    // Any surface within SAT_EPSILON_RAD of its mechanical limit counts as
    // "at limit" for this tick.  Sampled at the inner-loop rate (400 Hz)
    // and accumulated into the 2000-sample (5 s) ring buffer.
    const float sat_thr = AR_FOILCONTROL_FLAP_LIMIT_RAD - AR_FOILCONTROL_SAT_EPSILON_RAD;
    const bool flap_at_limit =
        (fabsf(_canard_cmd_rad) >= sat_thr) ||
        (fabsf(_main_cmd_rad)   >= sat_thr) ||
        (fabsf(_rudder_cmd_rad) >= sat_thr);
    push_sat_sample(flap_at_limit);

    // PR14a D3: attitude-error LPF for matrix-detector consumption.  Pure
    // bookkeeping: K2 ATT_DIVERGE (synthesis §2) needs a low-passed (θ_cmd −
    // θ_meas) signal so a single noisy sample at the inner-loop rate can't
    // trip the dwell.  τ = 0.1 s gives a 1.6 Hz corner — fast enough to
    // catch real divergence (typical K2 dwell ≥ 200 ms) and slow enough to
    // reject the 50 Hz LIDAR-noise re-injection into θ_cmd via the height
    // loop.  PR14b owns the actual K2 threshold + dwell.  Reuses the
    // `theta_meas` already snapshotted above for the pitch-attitude wrapper.
    {
        const float att_err = _theta_cmd_rad - theta_meas;
        const float att_lp_tau_s = 0.1f;
        const float att_alpha = dt / (att_lp_tau_s + dt);
        _att_err_lpf_rad += att_alpha * (att_err - _att_err_lpf_rad);
    }

    // roll_diff_cmd is unused in v0; reference once to silence -Wunused.
    (void)roll_diff_cmd;
}

// -----------------------------------------------------------------------------
// Speed loop (50 Hz): V_cmd -> throttle [0..1] -> AP_MotorsUGV (which uses
// -100..+100).  Writes via g2.motors.set_throttle so the existing rover
// motors_output() path emits the PWM on k_throttle.
// -----------------------------------------------------------------------------
void AR_FoilControl::update_throttle()
{
    const uint32_t now_us = AP_HAL::micros();
    float dt;
    if (_last_throttle_us == 0) {
        dt = 0.02f;  // 50 Hz nominal
        _v_pid.reset_filter();
        _v_pid.reset_I();
    } else {
        dt = (now_us - _last_throttle_us) * 1.0e-6f;
        dt = constrain_float(dt, 0.005f, 0.1f);
    }
    _last_throttle_us = now_us;

    // Forward speed feedback: prefer AHRS groundspeed (GPS/EKF fused) over
    // the body-frame projection so the controller is direction-agnostic.
    const float v_meas = AP::ahrs().groundspeed();
    float throttle = _v_pid.update_all(_speed_target_ms, v_meas, dt);
    throttle = constrain_float(throttle, 0.0f, 1.0f);

    _v_pid.set_target_rate(_speed_target_ms);
    _v_pid.set_actual_rate(v_meas);
    _throttle_cmd = throttle;
}

// -----------------------------------------------------------------------------
// Failsafe state machine (10 Hz) — PR6 §5: arm the saturation triggers only
// when foilborne + settled, then run the dwell-time latches.
//
// Two saturation triggers are armed by this gate (both per §5.5 of the
// design spec):
//   (a) Continuous-saturation: pitch_mix_cmd has been clipped for > 0.5 s
//       continuously. Driven by _pitch_saturated (the dwell-based flag set
//       inside update_inner()).
//   (b) Duty-cycle saturation: |u_sat - u_unsat| > 0.05·u_range averaged over
//       a 5 s window with duty > 10 %. STUBBED IN PR6 — the actual duty
//       counter + windowed average is a TODO(PR7); only the arming gate is
//       wired here so PR7 can hang the counter off it.
//
// Both triggers' action (latch into AUTO_DESCEND) is also TODO(PR7) — the
// state machine that owns the mode-switch lives outside this class. PR6
// only sets the conditions for the latch and logs the trigger.
// -----------------------------------------------------------------------------
void AR_FoilControl::update_failsafe()
{
    const uint32_t now_us = AP_HAL::micros();
    float dt;
    if (_last_failsafe_us == 0) {
        dt = 0.1f;  // 10 Hz nominal
    } else {
        dt = (now_us - _last_failsafe_us) * 1.0e-6f;
        dt = constrain_float(dt, 0.01f, 0.5f);
    }
    _last_failsafe_us = now_us;
    const uint32_t dt_ms = (uint32_t)(dt * 1000.0f + 0.5f);

    _t_since_mode_change_s += dt;

    // --- Arming gate (PR6 §5, kept) ---------------------------------------
    const float V = AP::ahrs().groundspeed();
    const bool foilborne_and_above_vto = _foilborne_now && (V > (_vto.get() + 0.2f));
    const bool engaged                  = _engage_factor > 0.9f;
    const bool settled                  = _t_since_mode_change_s > AR_FOILCONTROL_MODE_SETTLE_S;
    _sat_armed = foilborne_and_above_vto && engaged && settled;

    // --- PR7a D3: duty-cycle trigger replaces the PR6 instantaneous gate --
    // _sat_trigger_armed is now: armed AND (duty cycle exceeded over a full
    // 5 s window).  push_sat_sample() in update_inner() keeps _sat_duty_cycle
    // at 0 until the buffer is full, so this naturally gates the trigger
    // until the buffer is warm.
    const bool duty_exceeded = _sat_duty_cycle > _sat_duty_thr.get();
    _sat_trigger_armed = _sat_armed && duty_exceeded;

    // --- Continuous-saturation dwell (PR6, retained for diagnostics) ------
    // PR7a-D1 hangs the AUTO_DESCEND request off the duty-cycle trigger
    // (_sat_trigger_armed) below.  The continuous-sat dwell is kept here for
    // logging / future heuristics but is not on the failsafe-action path.
    if (_sat_armed && _pitch_saturated) {
        _continuous_sat_s += dt;
    } else {
        _continuous_sat_s = 0.0f;
    }

    // --- PR7a D1: FOIL_FAIL_DWELL counter -> AUTO_DESCEND request ---------
    // Accumulate dwell while the duty-cycle trigger is armed.  Latch the
    // request when the dwell exceeds FOIL_FAIL_DWELL; the latch is consumed
    // by Rover modes (ModeFoilborneHold::update polls should_failsafe_descend)
    // and cleared on the next notify_mode_change().  A transient duty dip
    // resets the dwell counter but does NOT drop the latch — once requested,
    // the descent stays requested until the mode actually switches.
    if (_sat_trigger_armed) {
        _failsafe_dwell_ms += dt_ms;
        if (_failsafe_dwell_ms >= (uint32_t)_fail_dwell_ms.get() &&
            !_failsafe_descend_request) {
            _failsafe_descend_request = true;
            // PR7a D2: signal the FOI3 one-shot event to Rover/Log.cpp via a
            // pull-and-clear pending flag, rather than emitting WriteStreaming
            // from inside the controller library.
            _failsafe_event_pending = true;
        }
    } else {
        _failsafe_dwell_ms = 0;
    }
}

void AR_FoilControl::output_to_servos()
{
    // Lazy-init servo ranges + nominal-gain capture; cheap and idempotent.
    init();

    // Convert radians → centidegree for SRV_Channels::set_output_scaled().
    constexpr float RAD_TO_CD = 18000.0f / M_PI;
    SRV_Channels::set_output_scaled(SRV_Channel::k_foilcontrol_canard, _canard_cmd_rad * RAD_TO_CD);
    SRV_Channels::set_output_scaled(SRV_Channel::k_foilcontrol_main,   _main_cmd_rad   * RAD_TO_CD);
    SRV_Channels::set_output_scaled(SRV_Channel::k_foilcontrol_rudder, _rudder_cmd_rad * RAD_TO_CD);
}
