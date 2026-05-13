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
//   q-axis (pitch rate): Kp=0.19, Ki=0.02, Kd=0
//   p-axis (roll  rate): Kp=0.04, Ki=0.005, Kd=0
//   r-axis (yaw   rate): Kp=1.2,  Ki=0.3,  Kd=0
#define AR_FOILCONTROL_Q_RATE_P         0.19f
#define AR_FOILCONTROL_Q_RATE_I         0.02f
#define AR_FOILCONTROL_Q_RATE_D         0.00f
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

// Pitch attitude PID (§1.1).  Kd term applied externally on body rate q.
#define AR_FOILCONTROL_THETA_P          4.00f
#define AR_FOILCONTROL_THETA_I          0.50f
#define AR_FOILCONTROL_THETA_D          0.00f      // D-on-q applied manually below
#define AR_FOILCONTROL_THETA_FF         0.00f
#define AR_FOILCONTROL_THETA_IMAX       2.00f
#define AR_FOILCONTROL_THETA_KD_Q       0.19f      // manual D coefficient: u -= Kd_theta * q

// Roll attitude PID (§1.2).
#define AR_FOILCONTROL_PHI_P            0.60f
#define AR_FOILCONTROL_PHI_I            0.05f
#define AR_FOILCONTROL_PHI_D            0.04f
#define AR_FOILCONTROL_PHI_FF           0.00f
#define AR_FOILCONTROL_PHI_IMAX         0.50f

// Heading PID (no explicit spec gains — first-cut: Kp=1.0, Ki=0, Kd=0 per §1.3 note).
#define AR_FOILCONTROL_PSI_P            1.00f
#define AR_FOILCONTROL_PSI_I            0.00f
#define AR_FOILCONTROL_PSI_D            0.00f
#define AR_FOILCONTROL_PSI_FF           0.00f
#define AR_FOILCONTROL_PSI_IMAX         1.00f

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
    // Pitch attitude PID (PR5) — theta_cmd → q_setpoint. AC_PID's internal D
    // is held at zero; a manual D-on-q term (FOIL_KD_THETA · q) is subtracted
    // outside the PID per §1.1 ("Kd is on q (rate), not on dθ/dt").
    AP_SUBGROUPINFO(_theta_pid, "PIT_", 12, AR_FoilControl, AC_PID),

    // @Group: YAW_
    // @Path: ../AC_PID/AC_PID.cpp
    // Heading PID (PR5) — psi_cmd → r_setpoint, first-cut Kp=1.0.
    AP_SUBGROUPINFO(_psi_pid, "YAW_", 13, AR_FoilControl, AC_PID),

    // @Param: VCRUISE
    // @DisplayName: V^2 scheduling cruise speed
    // @Description: Reference cruise speed for V^2 gain scheduling (m/s).
    // Pitch/roll attitude + rate-loop gains are multiplied by (VCRUISE / max(V, VMINSCHD))^2.
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

    // @Param: KD_THETA
    // @DisplayName: Pitch attitude D-on-q coefficient
    // @Description: Manual derivative coefficient applied to body pitch rate q
    // inside the pitch attitude wrapper (rad-flap per rad/s). See 04-control-law.md §1.1.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("KD_THETA",   16, AR_FoilControl, _kd_theta, AR_FOILCONTROL_THETA_KD_Q),

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
    _psi_pid(AR_FOILCONTROL_PSI_P, AR_FOILCONTROL_PSI_I, AR_FOILCONTROL_PSI_D,
             AR_FOILCONTROL_PSI_FF, AR_FOILCONTROL_PSI_IMAX,
             AR_FOILCONTROL_ATT_FILT_T_HZ, AR_FOILCONTROL_ATT_FILT_E_HZ, AR_FOILCONTROL_ATT_FILT_D_HZ),
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
    _nominal_gains_captured(false),
    _pitch_sat_dwell_s(0.0f),
    _pitch_saturated(false),
    _foilborne_now(false),
    _foilborne_engage_s(0.0f),
    _engage_factor(0.0f),
    _last_inner_us(0),
    _last_outer_us(0),
    _last_throttle_us(0),
    _servo_ranges_set(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
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
    _nominal_gains_captured = true;
}

void AR_FoilControl::apply_vsq_scheduling()
{
    if (!_nominal_gains_captured) {
        capture_nominal_gains();
    }
    // Pre-foilborne: foils have no usable authority and the V² formula
    // blows up (×54 at V=Vmin); use the nominal gains so the rate loop can
    // run at safe magnitudes while the boat is hull-borne.
    float scale = 1.0f;
    if (_foilborne_now) {
        // Forward speed from AHRS (m/s).  groundspeed() is GPS-fused; with
        // AHRS_EKF_TYPE=10 it reflects the JSON-supplied velocity directly.
        const float v = AP::ahrs().groundspeed();
        const float v_min = MAX(_vmin_sched.get(), 0.1f);
        const float v_eff = MAX(v, v_min);
        const float vc    = MAX(_vcruise.get(), v_min);  // never let Vc < Vmin
        const float ratio = vc / v_eff;
        scale = ratio * ratio;  // V^2 schedule per §1.1
        // Cap the scheduling multiplier.  Spec §6 q6 acknowledges that the
        // raw schedule needs Monte-Carlo verification across V∈[1,4] m/s.
        // Until that work lands (post-PR5), clamp the multiplier to 4× as
        // a defensive gain ceiling.
        const float SCHED_MAX = 4.0f;
        if (scale > SCHED_MAX) {
            scale = SCHED_MAX;
        }
    }

    // Rate loops + pitch/roll attitude wrappers.  Yaw / heading / speed / height
    // loops are intentionally NOT scheduled (no aerodynamic q-dependence in
    // their plant gains, per spec).
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
        _psi_pid.reset_filter();
        _theta_pid.reset_I();
        _phi_pid.reset_I();
        _psi_pid.reset_I();
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

        // u = Kp*err + I  - Kd*ż_up    (sign: pitch-up to climb when h<h_cmd
        //                                AND when descending: zdot_up<0 means we
        //                                want extra pitch-up, so subtract zdot)
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

    // --- Heading PID -> yaw-rate target -----------------------------------
    // Wrap heading error on ±π so we always take the short way round.
    const float yaw_meas = AP::ahrs().get_yaw_rad();
    const float yaw_err = wrap_PI(_heading_target_rad - yaw_meas);
    // Pre-foilborne: hold rudder at zero and bleed integrator (rudder T-foil
    // has near-zero authority below V_TO).
    if (!foilborne_now) {
        _psi_pid.reset_I();
        _r_setpoint_rad_s = 0.0f;
    } else {
        const float r_set = _psi_pid.update_error(yaw_err, dt, false) * engage;
        _r_setpoint_rad_s = constrain_float(r_set, -3.0f, 3.0f);  // clamp body-rate target ±3 rad/s
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

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: FOIL
    // @Description: AR_FoilControl outer-loop telemetry
    // @Field: TimeUS: Time since system startup
    // @Field: Hgt: Height above water from downward rangefinder (m), NaN if unhealthy
    // @Field: ThC: Pitch-cmd from height loop (rad)
    // @Field: PSet: Roll-rate target (rad/s)
    // @Field: RSet: Yaw-rate target (rad/s)
    AP::logger().WriteStreaming("FOIL",
                                "TimeUS,Hgt,ThC,PSet,RSet",
                                "smrrr",
                                "F0000",
                                "Qffff",
                                AP_HAL::micros64(),
                                h_meas,
                                _theta_cmd_rad,
                                _p_setpoint_rad_s,
                                _r_setpoint_rad_s);
#endif
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

    // V^2 gain schedule before each tick (cheap; touches AC_PID's AP_Float
    // backing so the parameter view still reads the user value -- we mutate
    // the in-RAM copy only).
    apply_vsq_scheduling();

    // Body rates p, q, r in rad/s.
    const Vector3f gyro = AP::ahrs().get_gyro();

    // --- Pitch attitude wrapper (400 Hz) ----------------------------------
    // θ_cmd -> q_setpoint via P+I PID, then manual -Kd*q derivative-on-rate.
    // Pre-foilborne: zero the rate target and bleed the theta integrator.
    // The foils have no authority in the water, so pitch tracking is futile
    // and the integrator would wind up against the hull stiffness.
    const float theta_meas = AP::ahrs().get_pitch_rad();
    const float q_meas     = gyro.y;
    if (!_foilborne_now) {
        _theta_pid.reset_I();
        _q_setpoint_rad_s = 0.0f;
    } else {
        float q_set       = _theta_pid.update_all(_theta_cmd_rad, theta_meas, dt);
        q_set            -= _kd_theta.get() * q_meas;       // D-on-q (§1.1)
        q_set            *= _engage_factor;                  // soft-engage at transition
        _q_setpoint_rad_s = constrain_float(q_set, -6.0f, 6.0f);
    }

    // --- Pilot stick fallback when no outer setpoint has been published ---
    // If the outer loop hasn't run yet (first 1-2 ticks), or the user is in a
    // mode that didn't override theta/heading, use the RC stick directly so
    // the rig stays controllable.  PR5: in foilborne mode the outer loop has
    // already populated the setpoints, so this branch is only hit at startup.
    if (_last_outer_us == 0 && rc().has_valid_input()) {
        const float p_stick = rc().get_roll_channel().norm_input_dz()  * AR_FOILCONTROL_PILOT_RATE_SCALE;
        const float q_stick = rc().get_pitch_channel().norm_input_dz() * AR_FOILCONTROL_PILOT_RATE_SCALE;
        const float r_stick = rc().get_yaw_channel().norm_input_dz()   * AR_FOILCONTROL_PILOT_RATE_SCALE;
        _p_setpoint_rad_s = p_stick;
        _q_setpoint_rad_s = q_stick;
        _r_setpoint_rad_s = r_stick;
    }

    // --- Body-rate PIDs ---------------------------------------------------
    // Pass `limit=true` to AC_PID's integrator when not foilborne — the foils
    // have no authority so any "error" the rate loop sees is hull-stiffness
    // residual, not something the controller can act on.  AC_PID's limit=true
    // mode lets the integrator shrink toward zero but blocks growth (one-sided
    // back-calculation).  Combined with the per-tick theta=0 setpoint, this
    // converges the rate-loop integrators to zero in the water.
    const bool rate_limit = !_foilborne_now;
    const float p_out = _p_rate_pid.update_all(_p_setpoint_rad_s, gyro.x, dt, rate_limit);
    const float q_out = _q_rate_pid.update_all(_q_setpoint_rad_s, q_meas,  dt, rate_limit);
    const float r_out = _r_rate_pid.update_all(_r_setpoint_rad_s, gyro.z,  dt, rate_limit);

    // --- Mixer (§2.1) -----------------------------------------------------
    // Pitch mix uses q_out (rad/s of pitch-rate-equivalent flap demand).
    // Roll diff uses p_out — only emitted to the differential channel; the
    // single-main-flap v0 rig does NOT route p_out into the main flap
    // (see §2 of 04-control-law.md and §6 open question 3).  We compute it
    // here so the v1 dual-surface main inherits the wiring without churn.
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

    // Mixer outputs (§2.1):
    //   canard      = +k_pc · pitch_mix_cmd      (roll term reserved for v1)
    //   main        = -k_pm · pitch_mix_cmd      (v0: no differential roll on
    //                                              the single main flap)
    //   rudder      = rudder_cmd
    // When the v1 dual-surface main lands, replace the single _main_cmd_rad
    // with split left/right channels that add ±roll_diff_cmd respectively.
    float canard_cmd = +AR_FOILCONTROL_MIX_K_PC * pitch_mix_cmd;
    float main_cmd   = -AR_FOILCONTROL_MIX_K_PM * pitch_mix_cmd;
    float rudder_cmd =  rudder_cmd_raw;

    // Per-channel mechanical clamp (defence-in-depth; pitch_mix already
    // bounded so this only bites on lift_trim / future contributions).
    _canard_cmd_rad = constrain_float(canard_cmd, -AR_FOILCONTROL_FLAP_LIMIT_RAD, +AR_FOILCONTROL_FLAP_LIMIT_RAD);
    _main_cmd_rad   = constrain_float(main_cmd,   -AR_FOILCONTROL_FLAP_LIMIT_RAD, +AR_FOILCONTROL_FLAP_LIMIT_RAD);
    _rudder_cmd_rad = constrain_float(rudder_cmd, -AR_FOILCONTROL_FLAP_LIMIT_RAD, +AR_FOILCONTROL_FLAP_LIMIT_RAD);

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

    // We do NOT call g2.motors.set_throttle() here to avoid coupling
    // AR_FoilControl directly to Rover internals.  The mode's update() reads
    // this value via get_throttle_cmd() and applies it to g2.motors, which
    // keeps set_servos() as the single PWM writer.
    _v_pid.set_target_rate(_speed_target_ms);
    _v_pid.set_actual_rate(v_meas);
    _throttle_cmd = throttle;
}

void AR_FoilControl::update_failsafe()
{
    // PR6.
    return;
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
