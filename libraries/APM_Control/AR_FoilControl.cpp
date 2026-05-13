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

// PR1 defaults (per research/flight-dev-rig/05-ardufoil-integration.md §5):
//   FOIL_HGT_TGT 0.10 m (100 mm ride height for 1 m rig / 4 m proto pre-tuning)
//   FOIL_VTOFF   1.8  m/s
//   FOIL_HGT_FB  0.06 m
// Outer-loop height-PID defaults from spec; unused until PR5 wires them in.
#define AR_FOILCONTROL_HGT_P            1.5f
#define AR_FOILCONTROL_HGT_I            0.2f
#define AR_FOILCONTROL_HGT_D            0.4f
#define AR_FOILCONTROL_HGT_TARGET       0.10f
#define AR_FOILCONTROL_V_TAKEOFF        1.8f
#define AR_FOILCONTROL_HGT_FB_THRESH    0.06f

// PR4 inner-loop body-rate PID gains.
// 04-control-law.md §1.1–1.3 specifies *attitude*-loop Kp/Ki/Kd; PR4 closes only the
// body-rate inner loop (no outer θ/φ wrappers yet), so we use the Kd from the doc
// (which already has units of rad-flap per rad/s rate-error) as our rate-loop Kp.
// The attitude-Kp/Ki come back in PR5 wrapping these. Filter freqs zero/default
// for now — IMU noise band-limit is dominated by AHRS internals.
//   q-axis (pitch rate): Kp=0.19, Ki=0.02, Kd=0.0, IMAX=0.2, FF=0
//   p-axis (roll  rate): Kp=0.04, Ki=0.005,Kd=0.0, IMAX=0.2, FF=0
//   r-axis (yaw   rate): Kp=1.2,  Ki=0.3,  Kd=0.0, IMAX=0.4, FF=0
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

#define AR_FOILCONTROL_RATE_FILT_T_HZ   0.00f
#define AR_FOILCONTROL_RATE_FILT_E_HZ   0.00f
#define AR_FOILCONTROL_RATE_FILT_D_HZ   20.0f

// pilot stick scale: full deflection maps to ±2 rad/s body-rate setpoint.
#define AR_FOILCONTROL_PILOT_RATE_SCALE 2.0f

// mixer constants per 04-control-law.md §2.1:
//   canard_flap = +1.0 * pitch_mix_cmd, main_flap = -0.40 * pitch_mix_cmd
#define AR_FOILCONTROL_MIX_K_PC          1.00f
#define AR_FOILCONTROL_MIX_K_PM          0.40f

// §5.1 mechanical limit on each flap channel (±25° = ±0.436 rad).
#define AR_FOILCONTROL_FLAP_LIMIT_RAD    0.436f

// SRV_Channels::set_angle takes uint16_t centidegree half-range.
// ±25° = ±2500 centideg, matching plant's ±500 PWM at PWM 1500 = 0°.
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

    // Indices 4..5 reserved for PR5: _phi_rate_pid (roll-attitude outer wrapper)
    // and _v_pid (forward-speed PID).

    // @Param: HGT_P
    // @DisplayName: Foil height control P gain
    // @Description: Outer-loop height-error to pitch-rate P gain (PR5 hookup)
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_P",       6, AR_FoilControl, _h_p, AR_FOILCONTROL_HGT_P),

    // @Param: HGT_I
    // @DisplayName: Foil height control I gain
    // @Description: Outer-loop height-error to pitch-rate I gain (PR5 hookup)
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_I",       7, AR_FoilControl, _h_i, AR_FOILCONTROL_HGT_I),

    // @Param: HGT_D
    // @DisplayName: Foil height control D gain
    // @Description: Outer-loop height-error to pitch-rate D gain (PR5 hookup)
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
    _height_target_m(AR_FOILCONTROL_HGT_TARGET),
    _speed_target_ms(0.0f),
    _heading_target_rad(0.0f),
    _canard_cmd_rad(0.0f),
    _main_cmd_rad(0.0f),
    _rudder_cmd_rad(0.0f),
    _last_inner_us(0),
    _servo_ranges_set(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AR_FoilControl::init()
{
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

// PR2: AHRS/rangefinder plumbing. Inner/outer control bodies still no-op
// (real control wiring lands in PR4 inner / PR5 outer+throttle / PR6 failsafe).
// Behaviour remains unchanged from baseline rover.
// Note: design doc §1 listed `float dt` parameters; the AP_Scheduler SCHED_TASK_CLASS
// macro requires `void()` signatures, so PR4+ will compute dt internally.

// Read the downward-facing rangefinder. The SITL JSON backend (RNGFND1_TYPE=100,
// RNGFND1_ORIENT=25 / PITCH_270) is fed by `rng_1` from foilboat_sim.py and
// reports height above water in metres. Returns NaN when no healthy instance
// is found (rangefinder absent, out-of-range, dropout) so the caller can
// distinguish "no data" from a real zero-height reading.
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

void AR_FoilControl::update_outer()
{
    // PR2: exercise the rangefinder path so PR4+ can rely on it.
    // Read + log + discard. No control output yet (PR5).
    const float height_m = get_height_above_water();
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: FOIL
    // @Description: AR_FoilControl outer-loop telemetry
    // @Field: TimeUS: Time since system startup
    // @Field: Hgt: Height above water from downward rangefinder (m), NaN if unhealthy
    AP::logger().WriteStreaming("FOIL",
                                "TimeUS,Hgt",
                                "sm",
                                "F0",
                                "Qf",
                                AP_HAL::micros64(),
                                height_m);
#else
    (void)height_m;
#endif
}

void AR_FoilControl::update_inner()
{
    // Compute dt from monotonic clock. Clamp to [1 ms, 10 ms] — 400 Hz nominal
    // is 2.5 ms; outside that window something is wrong and we de-rate to the
    // upper clamp rather than blow integration step.
    const uint32_t now_us = AP_HAL::micros();
    float dt;
    if (_last_inner_us == 0) {
        dt = 0.0025f;  // first call: assume nominal 400 Hz
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

    // Body rates p, q, r in rad/s (Vector3f.x=p, .y=q, .z=r — standard AHRS convention).
    const Vector3f gyro = AP::ahrs().get_gyro();

    // Pilot body-rate setpoints from RC sticks (PR4 has no outer loop yet).
    // norm_input_dz() returns -1..+1 with deadzone applied; ±1 maps to ±2 rad/s.
    // Sign: positive stick deflection = positive body rate in the conventional sense.
    float p_desired = 0.0f;
    float q_desired = 0.0f;
    float r_desired = 0.0f;
    if (rc().has_valid_input()) {
        p_desired = rc().get_roll_channel().norm_input_dz()  * AR_FOILCONTROL_PILOT_RATE_SCALE;
        q_desired = rc().get_pitch_channel().norm_input_dz() * AR_FOILCONTROL_PILOT_RATE_SCALE;
        r_desired = rc().get_yaw_channel().norm_input_dz()   * AR_FOILCONTROL_PILOT_RATE_SCALE;
    }

    // Run three body-rate PIDs. update_all(target, measurement, dt) is the AC_PID
    // canonical inner-loop entrypoint (matches AR_AttitudeControl::get_steering_out_rate).
    const float p_out = _p_rate_pid.update_all(p_desired, gyro.x, dt);
    const float q_out = _q_rate_pid.update_all(q_desired, gyro.y, dt);
    const float r_out = _r_rate_pid.update_all(r_desired, gyro.z, dt);

    // Mixer per 04-control-law.md §2:
    //   canard_flap     = +k_pc · pitch_mix_cmd
    //   main_flap       = -k_pm · pitch_mix_cmd     (sign-conjugate so net lift is conserved)
    //   rudder_flap     = rudder_steer_cmd
    // Roll (p_out) is unused in PR4 — single main-surface rig has no differential channel.
    // Deferred to PR5 once roll-attitude wrapper lands. p_out still flows through the
    // PID so the integrator stays warm and logging shows it.
    (void)p_out;

    float canard_cmd = +AR_FOILCONTROL_MIX_K_PC * q_out;
    float main_cmd   = -AR_FOILCONTROL_MIX_K_PM * q_out;
    float rudder_cmd =  r_out;

    // §5.1 mechanical clamp per channel (±25° = ±0.436 rad).
    _canard_cmd_rad = constrain_float(canard_cmd, -AR_FOILCONTROL_FLAP_LIMIT_RAD, AR_FOILCONTROL_FLAP_LIMIT_RAD);
    _main_cmd_rad   = constrain_float(main_cmd,   -AR_FOILCONTROL_FLAP_LIMIT_RAD, AR_FOILCONTROL_FLAP_LIMIT_RAD);
    _rudder_cmd_rad = constrain_float(rudder_cmd, -AR_FOILCONTROL_FLAP_LIMIT_RAD, AR_FOILCONTROL_FLAP_LIMIT_RAD);
}

void AR_FoilControl::update_throttle()
{
    return;
}

void AR_FoilControl::update_failsafe()
{
    return;
}

void AR_FoilControl::output_to_servos()
{
    // Lazy-init servo ranges; cheap and idempotent.
    init();

    // Convert radians → centidegree for SRV_Channels::set_output_scaled().
    // The set_angle(2500) call configured each channel's scale so that
    // ±2500 centideg == ±25° == ±500 PWM at PWM 1500 = 0°.
    constexpr float RAD_TO_CD = 18000.0f / M_PI;
    SRV_Channels::set_output_scaled(SRV_Channel::k_foilcontrol_canard, _canard_cmd_rad * RAD_TO_CD);
    SRV_Channels::set_output_scaled(SRV_Channel::k_foilcontrol_main,   _main_cmd_rad   * RAD_TO_CD);
    SRV_Channels::set_output_scaled(SRV_Channel::k_foilcontrol_rudder, _rudder_cmd_rad * RAD_TO_CD);
}
