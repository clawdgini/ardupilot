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

// PR1 defaults (per research/flight-dev-rig/05-ardufoil-integration.md §5):
//   FOIL_HGT_TGT 0.10 m (100 mm ride height for 1 m rig / 4 m proto pre-tuning)
//   FOIL_VTOFF   1.8  m/s
//   FOIL_HGT_FB  0.06 m
// Outer-loop height-PID defaults from spec; unused until PR4/PR5 wires them in.
#define AR_FOILCONTROL_HGT_P            1.5f
#define AR_FOILCONTROL_HGT_I            0.2f
#define AR_FOILCONTROL_HGT_D            0.4f
#define AR_FOILCONTROL_HGT_TARGET       0.10f
#define AR_FOILCONTROL_V_TAKEOFF        1.8f
#define AR_FOILCONTROL_HGT_FB_THRESH    0.06f

AR_FoilControl *AR_FoilControl::_singleton;

const AP_Param::GroupInfo AR_FoilControl::var_info[] = {

    // TODO(PR4): AP_SUBGROUPINFO entries for the cascaded AC_PIDs, e.g.:
    //   AP_SUBGROUPINFO(_p_rate_pid,   "P_RAT_", 1, AR_FoilControl, AC_PID),
    //   AP_SUBGROUPINFO(_q_rate_pid,   "Q_RAT_", 2, AR_FoilControl, AC_PID),
    //   AP_SUBGROUPINFO(_r_rate_pid,   "R_RAT_", 3, AR_FoilControl, AC_PID),
    //   AP_SUBGROUPINFO(_phi_rate_pid, "ROL_",   4, AR_FoilControl, AC_PID),
    //   AP_SUBGROUPINFO(_v_pid,        "SPD_",   5, AR_FoilControl, AC_PID),
    // Indices 1..5 reserved for these subgroups.

    // @Param: HGT_P
    // @DisplayName: Foil height control P gain
    // @Description: Outer-loop height-error to pitch-rate P gain (PR4 hookup)
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_P",       6, AR_FoilControl, _h_p, AR_FOILCONTROL_HGT_P),

    // @Param: HGT_I
    // @DisplayName: Foil height control I gain
    // @Description: Outer-loop height-error to pitch-rate I gain (PR4 hookup)
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_I",       7, AR_FoilControl, _h_i, AR_FOILCONTROL_HGT_I),

    // @Param: HGT_D
    // @DisplayName: Foil height control D gain
    // @Description: Outer-loop height-error to pitch-rate D gain (PR4 hookup)
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
    _height_target_m(AR_FOILCONTROL_HGT_TARGET),
    _speed_target_ms(0.0f),
    _heading_target_rad(0.0f)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// PR1: empty bodies. Real control wiring lands in PR4 (inner), PR5 (outer/throttle),
// PR6 (failsafe). Behaviour is unchanged from baseline rover.
// Note: design doc §1 listed `float dt` parameters; the AP_Scheduler SCHED_TASK_CLASS
// macro requires `void()` signatures, so PR4+ will compute dt internally.
void AR_FoilControl::update_outer()
{
    return;
}

void AR_FoilControl::update_inner()
{
    return;
}

void AR_FoilControl::update_throttle()
{
    return;
}

void AR_FoilControl::update_failsafe()
{
    return;
}
