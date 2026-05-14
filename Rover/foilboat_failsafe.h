#pragma once

#include <AP_Param/AP_Param.h>
#include <stdint.h>

/*
  FoilboatFailsafe — W6 failsafe-matrix dispatcher (per
  research/flight-dev-rig/06-w6-synthesis.md §3.4 + §5).

  PR14a scope (infrastructure only): class skeleton, scheduler hookup, FOI4
  LogStructure, ModeReason::FOILBOAT_FAILSAFE, AR_FoilControl getter migration.
  Exactly ONE fault detector is wired in this PR — A3 (FLAP_RATE_SAT) which
  consumes the existing PR7a `AR_FoilControl::should_failsafe_descend()`
  saturation-duty latch.  This preserves W5 behaviour through the migrated
  poll path without introducing a regression.  The remaining 22 fault rows are
  placeholders — their detectors land in PR14b.  The h_agl voter, V_fused
  complementary filter, and EKF lane-flap detector land in PR14c.

  Dispatch rule (§5.1): all AUTO_DESCEND requests go through
  FoilboatFailsafe::should_descend().  ModeFoilborneHold::update() no longer
  polls AR_FoilControl directly.

  Severity mapping (§1 D7):
    STAY              → INFO (0)
    REVERT_HULL_BORNE → WARN (1)
    AUTO_DESCEND      → CRIT (2)
    MOTOR_OFF         → CRIT (2)

  Logged at 10 Hz via FOI4 (Rover/Log.cpp).
*/

class Rover;  // forward-declare; defined in Rover.h

// Failsafe response taxonomy — synthesis §1 D1 canonical names.
enum class FoilFailsafeAction : uint8_t {
    STAY               = 0,
    REVERT_HULL_BORNE  = 1,
    AUTO_DESCEND       = 2,
    MOTOR_OFF          = 3,
};

// Fault bit positions — synthesis §2, 23 rows.  Bit position == matrix order
// so FOI4's `Fault` mask is directly readable against the table.
enum class FoilFault : uint32_t {
    IMU_DISAGREE_SOFT   = 0,
    IMU_DISAGREE_HARD   = 1,
    LIDAR_DROP          = 2,
    LIDAR_OUT_OF_RANGE  = 3,
    ULTRASONIC_DROP     = 4,
    H_AGL_DISAGREE      = 5,
    EKF_LANE_FLAP       = 6,
    H_AGL_BOTH_INVALID  = 7,
    SERVO_STUCK         = 8,
    SERVO_RUNAWAY       = 9,
    FLAP_RATE_SAT       = 10,
    MOTOR_STALL         = 11,
    ESC_OVERCURRENT     = 12,
    BATT_LOW_SOC        = 13,
    BATT_CELL_IMBAL     = 14,
    ESC_UNDERVOLTAGE    = 15,
    RC_LINK_LOSS        = 16,
    TELEM_LOSS          = 17,
    GS_HEARTBEAT_LOSS   = 18,
    INTEGRATOR_WIND_UP  = 19,
    ATT_DIVERGE         = 20,
    TUMBLE_CLAMP        = 21,
    V_OVER              = 22,
    V_UNDER_FOILBORNE   = 23,
    HULL_INGRESS        = 24,
};

static_assert(uint32_t(FoilFault::HULL_INGRESS) < 32,
              "FoilFault bit positions must fit in the FOI4 32-bit Fault/Armed masks");

class FoilboatFailsafe {
public:
    FoilboatFailsafe(Rover &rover_ref);

    CLASS_NO_COPY(FoilboatFailsafe);

    // One-shot init.  Idempotent; safe to call repeatedly.
    void init();

    // 10 Hz scheduler entrypoint.  PR14a body wires A3 (FLAP_RATE_SAT) only.
    void check();

    // Dispatched-action queries (single-dispatch rule, synthesis §5.1).
    bool should_descend()           const { return _should_descend; }
    bool should_revert_hull_borne() const { return _should_revert_hb; }
    bool should_motor_off()         const { return _should_motor_off; }

    // Telemetry getters consumed by FOI4 in Rover/Log.cpp.
    uint32_t           active_fault_mask() const { return _fault_mask; }
    uint32_t           armed_fault_mask()  const { return _armed_mask; }
    FoilFailsafeAction last_action()       const { return _last_action; }
    uint8_t            severity()          const { return _severity; }

    // Parameter var table.
    static const struct AP_Param::GroupInfo var_info[];

private:
    Rover &_rover;

    // PR14a: A3 is the only wired detector.  Remaining 22 are placeholders.
    // Per-fault debounce counters / latches will land in PR14b (one block per
    // matrix row).  Keeping the struct empty here so PR14b is a pure addition.

    // Dispatch state (computed each check() tick).
    bool               _should_descend;
    bool               _should_revert_hb;
    bool               _should_motor_off;
    uint32_t           _fault_mask;     // latched faults
    uint32_t           _armed_mask;     // faults whose dwell counter is ticking
    FoilFailsafeAction _last_action;
    uint8_t            _severity;       // 0=INFO, 1=WARN, 2=CRIT

    // PR14a placeholder enable bits (default 0 per synthesis §9 open item 4).
    // PR14b/c will reference these to gate the corresponding detector.  Kept
    // here so the param table exists before the detectors are wired, which
    // lets operator EEPROM defaults migrate cleanly.
    AP_Int8  _laneflap_en;     // FFS_LANEFLAP_EN — EKF lane-flap (S7)
    AP_Int8  _both_invalid_en; // FFS_BOTH_INV_EN — H_AGL_BOTH_INVALID (S8)
    AP_Int8  _ingress_en;      // FFS_INGRESS_EN  — HULL_INGRESS (H1)

    // Bookkeeping.
    uint32_t _last_check_us;
};
