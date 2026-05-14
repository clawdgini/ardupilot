#include "Rover.h"

// FoilboatFailsafe — W6 failsafe-matrix dispatcher.  See the header for the
// design rationale and the synthesis-doc cross-references.
//
// PR14a wires A3 (FLAP_RATE_SAT) only.  This is the existing PR7a saturation-
// duty latch — `AR_FoilControl::should_failsafe_descend()` — re-routed through
// FoilboatFailsafe so ModeFoilborneHold no longer polls AR_FoilControl
// directly (synthesis §5.1 single-dispatch rule).  Behaviour is identical to
// PR12 from the boat's perspective; the only observable change is that FOI4
// now logs the latched A3 bit and the dispatched AUTO_DESCEND action.

const AP_Param::GroupInfo FoilboatFailsafe::var_info[] = {

    // @Param: LANEFLAP_EN
    // @DisplayName: Failsafe S7 EKF lane-flap detector enable
    // @Description: Gate for the S7 EKF_LANE_FLAP detector that PR14c will
    // wire.  PR14a registers the param so operator EEPROM migrates cleanly
    // when the detector lands; the detector body is empty until PR14c.
    // Default 0 per synthesis §9 open item 4 (threshold 3-switches-in-5-s is
    // unvalidated; flip to 1 after W7 sweep).
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LANEFLAP_EN", 1, FoilboatFailsafe, _laneflap_en, 0),

    // @Param: BOTH_INV_EN
    // @DisplayName: Failsafe S8 h_agl-both-invalid detector enable
    // @Description: Gate for the S8 H_AGL_BOTH_INVALID detector that PR14c
    // will wire (voter cannot produce a valid h source for >400 ms).  PR14a
    // registers the param so operator EEPROM migrates cleanly; the detector
    // body is empty until PR14c.  Default 0 — flip to 1 once PR14c lands the
    // voter.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("BOTH_INV_EN", 2, FoilboatFailsafe, _both_invalid_en, 0),

    // @Param: INGRESS_EN
    // @DisplayName: Failsafe H1 hull-ingress detector enable
    // @Description: Gate for the H1 HULL_INGRESS detector that PR14b will
    // wire (bilge digital input → MOTOR_OFF).  PR14a registers the param so
    // operator EEPROM migrates cleanly; the detector body is empty until
    // PR14b.  Default 0 — flip to 1 once the bilge digital-input plumbing
    // lands (W8 hardware integration).
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("INGRESS_EN",  3, FoilboatFailsafe, _ingress_en,      0),

    AP_GROUPEND
};

// Severity mapping per synthesis §1 D7.
static inline uint8_t severity_for(FoilFailsafeAction a)
{
    switch (a) {
    case FoilFailsafeAction::STAY:               return 0;  // INFO
    case FoilFailsafeAction::REVERT_HULL_BORNE:  return 1;  // WARN
    case FoilFailsafeAction::AUTO_DESCEND:       return 2;  // CRIT
    case FoilFailsafeAction::MOTOR_OFF:          return 2;  // CRIT
    }
    return 0;
}

FoilboatFailsafe::FoilboatFailsafe(Rover &rover_ref) :
    _rover(rover_ref),
    _should_descend(false),
    _should_revert_hb(false),
    _should_motor_off(false),
    _fault_mask(0),
    _armed_mask(0),
    _last_action(FoilFailsafeAction::STAY),
    _severity(0),
    _last_check_us(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void FoilboatFailsafe::init()
{
    // PR14a: nothing to initialise — debounce counters / latches land in
    // PR14b.  Reserved as the entrypoint for one-shot param caching once the
    // matrix grows beyond a single detector.
    _last_check_us = AP_HAL::micros();
}

void FoilboatFailsafe::check()
{
    _last_check_us = AP_HAL::micros();

    // Clear per-tick dispatch state; the (single, PR14a-wired) A3 detector
    // below re-asserts as appropriate.  PR14b will OR-reduce across all
    // detectors per synthesis §5.3 multi-fault resolution.
    uint32_t fault_mask = 0;
    uint32_t armed_mask = 0;
    FoilFailsafeAction action = FoilFailsafeAction::STAY;

    // --- A3 FLAP_RATE_SAT (synthesis §2 row A3) -----------------------------
    // PR7a's existing saturation-duty trigger lives inside AR_FoilControl.
    // Per synthesis §5.1, the *poll* migrates here so ModeFoilborneHold sees a
    // single dispatcher.  The PR7a trigger logic (FOIL_SAT_DUTY rolling-window
    // counter + FOIL_FAIL_DWELL dwell) is preserved untouched — we just
    // consume the latch.
    if (_rover.g2.foil_control.should_failsafe_descend()) {
        fault_mask |= (1U << uint32_t(FoilFault::FLAP_RATE_SAT));
        action = FoilFailsafeAction::AUTO_DESCEND;
    }

    // --- PR14a placeholder rows ---------------------------------------------
    // The remaining 22 detectors (synthesis §2) land in PR14b.  PR14c adds
    // the voter, V_fused complementary filter, and the EKF lane-flap (S7) /
    // h_agl-both-invalid (S8) rows that depend on it.  Each detector follows
    // the EKF-check idiom: read signal, compare against threshold, increment
    // dwell counter, latch on dwell exceed, contribute to `fault_mask` here.

    _fault_mask  = fault_mask;
    _armed_mask  = armed_mask;
    _last_action = action;
    _severity    = severity_for(action);

    _should_descend   = (action == FoilFailsafeAction::AUTO_DESCEND);
    _should_revert_hb = (action == FoilFailsafeAction::REVERT_HULL_BORNE);
    _should_motor_off = (action == FoilFailsafeAction::MOTOR_OFF);
}
