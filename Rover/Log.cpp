#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#if HAL_LOGGING_ENABLED

// Write an attitude packet
void Rover::Log_Write_Attitude()
{
    float desired_pitch = degrees(g2.attitude_control.get_desired_pitch());
    const Vector3f targets(0.0f, desired_pitch, 0.0f);

    ahrs.Write_Attitude(targets);

    AP::ahrs().Log_Write();

    // log steering rate controller
    logger.Write_PID(LOG_PIDS_MSG, g2.attitude_control.get_steering_rate_pid().get_pid_info());
    logger.Write_PID(LOG_PIDA_MSG, g2.attitude_control.get_throttle_speed_pid_info());

    // log pitch control for balance bots
    if (is_balancebot()) {
        logger.Write_PID(LOG_PIDP_MSG, g2.attitude_control.get_pitch_to_throttle_pid().get_pid_info());
    }

    // log heel to sail control for sailboats
    if (g2.sailboat.sail_enabled()) {
        logger.Write_PID(LOG_PIDR_MSG, g2.attitude_control.get_sailboat_heel_pid().get_pid_info());
    }

    // log wheel rate controller instance 0
    if (g2.wheel_rate_control.enabled(0)) {
        logger.Write_PID(LOG_PIDW_MSG, g2.wheel_rate_control.get_pid(0).get_pid_info());
    }
}

#if AP_RANGEFINDER_ENABLED
// Write a range finder depth message
void Rover::Log_Write_Depth()
{
    // only log depth on boats
    if (!rover.is_boat() || !rangefinder.has_orientation(ROTATION_PITCH_270)) {
        return;
    }

    // get position
    Location loc;
    IGNORE_RETURN(ahrs.get_location(loc));

    for (uint8_t i=0; i<rangefinder.num_sensors(); i++) {
        const AP_RangeFinder_Backend *s = rangefinder.get_backend(i);
        
        if (s == nullptr || s->orientation() != ROTATION_PITCH_270 || !s->has_data()) {
            continue;
        }

        // check if new sensor reading has arrived
        const uint32_t reading_ms = s->last_reading_ms();
        if (reading_ms == rangefinder_last_reading_ms[i]) {
            continue;
        }
        rangefinder_last_reading_ms[i] = reading_ms;

        float temp_C;
        if (!s->get_temp(temp_C)) {
            temp_C = 0.0f;
        }

        // @LoggerMessage: DPTH
        // @Description: Depth messages on boats with downwards facing range finder
        // @Field: TimeUS: Time since system startup
        // @Field: Inst: Instance
        // @Field: Lat: Latitude 
        // @Field: Lng: Longitude   
        // @Field: Depth: Depth as detected by the sensor
        // @Field: Temp: Temperature

        logger.Write("DPTH", "TimeUS,Inst,Lat,Lng,Depth,Temp",
                            "s#DUmO", "F-GG00", "QBLLff",
                            AP_HAL::micros64(),
                            i,
                            loc.lat,
                            loc.lng,
                            (double)(s->distance()),
                            temp_C);
    }
}
#endif

// guided mode logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    float wp_bearing;
    float nav_bearing;
    uint16_t yaw;
    float xtrack_error;
};

// Write a navigation tuning packet
void Rover::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : control_mode->get_distance_to_destination(),
        wp_bearing          : control_mode->wp_bearing(),
        nav_bearing         : control_mode->nav_bearing(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        xtrack_error        : control_mode->crosstrack_error_m()
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_Sail()
{
    // only log sail if present
    if (!g2.sailboat.sail_enabled()) {
        return;
    }

    float wind_dir_tack = AP_Logger::quiet_nanf();
    uint8_t current_tack = 0;
    if (g2.windvane.enabled()) {
        wind_dir_tack = degrees(g2.windvane.get_tack_threshold_wind_dir_rad());
        current_tack = uint8_t(g2.windvane.get_current_tack());
    }

// @LoggerMessage: SAIL
// @Description: Sailboat information
// @Field: TimeUS: Time since system startup
// @Field: Tack: Current tack, 0 = port, 1 = starboard
// @Field: TackThr: Apparent wind angle used for tack threshold
// @Field: MainOut: Normalized mainsail output
// @Field: WingOut: Normalized wingsail output
// @Field: MastRotOut: Normalized direct-rotation mast output
// @Field: VMG: Velocity made good (speed at which vehicle is making progress directly towards destination)

    logger.Write("SAIL", "TimeUS,Tack,TackThr,MainOut,WingOut,MastRotOut,VMG",
                        "s-d%%%n", "F000000", "QBfffff",
                        AP_HAL::micros64(),
                        current_tack,
                        (double)wind_dir_tack,
                        (double)g2.motors.get_mainsail(),
                        (double)g2.motors.get_wingsail(),
                        (double)g2.motors.get_mast_rotation(),
                        (double)g2.sailboat.get_VMG());
}

struct PACKED log_Steering {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t steering_in;
    float steering_out;
    float desired_lat_accel;
    float lat_accel;
    float desired_turn_rate;
    float turn_rate;
};

// Write a steering packet
void Rover::Log_Write_Steering()
{
    float lat_accel = AP_Logger::quiet_nanf();
    g2.attitude_control.get_lat_accel(lat_accel);
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_us        : AP_HAL::micros64(),
        steering_in        : channel_steer->get_control_in(),
        steering_out       : g2.motors.get_steering(),
        desired_lat_accel  : control_mode->get_desired_lat_accel(),
        lat_accel          : lat_accel,
        desired_turn_rate  : degrees(g2.attitude_control.get_desired_turn_rate()),
        turn_rate          : degrees(ahrs.get_yaw_rate_earth())
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Throttle {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t throttle_in;
    float throttle_out;
    float desired_speed;
    float speed;
    float accel_x;
};

// PR7a D2: AR_FoilControl logging packets.
//
// FOI  — outer-loop telemetry (100 Hz cadence, downsampled to 10 Hz here).
// FOI2 — PR6 schedule / pre-load / saturation duty (10 Hz).
// FOI3 — D1 one-shot AUTO_DESCEND-request event (fires once per latch).
struct PACKED log_Foil_Outer {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    hgt;
    float    theta_cmd_rad;
    float    p_setpoint;
    float    r_setpoint;
};

struct PACKED log_Foil_Sched {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    canard_preload_rad;
    float    sched_scale;
    float    sat_duty_cycle;
    uint8_t  sat_armed;
    uint8_t  sat_trigger_armed;
};

struct PACKED log_Foil_Event {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    sat_duty_cycle;
    uint32_t dwell_ms;
    float    groundspeed;
    float    engage_factor;
};

// PR14a D2: FoilboatFailsafe streaming state-machine packet (FOI4).
//
// Cadence 10 Hz from update_logging1 (PR7a precedent). Fields per synthesis
// §4 plus a deliberate uint16_t→uint32_t widening on the mask fields: the
// canonical FoilFault enum has 25 bit positions (rows S1..H1, synthesis §2)
// which does not fit in 16 bits. The synthesis table notates the masks as
// "H" but predated the cross-review enum growth to 25 rows. Using `I` here
// preserves 1-bit-per-row addressability for the full matrix.
struct PACKED log_Foilboat_Failsafe {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t fault_mask;
    uint32_t armed_mask;
    uint8_t  response;
    uint8_t  severity;
};

// Write a throttle control packet
void Rover::Log_Write_Throttle()
{
    const Vector3f accel = ins.get_accel();
    float speed = AP_Logger::quiet_nanf();
    g2.attitude_control.get_forward_speed(speed);
    struct log_Throttle pkt = {
        LOG_PACKET_HEADER_INIT(LOG_THR_MSG),
        time_us         : AP_HAL::micros64(),
        throttle_in     : channel_throttle->get_control_in(),
        throttle_out    : g2.motors.get_throttle(),
        desired_speed   : g2.attitude_control.get_desired_speed(),
        speed           : speed,
        accel_x         : accel.x
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// PR7a D2: Emit FOI/FOI2 streaming records + FOI3 one-shot event.
//
// Pull-style: AR_FoilControl exposes the field values via const getters; the
// controller library does not touch AP::logger().  Called from Rover's 10 Hz
// update_logging loop.
void Rover::Log_Write_Foil(void)
{
    // FOI — outer-loop telemetry.
    struct log_Foil_Outer pkt_outer = {
        LOG_PACKET_HEADER_INIT(LOG_FOI_MSG),
        time_us           : AP_HAL::micros64(),
        hgt               : g2.foil_control.get_height_above_water(),
        theta_cmd_rad     : g2.foil_control.get_theta_cmd_rad(),
        p_setpoint        : g2.foil_control.get_p_setpoint_rad_s(),
        r_setpoint        : g2.foil_control.get_r_setpoint_rad_s(),
    };
    logger.WriteBlock(&pkt_outer, sizeof(pkt_outer));

    // FOI2 — schedule + pre-load + saturation duty.
    struct log_Foil_Sched pkt_sched = {
        LOG_PACKET_HEADER_INIT(LOG_FOI2_MSG),
        time_us            : AP_HAL::micros64(),
        canard_preload_rad : g2.foil_control.get_canard_preload_last(),
        sched_scale        : g2.foil_control.get_last_gain_scale(),
        sat_duty_cycle     : g2.foil_control.saturation_duty(),
        sat_armed          : (uint8_t)g2.foil_control.get_sat_armed(),
        sat_trigger_armed  : (uint8_t)g2.foil_control.get_sat_trigger_armed(),
    };
    logger.WriteBlock(&pkt_sched, sizeof(pkt_sched));

    // FOI3 — one-shot AUTO_DESCEND-request event (fires once per latch).
    if (g2.foil_control.consume_failsafe_event_log_pending()) {
        struct log_Foil_Event pkt_evt = {
            LOG_PACKET_HEADER_INIT(LOG_FOI3_MSG),
            time_us         : AP_HAL::micros64(),
            sat_duty_cycle  : g2.foil_control.saturation_duty(),
            dwell_ms        : g2.foil_control.get_failsafe_dwell_ms(),
            groundspeed     : AP::ahrs().groundspeed(),
            engage_factor   : g2.foil_control.engage_factor(),
        };
        logger.WriteBlock(&pkt_evt, sizeof(pkt_evt));
    }
}

// PR14a D2: Emit the FoilboatFailsafe FOI4 streaming record (10 Hz).
//
// Pull-style: FoilboatFailsafe exposes the fields via const getters; the
// failsafe class does not reach into AP::logger().  Called from Rover's 10 Hz
// update_logging1 path immediately after Log_Write_Foil() so FOI/FOI2/FOI3
// and FOI4 share a timestamp epoch.
void Rover::Log_Write_Foilboat_Failsafe(void)
{
    struct log_Foilboat_Failsafe pkt = {
        LOG_PACKET_HEADER_INIT(LOG_FOI4_MSG),
        time_us    : AP_HAL::micros64(),
        fault_mask : foilboat_failsafe.active_fault_mask(),
        armed_mask : foilboat_failsafe.armed_fault_mask(),
        response   : (uint8_t)foilboat_failsafe.last_action(),
        severity   : foilboat_failsafe.severity(),
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
#if AP_RSSI_ENABLED
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
#endif
}

void Rover::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode((uint8_t)control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information

const LogStructure Rover::log_structure[] = {
    LOG_COMMON_STRUCTURES,

// @LoggerMessage: THR
// @Description: Throttle related messages
// @Field: TimeUS: Time since system startup
// @Field: ThrIn: Throttle Input
// @Field: ThrOut: Throttle Output 
// @Field: DesSpeed: Desired speed 
// @Field: Speed: Actual speed
// @Field: AccX: Acceleration

    { LOG_THR_MSG, sizeof(log_Throttle),
      "THR", "Qhffff", "TimeUS,ThrIn,ThrOut,DesSpeed,Speed,AccX", "s--nno", "F--000" },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @URL: http://ardupilot.org/rover/docs/navigation.html
// @Field: TimeUS: Time since system startup
// @Field: WpDist: distance to the current navigation waypoint
// @Field: WpBrg: bearing to the current navigation waypoint
// @Field: DesYaw: the vehicle's desired heading
// @Field: Yaw: the vehicle's current heading
// @Field: XTrack: the vehicle's current distance from the current travel segment

    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),
      "NTUN", "QfffHf", "TimeUS,WpDist,WpBrg,DesYaw,Yaw,XTrack", "smhhhm", "F000B0" },
    
// @LoggerMessage: STER
// @Description: Steering related messages
// @Field: TimeUS: Time since system startup
// @Field: SteerIn: Steering input
// @Field: SteerOut: Normalized steering output 
// @Field: DesLatAcc: Desired lateral acceleration
// @Field: LatAcc: Actual lateral acceleration
// @Field: DesTurnRate: Desired turn rate
// @Field: TurnRate: Actual turn rate
    
    { LOG_STEERING_MSG, sizeof(log_Steering),
      "STER", "Qhfffff",   "TimeUS,SteerIn,SteerOut,DesLatAcc,LatAcc,DesTurnRate,TurnRate", "s--ookk", "F--0000" },

// @LoggerMessage: GUIP
// @Description: Guided mode target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
    
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUIP",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },

// @LoggerMessage: FOI
// @Description: AR_FoilControl outer-loop telemetry
// @Field: TimeUS: Time since system startup
// @Field: Hgt: Height above water from downward rangefinder (m), NaN if unhealthy
// @Field: ThC: Pitch-cmd from height loop (rad)
// @Field: PSet: Roll-rate target (rad/s)
// @Field: RSet: Yaw-rate target (rad/s)

    { LOG_FOI_MSG, sizeof(log_Foil_Outer),
      "FOI", "Qffff", "TimeUS,Hgt,ThC,PSet,RSet", "smrrr", "F0000", true },

// @LoggerMessage: FOI2
// @Description: AR_FoilControl PR6 schedule + pre-load + saturation duty
// @Field: TimeUS: Time since system startup
// @Field: Preld: Canard pre-load applied this tick (rad)
// @Field: SchS: Active V^2 gain-schedule scale (multiplier on nominal gains)
// @Field: Duty: Saturation duty cycle over the 5 s rolling window (0..1)
// @Field: SArm: Failsafe arming gate (foilborne + settled), 0/1
// @Field: STrg: Duty-cycle saturation trigger (SArm && Duty > FOIL_SAT_DUTY_THR), 0/1

    { LOG_FOI2_MSG, sizeof(log_Foil_Sched),
      "FOI2", "QfffBB", "TimeUS,Preld,SchS,Duty,SArm,STrg", "sr-r--", "F0000-", true },

// @LoggerMessage: FOI3
// @Description: AR_FoilControl one-shot AUTO_DESCEND request event
// @Field: TimeUS: Time since system startup
// @Field: Duty: Saturation duty cycle at the latching tick (0..1)
// @Field: DwlMs: FOIL_FAIL_DWELL counter at latch (ms)
// @Field: V: Groundspeed at latch (m/s)
// @Field: Eng: Soft-engage factor at latch (0..1)

    { LOG_FOI3_MSG, sizeof(log_Foil_Event),
      "FOI3", "QfIff", "TimeUS,Duty,DwlMs,V,Eng", "s-snn", "F-000" },

// @LoggerMessage: FOI4
// @Description: FoilboatFailsafe state-machine streaming (10 Hz, PR14a D2).
// @Field: TimeUS: Time since system startup
// @Field: Fault: Bitmask of FoilFault rows currently latched. Bit positions
//                match the synthesis §2 table (S1=0, ..., H1=24).  Widened
//                from `H` to `I` versus the brief: the canonical 25-row enum
//                does not fit in 16 bits.
// @Field: Armed: Bitmask of FoilFault rows whose dwell counter is ticking
//                (early-warning view).  Same bit layout as `Fault`.
// @Field: Resp:  Dispatched FoilFailsafeAction this tick.
//                0=STAY, 1=REVERT_HULL_BORNE, 2=AUTO_DESCEND, 3=MOTOR_OFF.
// @Field: Sev:   Severity of the dispatched action.  0=INFO (STAY),
//                1=WARN (REVERT_HULL_BORNE), 2=CRIT (AUTO_DESCEND / MOTOR_OFF).

    { LOG_FOI4_MSG, sizeof(log_Foilboat_Failsafe),
      "FOI4", "QIIBB", "TimeUS,Fault,Armed,Resp,Sev", "s----", "F----", true },
};

uint8_t Rover::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // LOGGING_ENABLED
