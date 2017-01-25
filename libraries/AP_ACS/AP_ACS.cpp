#include <AP_ACS/AP_ACS.h>

#include <AP_Notify/AP_Notify.h>

//#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
# define Debug(fmt, args ...)
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_ACS::var_info[] = {
    // @Param: WATCH_HB
    // @DisplayName: Watch the payload heartbeat
    // @Description: If this setting is not 0 then the plane will RTL if 
    // it doesn't receive heartbeats from the payload companion computer.
    // @User: Advanced
    AP_GROUPINFO("WATCH_HB",     0, AP_ACS, _watch_heartbeat,    1),

    // @Param: KILL_THR
    // @DisplayName: Force kill throttle
    // @Description: Can be set to 1 in flight to force throttle to 0 in an emergency.
    // @User: Advanced
    AP_GROUPINFO("KILL_THR", 1, AP_ACS, _kill_throttle, 0),


    AP_GROUPEND
};

AP_ACS::AP_ACS(const AP_BattMonitor* batt) 
    : _last_computer_heartbeat_ms(0)
    , _fence_breach_time_ms(0)
    , _current_fs_state(NO_FS)
    , _previous_mode(ACS_NONE)
    , _preland_started(false)
    , _thr_kill_notified(false)
    , _payload_failsafe_already_fired(false)
    , _battery(batt)
    , _last_good_motor_time_ms(0)
    , _motor_fail_workaround_start_ms(0)
    , _motor_restart_attempts(0)
    , _last_log_time(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_ACS::handle_heartbeat(mavlink_message_t* msg) {
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(msg, &packet);

    if (packet.type == MAV_TYPE_ONBOARD_CONTROLLER) {
        _last_computer_heartbeat_ms = AP_HAL::millis();

        return true;
    }

    return false;
}

AP_Int8 AP_ACS::get_kill_throttle() {
    //kill throttle in GPS_LONG_FS or GEOFENCE_SECONDARY_FS 
    if (_current_fs_state == GPS_LONG_FS ||

        _current_fs_state == GEOFENCE_SECONDARY_FS) {
        AP_Int8 retVal;
        retVal.set(1);
        return retVal;
    }

    return _kill_throttle;
}

void AP_ACS::set_kill_throttle(int kt) {
    _kill_throttle = kt;

    //we've stopped notifying about a kill throttle if we're no 
    //longer killing the throttle.
    if (_kill_throttle == false) {
        _thr_kill_notified = false;
    }
}

//Retrun true if the throttle kill notify flag has been sent.
//The intent of this method is to avoid spamming "I'm killing throttle" msgs.
bool AP_ACS::get_throttle_kill_notified() {
    return _thr_kill_notified;
}

void AP_ACS::set_throttle_kill_notified(bool tkn) {
    _thr_kill_notified = tkn;
}

// check for failsafe conditions IN PRIORITY ORDER
bool AP_ACS::check(ACS_FlightMode mode, 
        AP_Vehicle::FixedWing::FlightStage flight_stage, 
        int16_t thr_out, uint32_t last_heartbeat_ms,
        uint32_t last_gps_fix_ms, bool fence_breached, bool is_flying) {

    uint32_t now = AP_HAL::millis();

    //always ignore failsafes in MANUAL modes
    if (mode == ACS_MANUAL || mode == ACS_FLY_BY_WIRE_B || mode == ACS_FLY_BY_WIRE_A) {
        _current_fs_state = NO_FS;

        return false;
    }

    //The failsafes are ignored during takeoff and landing approach
    //(failsafes triggered near the ground could cause the plane to veer at
    //very low altitude into personnel or property)

    if (flight_stage == AP_Vehicle::FixedWing::FlightStage::FLIGHT_TAKEOFF ||
        flight_stage == AP_Vehicle::FixedWing::FlightStage::FLIGHT_LAND_APPROACH || 
        flight_stage == AP_Vehicle::FixedWing::FlightStage::FLIGHT_LAND_FINAL) {
        
        //no longer in preland flight phase
        //(Need this line to re-enable battery, GCS, and payload heartbeat
        // failsafes after starting a preland)
        set_preland_started(false);

        _current_fs_state = NO_FS;
        return false;
    }

    //always check loss of GPS first
    if ((now - last_gps_fix_ms) > 20000) {
        if (_current_fs_state != GPS_LONG_FS) {
            hal.console->println("20 sec GPS FS");
        }

        _current_fs_state = GPS_LONG_FS;

        //actually killing throttle is handled in the
        //get_kill_throttle method
        //and the Arduplane code (see Attitude.pde)
        return false;
    } else if ((now - last_gps_fix_ms) >= 5000) {
        if (_current_fs_state != GPS_SHORT_FS) {
            hal.console->println("5 sec GPS FS");
            _previous_mode = mode;
        }

        _current_fs_state = GPS_SHORT_FS;
       return false; 
    } else {
        //GPS is not failing
        //If it was before we have to exit circle mode if we were in it.
        if (_current_fs_state == GPS_LONG_FS 
            || _current_fs_state == GPS_SHORT_FS 
            || _current_fs_state == GPS_RECOVERING_FS) {
            
            if (mode == ACS_LOITER) {
                //signal that we need to go back to previous flight mode
                _current_fs_state = GPS_RECOVERING_FS;
                return false;
            }
        }
    }

    //ignore all failsafes except GPS when not flying
    if (! is_flying) {
        _current_fs_state = NO_FS;
        return false;
    }

    //always check secondary fence 2nd.  If the fence has been breached
    //for too long, then we assume the plane's control is such that it
    //can't return.  In that case, the throttle will be cut.
    if (fence_breached) {
        if (_fence_breach_time_ms == 0) {
            _fence_breach_time_ms = now;
        }

        if (now - _fence_breach_time_ms > 20000) {
            _current_fs_state = GEOFENCE_SECONDARY_FS;

            //actually killing throtte is handled in the
            //get_kill_throttle method
            //and the ArduPlane code (see Attitude.pde)
            return false;
        }
    } else {
        //reset timer
        _fence_breach_time_ms = 0;
    }

    //Don't trigger any failsafe behavior except GPS and fence if 
    //we've started landing (but haven't yet started approach)
    //UNLESS: this is a failsafe induced landing.
    if (preland_started() && _current_fs_state != GCS_AUTOLAND_FS) {
        _current_fs_state = NO_FS;
        return false;
    }

    if (_battery != NULL) {
        //don't use motor failsafe if battery failsafe is active
        if (AP_Notify::flags.failsafe_battery) {
            _last_good_motor_time_ms = now;
        }

        //is throttle is above 60 and current is below 2 and
        //the failsafe workaround not currently active?
        if (_motor_fail_workaround_start_ms == 0 && 
                (thr_out < 60 || _battery->current_amps() >= 2.0f)) {
            _last_good_motor_time_ms = now;
        }

        //5 seconds since last good motor time?
        if (now - _last_good_motor_time_ms > 5000) {
            if (_motor_fail_workaround_start_ms == 0) {
                _motor_fail_workaround_start_ms = now;
            }

            //kill throttle for 2.5 seconds to attempt to revive the ESC
            if (_motor_restart_attempts < 4 &&  
                now - _motor_fail_workaround_start_ms <= 2500) {
                if (! get_kill_throttle()) {
                    set_kill_throttle(1);
                    _motor_restart_attempts++;
                }
            } else if (_motor_fail_workaround_start_ms != 0) {
                set_kill_throttle(0);
                _motor_fail_workaround_start_ms = 0;
            }

            _current_fs_state = MOTOR_FS;
            return false;
        }
    } 

    //next check loss of GCS comms (for auto landing)
    //If we haven't had any contact with the GCS for two minutes, then 
    //enter failsafe state
    if (now - last_heartbeat_ms > 120000) {
        _current_fs_state = GCS_AUTOLAND_FS;
        return false;
    }

    if (_watch_heartbeat != 0 &&
        now - _last_computer_heartbeat_ms > 20000) {
        //edge triggered when companion computer heartbeat lost
        if (! _payload_failsafe_already_fired) {
            _current_fs_state = NO_COMPANION_COMPUTER_FS;
            _payload_failsafe_already_fired = true;
            return false;
        } 
    } else {
        //if the hearbeat comes back, reset this flag
        _payload_failsafe_already_fired = false;
    }
  
    //if we made it here then no failsafes are in effect.
    _current_fs_state = NO_FS;

    return true;
}

#if AP_AHRS_NAVEKF_AVAILABLE
void AP_ACS::send_position_attitude_to_payload(AP_AHRS_NavEKF &ahrs, mavlink_channel_t chan, DataFlash_Class &df, bool is_flying) {
    struct Location loc;
    const struct Location home = ahrs.get_home();
    Matrix3f mat;
    Vector3f eulers;
    Quaternion quat;
    Vector3f velNED;
    Vector3f gyro;    
    uint8_t ekf_state = 0; //for dataflash logging
    uint32_t now = AP_HAL::millis();

    if (ahrs.get_NavEKF2().healthy()) {
        ahrs.get_NavEKF2().getEulerAngles(0, eulers);
        ahrs.get_NavEKF2().getVelNED(0, velNED);
        ahrs.get_NavEKF2().getLLH(loc);
        ekf_state = 1;
    } else {
        ahrs.get_rotation_body_to_ned().to_euler(&eulers.x, &eulers.y, &eulers.z);
        velNED = ahrs.get_gps().velocity();
        loc = ahrs.get_gps().location();
        ekf_state = 0;
    }
    
    quat.from_euler(eulers.x, eulers.y, eulers.z);

    gyro = ahrs.get_gyro();

    float pose[4];
    pose[0] = quat.q2; pose[1] = quat.q3; pose[2] = quat.q4; pose[3] = quat.q1;    
    mavlink_msg_global_pos_att_ned_send (chan,
                           AP_HAL::millis(),
                           loc.lat,
                           loc.lng,
                           loc.alt,
                           loc.alt - home.alt,
                           velNED.x*100.f,
                           velNED.y*100.f,
                           velNED.z*100.f,
                           pose,
                           gyro.x,
                           gyro.y,
                           gyro.z
                           );

    //log at a slower rate than we send to keep disk I/O down
    //and only log when flying
    if (is_flying && now - _last_log_time > 333) {
        //Log sending of message to dataflash
        struct log_payload_pose pkt = {
            LOG_PACKET_HEADER_INIT(LOG_PAYLOAD_POSE_MSG),
            time_us     : AP_HAL::micros64(),
            ekf_state   : ekf_state, 
            lat         : loc.lat,
            lng         : loc.lng,
            alt         : loc.alt,
            rel_alt     : (loc.alt - home.alt),
            vel_x       : velNED.x*100.f,
            vel_y       : velNED.y*100.f,
            vel_z       : velNED.z*100.f,
            pose_0      : pose[0],
            pose_1      : pose[1],
            pose_2      : pose[2],
            pose_3      : pose[3]
        };
        df.WriteBlock(&pkt, sizeof(pkt));

        _last_log_time = now;
    }
}
#endif //AP_AHRS_NAVEKF_AVAILABLE

