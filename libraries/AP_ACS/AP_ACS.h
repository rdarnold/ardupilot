#ifndef AP_ACS_H__
#define AP_ACS_H__
/* Naval Postgraduate School Aerial Combat Swarms module */

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Mission/AP_Mission.h>
#include <DataFlash/DataFlash.h>

class AP_ACS {
public:
    //TODO: modes should be in their own library so they aren't duplicated here.
    //I wonder if AP_Mission was supposed to do that.  It may need some help.
    typedef enum ACS_FlightMode {
        ACS_NONE          = -1,
        ACS_MANUAL        = 0,
        ACS_CIRCLE        = 1,
        ACS_STABILIZE     = 2,
        ACS_TRAINING      = 3,
        ACS_ACRO          = 4,
        ACS_FLY_BY_WIRE_A = 5,
        ACS_FLY_BY_WIRE_B = 6,
        ACS_CRUISE        = 7,
        ACS_AUTO          = 10,
        ACS_RTL           = 11,
        ACS_LOITER        = 12,
        ACS_GUIDED        = 15,
        ACS_INITIALISING  = 16
    } ACS_FlightMode;

    typedef enum FailsafeState {
        GPS_LONG_FS = 0,
        GPS_SHORT_FS,
        GPS_RECOVERING_FS,
        SEC_CONTROL_FS,
        BATTERY_CURR_FS,
        BATTERY_VOLT_FS,
        GEOFENCE_SECONDARY_FS,  //fence breach for too long: cut throttle
        GCS_AUTOLAND_FS,
        GCS_AUTOLAND_TOO_FAR_FS, //Autoland waypoint too far away: RTL instead
        MOTOR_FS,
        NO_FS,
        NO_COMPANION_COMPUTER_FS
    } FailsafeState;

    // for holding parameters
	static const struct AP_Param::GroupInfo var_info[];

    AP_ACS(const AP_BattMonitor* batt, const AP_Mission* miss);

    //returns true if the hearbeat was from a companion computer
    bool handle_heartbeat(mavlink_message_t* msg);

    // essentially returns a bool: if not 0, then DO kill throttle
    AP_Int8 get_kill_throttle();

    // essentially setting a bool: if not 0, then DO kill throttle
    void set_kill_throttle(int kt);

    FailsafeState get_current_fs_state() { return _current_fs_state; }

    //used when recovering from a GPS failsafe state to get back to 
    //the mode we were in before the failsafe triggered
    ACS_FlightMode get_previous_mode() { return _previous_mode; }

    //Retrun true if the throttle kill notify flag has been sent.
    //The intent of this method is to avoid spamming 
    //  "I'm killing the throttle" msgs. (send just 1 message)
    bool get_throttle_kill_notified();

    void set_throttle_kill_notified(bool tkn);

    //returns true if everything OK.
    //false if RTL should happen
    bool check(ACS_FlightMode mode, 
           AP_Vehicle::FixedWing::FlightStage flight_stage, const AP_AHRS& ahrs,
           int16_t thr_out,
           uint32_t last_heartbeat_ms, uint8_t num_gps_sats,
           bool fence_breached, bool is_flying);

#if AP_AHRS_NAVEKF_AVAILABLE
    void send_position_attitude_to_payload(AP_AHRS_NavEKF &ahrs,
            mavlink_channel_t chan, DataFlash_Class &df, bool is_flying);
#endif //AP_AHRS_NAVEKF_AVAILABLE

    void set_preland_started(bool do_start) { _preland_started = do_start; }

    bool preland_started() { return _preland_started; }

protected:
    //params
    AP_Int8             _watch_heartbeat;
    AP_Int8             _kill_throttle;
    AP_Int8             _aland_limit_km;

    uint32_t            _last_computer_heartbeat_ms;
    uint32_t            _fence_breach_time_ms;
     
    FailsafeState       _current_fs_state;
    
    ACS_FlightMode      _previous_mode;

    bool                _preland_started;
    bool                _thr_kill_notified;
    bool                _payload_failsafe_already_fired;

    const AP_BattMonitor*     _battery;
    uint32_t            _last_good_motor_time_ms;
    uint32_t            _motor_fail_workaround_start_ms;
    int                 _motor_restart_attempts;

    uint32_t            _last_log_time;
    uint32_t            _last_gps_fix_time_ms;

    const AP_Mission*   _mission;
};

#endif // AP_ACS_H__
