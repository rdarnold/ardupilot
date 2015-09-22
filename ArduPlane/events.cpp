#include "Plane.h"

void Plane::failsafe_short_on_event(enum failsafe_state fstype, mode_reason_t reason)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.ch3_timer_ms = millis();
    gcs_send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on, ");
    switch(control_mode)
    {
#if AP_ACS_USE != TRUE
    case MANUAL:
#endif
    case STABILIZE:
    case ACRO:
#if AP_ACS_USE != TRUE
    case FLY_BY_WIRE_A:
#endif
    case AUTOTUNE:
#if AP_ACS_USE != TRUE
    case FLY_BY_WIRE_B:
#endif
    case CRUISE:
    case TRAINING:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = 1;
        if(g.short_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else {
            set_mode(CIRCLE, reason);
        }
        break;

    case QSTABILIZE:
    case QLOITER:
    case QHOVER:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = 1;
        set_mode(QLAND, reason);
        break;
        
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
        if(g.short_fs_action != 0) {
            failsafe.saved_mode = control_mode;
            failsafe.saved_mode_set = 1;
            if(g.short_fs_action == 2) {
                set_mode(FLY_BY_WIRE_A, reason);
            } else {
                set_mode(CIRCLE, reason);
            }
        }
        break;

    case CIRCLE:
    case RTL:
    case QLAND:
    case QRTL:
    default:
        break;
    }
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype, mode_reason_t reason)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs_send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on, ");
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    failsafe.state = fstype;
    switch(control_mode)
    {
#if AP_ACS_USE != TRUE
    case MANUAL:
#endif
    case STABILIZE:
    case ACRO:
#if AP_ACS_USE != TRUE
    case FLY_BY_WIRE_A:
#endif
    case AUTOTUNE:
#if AP_ACS_USE != TRUE
    case FLY_BY_WIRE_B:
#endif
    case CRUISE:
    case TRAINING:
    case CIRCLE:
        if(g.long_fs_action == 3) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.long_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else {
            set_mode(RTL, reason);
        }
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
        set_mode(QLAND, reason);
        break;
        
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
        if(g.long_fs_action == 3) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.long_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else if (g.long_fs_action == 1) {
            set_mode(RTL, reason);
        }
        break;

    case RTL:
    case QLAND:
    case QRTL:
    default:
        break;
    }
    if (fstype == FAILSAFE_GCS) {
        gcs_send_text(MAV_SEVERITY_CRITICAL, "No GCS heartbeat");
    }
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_short_off_event(mode_reason_t reason)
{
    // We're back in radio contact
    gcs_send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off");
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == CIRCLE && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = 0;
        set_mode(failsafe.saved_mode, reason);
    }
}

void Plane::low_battery_event(void)
{
#if AP_ACS_USE == TRUE
    if (control_mode == MANUAL || control_mode == FLY_BY_WIRE_B || control_mode == FLY_BY_WIRE_A) return;
#endif

    if (failsafe.low_battery) {
        return;
    }
    gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Low battery %.2fV used %.0f mAh",
                      (double)battery.voltage(), (double)battery.current_total_mah());
    if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
#if AP_ACS_USE == TRUE
        gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Battery low: auto-landing."));

        //start landing if not already (ACS-specific behavior -- land vice RTL)
        if (! jump_to_landing_sequence()) {
            gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Failed to start emergency land sequence!!"));
        }
#else
    	set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE);
    	aparm.throttle_cruise.load();
#endif //AP_ACS_USE == TRUE
    }

    failsafe.low_battery = true;
    AP_Notify::flags.failsafe_battery = true;
}

void Plane::update_events(void)
{
    ServoRelayEvents.update_events();
}
