/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * flight.pde - high level calls to set and update flight modes
 *      logic for individual flight modes is in control_acro.pde, control_stabilize.pde, etc
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        return true;
    }

    switch(mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_acro_init(ignore_checks);
            #else
                success = acro_init(ignore_checks);
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_stabilize_init(ignore_checks);
            #else
                success = stabilize_init(ignore_checks);
            #endif
            break;

        case STAB_RUAS:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_stabilize_init(ignore_checks);
            #else
                success = stabilize_init(ignore_checks);
            #endif
            break;

        case ALT_HOLD:
            success = althold_init(ignore_checks);
            break;

        case AUTO:
            success = auto_init(ignore_checks);
            break;

        case AUTO_RUAS:
            success = auto_init(ignore_checks);
            break;

        case CIRCLE:
            success = circle_init(ignore_checks);
            break;

        case LOITER:
            success = loiter_init(ignore_checks);
            break;

        case GUIDED:
            success = guided_init(ignore_checks);
            break;

        case LAND:
            success = land_init(ignore_checks);
            break;

        case RTL:
            success = rtl_init(ignore_checks);
            break;

        case DRIFT:
            success = drift_init(ignore_checks);
            break;

        case SPORT:
            success = sport_init(ignore_checks);
            break;

        case FLIP:
            success = flip_init(ignore_checks);
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            success = autotune_init(ignore_checks);
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            success = poshold_init(ignore_checks);
            break;
#endif

        case BRAKE:
            success = brake_init(ignore_checks);
            break;

        case THROW:
            success = throw_init(ignore_checks);
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        control_mode = mode;
        DataFlash.Log_Write_Mode(control_mode);

#if AC_FENCE == ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
#endif
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // update notify object
    if (success) {
        notify_flight_mode(control_mode);
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
    // Update EKF speed limit - used to limit speed when we are using optical flow
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    switch (control_mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                heli_acro_run();
            #else
                acro_run();
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                heli_stabilize_run();
            #else
                stabilize_run();
            #endif
            break;

        case STAB_RUAS:
            #if FRAME_CONFIG == HELI_FRAME
                heli_stabilize_run_ruas();
            #else
                stabilize_run();
            #endif
            break;

        case ALT_HOLD:
            althold_run();
            break;

        case AUTO:
            auto_run();
            break;

        case AUTO_RUAS:
            auto_run_ruas();
            break;

        case CIRCLE:
            circle_run();
            break;

        case LOITER:
            loiter_run();
            break;

        case GUIDED:
            guided_run();
            break;

        case LAND:
            land_run();
            break;

        case RTL:
            rtl_run();
            break;

        case DRIFT:
            drift_run();
            break;

        case SPORT:
            sport_run();
            break;

        case FLIP:
            flip_run();
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            autotune_run();
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            poshold_run();
            break;
#endif

        case BRAKE:
            brake_run();
            break;

        case THROW:
            throw_run();
            break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(uint8_t old_control_mode, uint8_t new_control_mode)
{
#if AUTOTUNE_ENABLED == ENABLED
    if (old_control_mode == AUTOTUNE) {
        autotune_stop();
    }
#endif

    // stop mission when we leave auto mode
    if (old_control_mode == AUTO || old_control_mode == AUTO_RUAS) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }

    if (old_control_mode == THROW) {
        throw_exit();
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (mode_has_manual_throttle(old_control_mode) && !mode_has_manual_throttle(new_control_mode) && motors.armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(channel_throttle->control_in));
    }

    // cancel any takeoffs in progress
    takeoff_stop();

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_control_mode == ACRO) {
        attitude_control.use_flybar_passthrough(false, false);
        motors.set_acro_tail(false);
    }

    // if we are changing from a mode that did not use manual throttle,
    // stab col ramp value should be pre-loaded to the correct value to avoid a twitch
    // heli_stab_col_ramp should really only be active switching between Stabilize and Acro modes
    if (!mode_has_manual_throttle(old_control_mode)){
        if (new_control_mode == STABILIZE || new_control_mode == STAB_RUAS){
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_control_mode == ACRO){
            input_manager.set_stab_col_ramp(0.0);
        }
    }

    // reset RC Passthrough to motors
    motors.reset_radio_passthrough();
#endif //HELI_FRAME
}

// returns true or false whether mode requires GPS
bool Copter::mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case AUTO_RUAS:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
        case POSHOLD:
        case BRAKE:
        case THROW:
            return true;
        default:
            return false;
    }

    return false;
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
bool Copter::mode_has_manual_throttle(uint8_t mode) {
    switch(mode) {
        case ACRO:
        case STABILIZE:
        case STAB_RUAS:
            return true;
        default:
            return false;
    }

    return false;
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
bool Copter::mode_allows_arming(uint8_t mode, bool arming_from_gcs) {
    if (mode_has_manual_throttle(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || mode == DRIFT || mode == SPORT || mode == THROW || (arming_from_gcs && mode == GUIDED)) {
        return true;
    }
    return false;
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case AUTO_RUAS:
        case GUIDED:
        case RTL:
        case CIRCLE:
        case LAND:
            // autopilot modes
            AP_Notify::flags.autopilot_mode = true;
            break;
        default:
            // all other are manual flight modes
            AP_Notify::flags.autopilot_mode = false;
            break;
    }
}

//RUAS
void Copter::avoidance_maneuver()
{
  float safety_bubble = 500;            // 5 meters safety bubble around the red dragons helicopter
  float manouver_bubble = 650;          // where we start moving
  float avoidance_gain = 500;           // avoidance gain (arbitrary)

  float avoidance_accel_roll;         // acceleration in roll for avoidance, calculated based on distance, rel velocity, safety bubble, etc
  float avoidance_accel_pitch;        // acceleration in pitch for avoidance
  float response;


  if(abs(_trafic_angle) < 70 && _trafic_distance > 50 && _trafic_distance < 1000 ) {
    do_track_maneuver = true;
  } else {
    do_track_maneuver = false;  // if we will yaw (but not move) towards the traffic
  }


  if(abs(_trafic_angle) < 70 && _trafic_distance > 50 && _trafic_distance < 700)
  {
    do_avoid_maneuver = true;

    if(_trafic_distance < safety_bubble){_trafic_distance = safety_bubble;} // this prevents the limit from going to infinaty

    response = 10*(avoidance_gain)/(_trafic_distance*sqrt(safety_bubble)-1); // response parameter as calculated in the MATLAB simulations

    avoidance_accel_roll  = _rel_v.x * response; // response in roll
    //avoidance_accel_pitch = _rel_v.y * response; // respose in pitch
    avoidance_roll_angle_cd = atanf(avoidance_accel_roll/9.81);   // bank angle calculation
    avoidance_pitch_angle_cd = atanf(avoidance_accel_pitch/9.81); // pitch angle calculation

    // check if avoidance roll angle is more than 5 degrees (according to Jacob)
    if(abs(avoidance_roll_angle_cd) > 5) {avoidance_roll_angle_cd = 5;}

    // check if avoidance roll angle is more than 5 degrees (according to Jacob)
    if(abs(avoidance_pitch_angle_cd) > 5) {avoidance_pitch_angle_cd = 5;}

    // run simulation and find relationship between pitch angle and acceleration
    // check if the helicopter is coming from the left or right to determine if the roll should be left or right
    if(_rel_d.y < 0) { avoidance_roll_angle_cd = -avoidance_roll_angle_cd;}
    if(_rel_d.x < 0) { avoidance_pitch_angle_cd = -avoidance_pitch_angle_cd;}

    // convert angle to centi-degrees
    avoidance_roll_angle_cd *= -10000;
    avoidance_pitch_angle_cd *= -10000;
  } else {
    do_avoid_maneuver = false;
  }

  Log_Write_Avoidance(do_avoid_maneuver, avoidance_roll_angle_cd, avoidance_pitch_angle_cd, do_track_maneuver, trafic_angle * g.acro_yaw_p);

}

//
// print_flight_mode - prints flight mode to serial port.
//
void Copter::print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print("STABILIZE");
        break;
    case STAB_RUAS:
        port->print("STAB_RUAS");
        break;
    case ACRO:
        port->print("ACRO");
        break;
    case ALT_HOLD:
        port->print("ALT_HOLD");
        break;
    case AUTO:
        port->print("AUTO");
        break;
    case AUTO_RUAS:
        port->print("AUTO_RUAS");
        break;
    case GUIDED:
        port->print("GUIDED");
        break;
    case LOITER:
        port->print("LOITER");
        break;
    case RTL:
        port->print("RTL");
        break;
    case CIRCLE:
        port->print("CIRCLE");
        break;
    case LAND:
        port->print("LAND");
        break;
    case DRIFT:
        port->print("DRIFT");
        break;
    case SPORT:
        port->print("SPORT");
        break;
    case FLIP:
        port->print("FLIP");
        break;
    case AUTOTUNE:
        port->print("AUTOTUNE");
        break;
    case POSHOLD:
        port->print("POSHOLD");
        break;
    case BRAKE:
        port->print("BRAKE");
        break;
    case THROW:
        port->print("THROW");
        break;
    default:
        port->printf("Mode(%u)", (unsigned)mode);
        break;
    }
}
