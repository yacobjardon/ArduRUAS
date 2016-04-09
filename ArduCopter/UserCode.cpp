/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{

      Vector3f new_d = read_rel_location();
      Vector3f new_v = read_rel_velocity();
      float new_distance = pythagorous2(new_d.x, new_d.y);

      rel_v.x = 25 * new_d.x / new_distance;//100/new_d.y;
      rel_v.y = 25 * new_d.y / new_distance;//100/new_d.x; //these are static for testing, delete in flight
      rel_v.z = 0;

  /*
      //if no new update, assume object is moving away and slowing
      if(trafic_distance == new_distance || new_distance < 5 || new_distance > 1100){

        _rel_d *= 1.001;
        _rel_v *= 1/1.001;
        detect_health -= .05; //for fuzzy logic control

        //if new update, slew towards new updates
      } else {

        _rel_v    += (rel_v - _rel_v) / 1500 ;
        _rel_d    += (rel_d - _rel_d) / 1500 ;
        detect_health += .1; //for fuzzy logic control

      }
      if (detect_health < 0) {detect_health = 0;}
      if (detect_health > 1) {detect_health = 1;}
  */


     rel_d = new_d;
     //rel_v = new_v;
     _rel_v = rel_v; //+= (new_v - _rel_v) / 10 ;
     _rel_d = new_d;//+= (new_d - _rel_d) / 10 ;

  if (abs(trafic_distance - new_distance) < 1){
    _rel_d *= 0.9999;
    _rel_v *= 0.9999;
  }
      trafic_distance = pythagorous2(rel_d.x, rel_d.y);
      trafic_angle    = atanf(rel_d.y/rel_d.x); // the direction of traffic in the horizontal direction
      _trafic_distance = pythagorous2(_rel_d.x, _rel_d.y); //finding the magnitude of the relative distance
      _trafic_angle    = atanf(_rel_d.y/_rel_d.x); // the direction of traffic in the horizontal direction

      //only dataflash log straight from the antena
      Log_Write_Detection(new_v, new_d, new_distance, trafic_angle);

      return;}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
  if(do_track_maneuver && do_avoid_maneuver)
       {gcs_send_text(MAV_SEVERITY_CRITICAL,"In AVOIDANCE region!!");}
  else if(do_track_maneuver)
       {gcs_send_text(MAV_SEVERITY_CRITICAL,"In TRACKING region!!");}
}
#endif
