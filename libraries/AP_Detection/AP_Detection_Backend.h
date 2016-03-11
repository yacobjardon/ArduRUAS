/*
  code created for RUAS 2015/2016 project for guidance and control
  backend driver class for detection
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_Detection_Backend {
public:
    // probe and initialise the PSoC
     void init(void);

    // return the 3D location in meters
     void get_location_detection(float &x_loc, float &y_loc, float &z_loc);

    // return the relative velocity in meters per second
     void get_relative_velocity(float &x_vel, float &y_vel, float &z_vel);
};
