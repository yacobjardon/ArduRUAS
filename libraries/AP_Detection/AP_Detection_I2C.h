/*
  code created for RUAS 2015/2016 project for guidance and control
  backend driver for detection for I2C
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_Detection_Backend.h"

class AP_Detection_I2C : public AP_Detection_Backend
{
public:
    // probe and initialise the PSoC
    void init();

	// return the 3D location in meters
    void get_location_detection(float &x_loc, float &y_loc, float &z_loc);

	// return the relative velocity in meters per second
    void get_relative_velocity(float &x_vel, float &y_vel, float &z_vel);

private:
    void _measure();
    void _collect();
    void _timer();

	  float _relative_velocity_x;
	  float _relative_velocity_y;
	  float _relative_velocity_z;

    float _location_x;
	  float _location_y;
	  float _location_z;

	  uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;
};
