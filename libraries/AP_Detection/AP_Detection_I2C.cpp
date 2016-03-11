/*
  code created for RUAS 2015/2016 project for guidance and control
  backend driver for detection from 5LP PSoC
 */

#include "AP_Detection_I2C.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define I2C_ADDRESS_PSOC 0x2B // address provided by Brian Lemky

// probe and initialise the PSoC
void AP_Detection_I2C::init()
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(200)) {
        return;
    }

    _measure();
    hal.scheduler->delay(10);
    _collect();
    i2c_sem->give();

	if (_last_sample_time_ms != 0) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Detection_I2C::_timer, void));
        return;
    }
    return;
}

// start a measurement by writing 0x00 to the PSoC
void AP_Detection_I2C::_measure()
{
    _measurement_started_ms = 0;
    if (hal.i2c->writeRegisters(I2C_ADDRESS_PSOC, 0, 0, NULL) == 0) {
        _measurement_started_ms = AP_HAL::millis(); //millis() is the number of milliseconds since the program started
    }
}

// read the values from the PSoC
void AP_Detection_I2C::_collect()
{
    uint8_t data[12]; // reading 12 bytes, 6 bytes for location and 6 for relative velocity

    _measurement_started_ms = 0;

    if (hal.i2c->read(I2C_ADDRESS_PSOC, 12, data) != 0) {
        return;
    }

	float x_loc_m, y_loc_m, z_loc_m, x_loc_cm, y_loc_cm, z_loc_cm, x_vel_m, y_vel_m, z_vel_m, x_vel_cm, y_vel_cm, z_vel_cm;

/*
  int num = 0;
  for(num=0;num<12;num++)
  {
    if(data[num] > 127)
    {
      data[num] = data[num]-256;
    }
  }
*/

	x_loc_m = data[0];
	y_loc_m = data[1];
	z_loc_m = data[2];
	x_loc_cm = data[3];
	y_loc_cm = data[4];
	z_loc_cm = data[5];
	x_vel_m = data[6];
	y_vel_m = data[7];
	z_vel_m = data[8];
	x_vel_cm = data[9];
	y_vel_cm = data[10];
	z_vel_cm = data[11];

if(x_loc_m > 127){x_loc_m-=256;}
if(y_loc_m > 127){y_loc_m-=256;}
if(z_loc_m > 127){z_loc_m-=256;}

if(x_loc_cm > 127){x_loc_cm-=256;}
if(y_loc_cm > 127){y_loc_cm-=256;}
if(z_loc_cm > 127){z_loc_cm-=256;}

	_location_x = (double)x_loc_m*100 + x_loc_cm;
	_location_y = (double)y_loc_m*100 + y_loc_cm;
	_location_z = (double)z_loc_m*100 + z_loc_cm;

	_relative_velocity_x = (double)x_vel_m + (double)x_vel_cm/100;
	_relative_velocity_y = (double)y_vel_m + (double)y_vel_cm/100;
	_relative_velocity_z = (double)z_vel_m + (double)z_vel_cm/100;

    _last_sample_time_ms = AP_HAL::millis();
}


void AP_Detection_I2C::_timer()
{
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    if (!i2c_sem->take_nonblocking()) {
        return;
    }

    if (_measurement_started_ms == 0) {
        _measure();
        i2c_sem->give();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
    i2c_sem->give();
}

// return the 3D location in meters
void AP_Detection_I2C::get_location_detection(float &x_loc, float &y_loc, float &z_loc)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return;
    }
    x_loc = _location_x;
	  y_loc = _location_y;
  	z_loc = _location_z;

    return;
}

// return the relative velocity in meters per second
void AP_Detection_I2C::get_relative_velocity(float &x_vel, float &y_vel, float &z_vel)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return;
    }

	x_vel = _relative_velocity_x;
	y_vel = _relative_velocity_y;
	z_vel = _relative_velocity_z;

    return;
}
