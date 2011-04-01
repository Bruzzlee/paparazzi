/*
* Driver for the EagleTree Systems Airspeed Sensor
* Has only been tested with V3 of the sensor hardware
*
* Notes:
* Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
* Sensor should be in the proprietary mode (default) and not in 3rd party mode.
*
* Sensor module wire assignments:
* Red wire: 5V
* White wire: Ground
* Yellow wire: SDA
* Brown wire: SCL
*
* Copyright (C) 2009 Vassilis Varveropoulos
* Modified by Mark Griffin on 8 September 2010 to work with new i2c transaction routines.
* Converted by Gautier Hattenberger to modules (10/2010)
*
* This file is part of paparazzi.
*
* paparazzi is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2, or (at your option)
* any later version.
*
* paparazzi is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*
*/
#include "sensors/airspeed_amsys.h"
#include "estimator.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include <math.h>
//#include <stdlib.h>

#ifndef USE_AIRSPEED
// Just a Warning --> We do't use it.
//#ifndef SENSOR_SYNC_SEND
//#warning either set USE_AIRSPEED or SENSOR_SYNC_SEND to use amsys_airspeed
//#endif
#endif

#define AIRSPEED_AMSYS_ADDR 0xF4 // original F0
#ifndef AIRSPEED_AMSYS_SCALE
#define AIRSPEED_AMSYS_SCALE 1
#endif
#ifndef AIRSPEED_AMSYS_OFFSET
#define AIRSPEED_AMSYS_OFFSET 0
#endif
#define AIRSPEED_AMSYS_OFFSET_MAX 29491
#define AIRSPEED_AMSYS_OFFSET_MIN 3277
#define AIRSPEED_AMSYS_OFFSET_NBSAMPLES_INIT 40
#define AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG 60
#define AIRSPEED_AMSYS_NBSAMPLES_AVRG 10
#ifndef AIRSPEED_AMSYS_MAXPRESURE
#define AIRSPEED_AMSYS_MAXPRESURE 2068//2073 //Pascal
#endif
#ifndef AIRSPEED_AMSYS_I2C_DEV
#define AIRSPEED_AMSYS_I2C_DEV i2c0
#endif
#define TEMPERATURE_AMSYS_OFFSET_MAX 29491
#define TEMPERATURE_AMSYS_OFFSET_MIN 3277
#define TEMPERATURE_AMSYS_MAX 110
#define TEMPERATURE_AMSYS_MIN -25

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

// Global variables
uint16_t airspeed_amsys_raw;
uint16_t tempAS_amsys_raw;
bool_t airspeed_amsys_valid;
float airspeed_tmp;
float pressure_amsys; //Pascal
float airspeed_amsys; //mps
float airspeed_scale;
float airspeed_filter;
int airspeed_amsys_buffer_idx;
float airspeed_amsys_buffer[AIRSPEED_AMSYS_NBSAMPLES_AVRG];
struct i2c_transaction airspeed_amsys_i2c_trans;

// Local variables
volatile bool_t airspeed_amsys_i2c_done;
bool_t airspeed_amsys_offset_init;
float airspeed_amsys_pressure_offset_tmp;
float airspeed_amsys_offset_pressure;
float airspeed_amsys_offset;
uint16_t airspeed_amsys_cnt;
float airspeed_temperature = 0.0;
float airspeed_old = 0.0;


#define AS_ANZ 30
float airspeed_values[AS_ANZ];
//float airspeed_sorted_values[AS_ANZ];
int8_t airspeed_head;
//float baro_average;
float airspeed_sum;


void airspeed_amsys_init( void ) {
	int n;
	airspeed_amsys_raw = 0;
	airspeed_amsys = 0.0;
	pressure_amsys = 0.0;
	airspeed_amsys_offset = AIRSPEED_AMSYS_OFFSET;
	airspeed_amsys_offset_pressure = 0.0;
	airspeed_amsys_i2c_done = TRUE;
	airspeed_amsys_valid = TRUE;
	airspeed_amsys_offset_init = FALSE;
	airspeed_amsys_cnt = AIRSPEED_AMSYS_OFFSET_NBSAMPLES_INIT + AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG;
	airspeed_scale = AIRSPEED_SCALE;
	airspeed_head=0;
	airspeed_filter = AIRSPEED_FILTER;

	airspeed_amsys_buffer_idx = 0;
	for (n=0; n < AIRSPEED_AMSYS_NBSAMPLES_AVRG; ++n)
		airspeed_amsys_buffer[n] = 0.0;

	airspeed_amsys_i2c_trans.status = I2CTransDone;
}

//float float_cmp(const void *a, const void *b) 
//{ 
//    const float *ia = (const float *)a; // casting pointer types 
//    const float *ib = (const float *)b;
//    return *ia  - *ib; 
	/* integer comparison: returns negative if b > a 
	and positive if a > b */ 
//} 

void airspeed_amsys_read_periodic( void ) {
	#ifndef SITL
	if (airspeed_amsys_i2c_trans.status == I2CTransDone)
		I2CReceive(AIRSPEED_AMSYS_I2C_DEV, airspeed_amsys_i2c_trans, AIRSPEED_AMSYS_ADDR, 4);
	#else // SITL
		extern float sim_air_speed;
		EstimatorSetAirspeed(sim_air_speed);
	#endif //SITL
}

void airspeed_amsys_read_event( void ) {
	// Get raw airspeed from buffer
	airspeed_amsys_raw = 0;
	airspeed_amsys_raw = (airspeed_amsys_i2c_trans.buf[0]<<8) | airspeed_amsys_i2c_trans.buf[1];
	tempAS_amsys_raw = (airspeed_amsys_i2c_trans.buf[2]<<8) | airspeed_amsys_i2c_trans.buf[3];
	// Check if this is valid airspeed
	if (airspeed_amsys_raw == 0)
		airspeed_amsys_valid = FALSE;
	else
		airspeed_amsys_valid = TRUE;

	// Continue only if a new airspeed value was received
	if (airspeed_amsys_valid) {
	 
		// raw not under offest min
		if (airspeed_amsys_raw<AIRSPEED_AMSYS_OFFSET_MIN)
			airspeed_amsys_raw = AIRSPEED_AMSYS_OFFSET_MIN;
		// raw not over offest mac
		if (airspeed_amsys_raw>AIRSPEED_AMSYS_OFFSET_MAX)
			airspeed_amsys_raw = AIRSPEED_AMSYS_OFFSET_MAX;

		// calculate raw to pressure
		pressure_amsys = (float)(airspeed_amsys_raw-AIRSPEED_AMSYS_OFFSET_MIN)*AIRSPEED_AMSYS_MAXPRESURE/(float)(AIRSPEED_AMSYS_OFFSET_MAX-AIRSPEED_AMSYS_OFFSET_MIN);

		// Calculate offset average if not done already
//		if (!airspeed_amsys_offset_init) {
//			--airspeed_amsys_cnt;
//			// Check if averaging completed
//			if (airspeed_amsys_cnt == 0) {
//				// Calculate average
//				airspeed_amsys_offset_pressure = (float)(airspeed_amsys_pressure_offset_tmp / AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG);
//				airspeed_amsys_offset_init = TRUE;
//			}
//			// Check if averaging needs to continue
//			else if (airspeed_amsys_cnt <= AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG)
//				airspeed_amsys_pressure_offset_tmp += pressure_amsys;
//		}
	
		// Set Minimal
//		if (pressure_amsys<airspeed_amsys_offset_pressure)
//			pressure_amsys=airspeed_amsys_offset_pressure;

		//airspeed_tmp = sqrtf(2*(pressure_amsys-airspeed_amsys_offset_pressure)*airspeed_scale/1.2041);
		airspeed_tmp = sqrtf(2*(pressure_amsys)*airspeed_scale/1.2041); //without offset
    
	// Airspeed should always be positive
		//if (airspeed_tmp < 0.0)
		//	airspeed_tmp = 0.0;

    // Avarage filter of airspeed
	airspeed_head++;
	if (airspeed_head >= AS_ANZ) {
		airspeed_head = 0;
	}
	airspeed_sum = airspeed_sum + airspeed_tmp - airspeed_values[airspeed_head]; 
	airspeed_values[airspeed_head] = airspeed_tmp;
	airspeed_amsys = airspeed_scale*(airspeed_sum / AS_ANZ);
	//airspeed_amsys = airspeed_tmp;
		
		//airspeed_sorted_values=airspeed_values;
		//qsort(airspeed_sorted_values[0], AS_ANZ, sizeof(float), float_cmp);


		//Lowpass filter
		//airspeed_amsys = airspeed_filter * airspeed_old + (1 - airspeed_filter) * airspeed_tmp;
		//airspeed_old = airspeed_amsys;
		
	
	airspeed_temperature = (float)((float)(tempAS_amsys_raw-TEMPERATURE_AMSYS_OFFSET_MIN)/((float)(TEMPERATURE_AMSYS_OFFSET_MAX-TEMPERATURE_AMSYS_OFFSET_MIN)/TEMPERATURE_AMSYS_MAX)+TEMPERATURE_AMSYS_MIN);// Tmin=-25, Tmax=85	
		
		
#ifdef USE_AIRSPEED
    EstimatorSetAirspeed(airspeed_amsys);
#endif
#ifdef SENSOR_SYNC_SEND
    DOWNLINK_SEND_AMSYS_AIRSPEED(DefaultChannel, &airspeed_amsys_raw, &airspeed_amsys_offset_pressure, &pressure_amsys, &airspeed_tmp, &airspeed_amsys, &airspeed_temperature);
#else
    RunOnceEvery(10, DOWNLINK_SEND_AMSYS_AIRSPEED(DefaultChannel, &airspeed_amsys_raw, &airspeed_amsys_offset_pressure, &pressure_amsys, &airspeed_tmp, &airspeed_amsys, &airspeed_temperature));
#endif

  } 

  // Transaction has been read
  airspeed_amsys_i2c_trans.status = I2CTransDone;
}

