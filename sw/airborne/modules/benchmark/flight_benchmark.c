


// #include BOARD_CONFIG
// #include <stdbool.h>
// #include "firmwares/fixedwing/main_fbw.h"
//#include "sys_time.h"
//#include "sys_time_hw.h"



//bruzzlee
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "estimator.h"
#include "messages.h"
#include "downlink.h"
#include "mcu_periph/uart.h"
#include "generated/airframe.h"
#include "subsystems/nav.h"

// For Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif



float SquareSumErr_airspeed;
float SquareSumErr_altitude;
float SquareSumErr_position;

//uint8_t numOfCount;



void flight_benchmark_init( void ) {
	SquareSumErr_airspeed = 0;
	SquareSumErr_altitude = 0;
	SquareSumErr_position = 0;
}

void flight_benchmark_periodic( void ) {
	float err_temp = 0;
#ifdef BANCHMARK_AIRSPEED
	err_temp = estimator_airspeed - v_ctl_auto_airspeed_setpoint;
	SquareSumErr_airspeed += (err_temp * err_temp);
#endif
	
#ifdef BANCHMARK_ALTITUDE
	err_temp = estimator_z - v_ctl_altitude_setpoint;
	SquareSumErr_altitude += (err_temp * err_temp);
#endif
	
#ifdef BANCHMARK_POSITION
// 	err_temp = waypoints[target].x - estimator_x;
	SquareSumErr_position += (nav_shift * nav_shift);
#endif
	
	DOWNLINK_SEND_FLIGHT_BENCHMARK(DefaultChannel, &SquareSumErr_airspeed, &SquareSumErr_altitude, &SquareSumErr_position)

}

void flight_benchmark_reset( void ) {
	SquareSumErr_airspeed = 0;
	SquareSumErr_altitude = 0;
	SquareSumErr_position = 0;
}














