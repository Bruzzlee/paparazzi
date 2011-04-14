


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
float ToleranceAispeed;
float ToleranceAltitude;
float TolerancePosition;
bool_t benchm_reset;
bool_t benchm_go;


//uint8_t numOfCount;



void flight_benchmark_init( void ) {
	SquareSumErr_airspeed = 0;
	SquareSumErr_altitude = 0;
	SquareSumErr_position = 0;
	ToleranceAispeed = BENCHMARK_TOLERANCE_AIRSPEED;
	ToleranceAltitude = BENCHMARK_TOLERANCE_ALTITUDE;
	TolerancePosition = BENCHMARK_TOLERANCE_POSITION;	
	benchm_reset = 0;
	benchm_go = 0;
}

void flight_benchmark_periodic( void ) {
	float Err_airspeed = 0;
	float Err_altitude = 0;
	float Err_position = 0;
	
	if (benchm_reset){
		flight_benchmark_reset();
		benchm_reset = 0;
	}
	
	if (benchm_go){
	#ifdef BENCHMARK_AIRSPEED
		Err_airspeed = fabs(estimator_airspeed - v_ctl_auto_airspeed_setpoint);
		if (Err_airspeed>ToleranceAispeed){
			Err_airspeed = Err_airspeed-ToleranceAispeed;
			SquareSumErr_airspeed += (Err_airspeed * Err_airspeed);
		}
		
	#endif
		
	#ifdef BENCHMARK_ALTITUDE
		Err_altitude = fabs(estimator_z - v_ctl_altitude_setpoint);
		if (Err_altitude>ToleranceAltitude){
			Err_altitude = Err_altitude-ToleranceAltitude;
			SquareSumErr_altitude += (Err_altitude * Err_altitude);
		}
	#endif
		
	#ifdef BENCHMARK_POSITION
	// 	err_temp = waypoints[target].x - estimator_x;
		if (nav_shift>TolerancePosition){
			Err_position = nav_shift-TolerancePosition;
			SquareSumErr_position += (Err_position * Err_position);
		}
	#endif
	}
	
	DOWNLINK_SEND_FLIGHT_BENCHMARK(DefaultChannel, &SquareSumErr_airspeed, &SquareSumErr_altitude, &SquareSumErr_position, &Err_airspeed, &Err_altitude, &Err_position)

}

void flight_benchmark_reset( void ) {
	SquareSumErr_airspeed = 0;
	SquareSumErr_altitude = 0;
	SquareSumErr_position = 0;
}














