


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
// #include "math/pprz_algebra_int.h"
// #include "math/pprz_algebra_float.h"

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
		if (nav_in_segment){
			float deltaX = nav_segment_x_2 - nav_segment_x_1;
			float deltaY = nav_segment_y_2 - nav_segment_y_1;
			float anglePath = atan2(deltaX,deltaY);
			float deltaPlaneX = nav_segment_x_2 - estimator_x;
			float deltaPlaneY = nav_segment_y_2 - estimator_y;
			float anglePlane = atan2(deltaPlaneX,deltaPlaneY);
			float angleDiff = fabs(anglePlane - anglePath);
			Err_position = sin(angleDiff)*sqrt(deltaPlaneX*deltaPlaneX+deltaPlaneY*deltaPlaneY);
// 			
		}
		if (nav_in_circle){
			float deltaPlaneX = nav_circle_x - estimator_x;
			float deltaPlaneY = nav_circle_y - estimator_y;
			Err_position = fabs(sqrt(deltaPlaneX*deltaPlaneX+deltaPlaneY*deltaPlaneY)-nav_circle_radius);
// 			nav_circle_x
// 			nav_circle_radius
// 			nav_circle_y
		}
		
		if (nav_shift>TolerancePosition){
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














