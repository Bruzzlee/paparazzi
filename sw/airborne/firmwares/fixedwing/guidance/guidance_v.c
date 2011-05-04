/*
 * $Id$
 *
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin, Michel Gorraz
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 *  \file v_ctl_ctl
 *  \brief Vertical control for fixed wing vehicles.
 *
 */

#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "estimator.h"
#include "subsystems/nav.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"
//For Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#ifndef DOWNLINK_DEVICE
	#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

float h_ctl_pitch_min_setpoint;
float h_ctl_pitch_max_setpoint;

/* mode */
uint8_t v_ctl_mode;
uint8_t v_ctl_airspeed_mode;


/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb;
float v_ctl_altitude_pgain;
float v_ctl_altitude_error;

/* inner loop */
float v_ctl_climb_setpoint;
uint8_t v_ctl_climb_mode;
uint8_t v_ctl_auto_throttle_submode;

/* "auto throttle" inner loop parameters */
float v_ctl_auto_throttle_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_throttle;
float v_ctl_auto_throttle_climb_throttle_increment;
float v_ctl_auto_throttle_pgain;
float v_ctl_auto_throttle_igain;
float v_ctl_auto_throttle_dgain;
float v_ctl_auto_throttle_sum_err;
#define V_CTL_AUTO_THROTTLE_MAX_SUM_ERR 150
float v_ctl_auto_throttle_pitch_of_vz_pgain;
float v_ctl_auto_throttle_pitch_of_vz_dgain;

#ifndef V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN
	#define V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN 0.
#endif

/* agressive tuning */
#ifdef TUNE_AGRESSIVE_CLIMB
	float agr_climb_throttle;
	float agr_climb_pitch;
	float agr_climb_nav_ratio;
	float agr_descent_throttle;
	float agr_descent_pitch;
	float agr_descent_nav_ratio;
#endif

pprz_t v_ctl_throttle_setpoint;
pprz_t v_ctl_throttle_slewed;

inline static void v_ctl_climb_auto_throttle_loop(void);
//#ifdef V_CTL_AUTO_PITCH_PGAIN
//inline static void v_ctl_climb_auto_pitch_loop( void );
//#endif

#ifndef V_CTL_AIRSPEED_MODE
#define V_CTL_AIRSPEED_MODE AS_MODE_STANDARD
#endif

#ifdef USE_AIRSPEED
	float v_ctl_auto_airspeed_setpoint;
	float v_ctl_auto_airspeed_controlled;
	float v_ctl_auto_airspeed_deadband;

	//Vassillis
	float v_ctl_auto_airspeed_throttle_pgain_v;
	float v_ctl_auto_airspeed_throttle_igain_v;
	float v_ctl_auto_airspeed_pitch_pgain_v;
	float v_ctl_auto_airspeed_pitch_igain_v;
	//AirSpeed Pitch Climbrate
	float v_ctl_auto_airspeed_throttle_pgain_aspc;
	float v_ctl_auto_airspeed_throttle_igain_aspc;
	float v_ctl_auto_airspeed_prethrottle_aspc;
	float v_ctl_auto_airspeed_pitch_pgain_aspc;
	float v_ctl_auto_airspeed_pitch_igain_aspc;
	//AirSpeed Pitch Simple
	float v_ctl_auto_airspeed_throttle_pgain_asps;
	float v_ctl_auto_airspeed_throttle_igain_asps;
	float v_ctl_auto_airspeed_prethrottle_asps;
	float v_ctl_auto_airspeed_pitch_pgain_asps;
	float v_ctl_auto_airspeed_pitch_igain_asps;
	#define V_CTL_AUTO_ALT_MAX_SUM_ERR 200
	//AirSpeed Manual Power
	float v_ctl_auto_airspeed_throttlesetp_asmp;
	float v_ctl_auto_airspeed_pitch_pgain_asmp;
	float v_ctl_auto_airspeed_pitch_igain_asmp;
	//AirSpeed Pitch Acceleration
	float v_ctl_auto_airspeed_throttle_pgain_aspa;
	float v_ctl_auto_airspeed_throttle_igain_aspa;
	float v_ctl_auto_airspeed_prethrottle_aspa;
	float v_ctl_auto_airspeed_pitch_pgain_aspa;
	float v_ctl_auto_airspeed_pitch_igain_aspa;
	float v_ctl_airspeed_acc_filter_value;
	float v_ctl_accel_pgain;
	float v_ctl_altitude_max_accel;
	float v_ctl_auto_acceleration_sum_err;

	float v_ctl_auto_airspeed_sum_err;
	float v_ctl_auto_climb_limit;

	float v_ctl_auto_pitch_sum_err;
	float v_ctl_auto_alt_sum_err; //AirSpeed Pitch Simple - AirSpeed Pitch Acceleration
	#define V_CTL_AUTO_PITCH_MAX_SUM_ERR 100
	float v_ctl_auto_groundspeed_setpoint;
	float v_ctl_auto_groundspeed_pgain;
	float v_ctl_auto_groundspeed_igain;
	float v_ctl_auto_groundspeed_sum_err;
	#define V_CTL_AUTO_AIRSPEED_MAX_SUM_ERR 200
	#define V_CTL_AUTO_ACCELERATION_MAX_SUM_ERR 200 //AirSpeed Pitch Acceleration
	#define V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR 100
	#ifndef V_CTL_AUTO_CLIMB_LIMIT
		#define V_CTL_AUTO_CLIMB_LIMIT 0.5/4.0 // m/s/s
	#endif
	#define V_CTL_AUTO_AGR_CLIMB_GAIN 2.0 // altitude gain multiplier while in aggressive climb mode
#endif
	


void v_ctl_init(void) {
	/* mode */
	v_ctl_mode = V_CTL_MODE_MANUAL;

	/* outer loop */
	v_ctl_altitude_setpoint = 0.;
	v_ctl_altitude_pre_climb = 0.;
	v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
	v_ctl_altitude_error = 0.;

		v_ctl_airspeed_mode = V_CTL_AIRSPEED_MODE;
	
	/* inner loops */
	v_ctl_climb_setpoint = 0.;
	v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
	v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;

	/* "auto throttle" inner loop parameters */
	v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
	v_ctl_auto_throttle_cruise_throttle = v_ctl_auto_throttle_nominal_cruise_throttle;
	v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
	v_ctl_auto_throttle_pgain = V_CTL_AUTO_THROTTLE_PGAIN;
	v_ctl_auto_throttle_igain = V_CTL_AUTO_THROTTLE_IGAIN;
	v_ctl_auto_throttle_dgain = 0.;
	v_ctl_auto_throttle_sum_err = 0.;
	v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;
	v_ctl_auto_throttle_pitch_of_vz_dgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN;


	/* "auto pitch" inner loop parameters */
	 
	h_ctl_pitch_min_setpoint = H_CTL_PITCH_MIN_SETPOINT;
	h_ctl_pitch_max_setpoint = H_CTL_PITCH_MAX_SETPOINT;


	#ifdef USE_AIRSPEED
		v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
		v_ctl_auto_airspeed_controlled = V_CTL_AUTO_AIRSPEED_SETPOINT;
		v_ctl_auto_airspeed_deadband = V_CTL_AUTO_AIRSPEED_DEADBAND;
		//Vassillis
		v_ctl_auto_airspeed_throttle_pgain_v = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN_V;
		v_ctl_auto_airspeed_throttle_igain_v = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN_V;
		v_ctl_auto_airspeed_pitch_pgain_v = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_V;
		v_ctl_auto_airspeed_pitch_igain_v = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_V;
		//AirSpeed Pitch Climbrate
		v_ctl_auto_airspeed_throttle_pgain_aspc = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN_ASPC;
		v_ctl_auto_airspeed_throttle_igain_aspc = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN_ASPC;
		v_ctl_auto_airspeed_prethrottle_aspc = V_CTL_AUTO_AIRSPEED_PRETHROTTLE_ASPC;
		v_ctl_auto_airspeed_pitch_pgain_aspc = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_ASPC;
		v_ctl_auto_airspeed_pitch_igain_aspc = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_ASPC;
		//AirSpeed Pitch Simple
		v_ctl_auto_airspeed_throttle_pgain_asps = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN_ASPS;
		v_ctl_auto_airspeed_throttle_igain_asps = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN_ASPS;
		v_ctl_auto_airspeed_prethrottle_asps = V_CTL_AUTO_AIRSPEED_PRETHROTTLE_ASPS;
		v_ctl_auto_airspeed_pitch_pgain_asps = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_ASPS;
		v_ctl_auto_airspeed_pitch_igain_asps = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_ASPS;
		//AirSpeed Manual Power
		v_ctl_auto_airspeed_throttlesetp_asmp = V_CTL_AUTO_AIRSPEED_THROTTLESETP_ASMP;
		v_ctl_auto_airspeed_pitch_pgain_asmp = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_ASMP;
		v_ctl_auto_airspeed_pitch_igain_asmp = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_ASMP;
		//AirSpeed Pitch Acceleration
		v_ctl_auto_airspeed_throttle_pgain_aspa = V_CTL_AUTO_AIRSPEED_THROTTLE_PGAIN_ASPA;
		v_ctl_auto_airspeed_throttle_igain_aspa = V_CTL_AUTO_AIRSPEED_THROTTLE_IGAIN_ASPA;
		v_ctl_auto_airspeed_prethrottle_aspa = V_CTL_AUTO_AIRSPEED_PRETHROTTLE_ASPA;
		v_ctl_auto_airspeed_pitch_pgain_aspa = V_CTL_AUTO_AIRSPEED_PITCH_PGAIN_ASPA;
		v_ctl_auto_airspeed_pitch_igain_aspa = V_CTL_AUTO_AIRSPEED_PITCH_IGAIN_ASPA;
		v_ctl_airspeed_acc_filter_value = V_CTL_AIRSPEED_ACC_FILTER_VALUE;
		v_ctl_accel_pgain = V_CTL_ACCEL_PGAIN;
		v_ctl_altitude_max_accel = V_CTL_ALTITUDE_MAX_ACCEL;

		v_ctl_auto_airspeed_sum_err = 0.;
		v_ctl_auto_climb_limit = V_CTL_AUTO_CLIMB_LIMIT;

		v_ctl_auto_groundspeed_setpoint = V_CTL_AUTO_GROUNDSPEED_SETPOINT;
		v_ctl_auto_groundspeed_pgain = V_CTL_AUTO_GROUNDSPEED_PGAIN;
		v_ctl_auto_groundspeed_igain = V_CTL_AUTO_GROUNDSPEED_IGAIN;
		v_ctl_auto_groundspeed_sum_err = 0.;


		v_ctl_auto_pitch_sum_err = 0.;
		v_ctl_auto_alt_sum_err = 0.;
	#endif

	v_ctl_throttle_setpoint = 0;

	/*agressive tuning*/
	#ifdef TUNE_AGRESSIVE_CLIMB
		agr_climb_throttle = AGR_CLIMB_THROTTLE;
		#undef   AGR_CLIMB_THROTTLE
		#define AGR_CLIMB_THROTTLE agr_climb_throttle
		agr_climb_pitch = AGR_CLIMB_PITCH;
		#undef   AGR_CLIMB_PITCH
		#define   AGR_CLIMB_PITCH agr_climb_pitch
		agr_climb_nav_ratio = AGR_CLIMB_NAV_RATIO;
		#undef   AGR_CLIMB_NAV_RATIO
		#define   AGR_CLIMB_NAV_RATIO agr_climb_nav_ratio
		agr_descent_throttle = AGR_DESCENT_THROTTLE;
		#undef   AGR_DESCENT_THROTTLE
		#define   AGR_DESCENT_THROTTLE agr_descent_throttle
		agr_descent_pitch = AGR_DESCENT_PITCH;
		#undef   AGR_DESCENT_PITCH
		#define   AGR_DESCENT_PITCH agr_descent_pitch
		agr_descent_nav_ratio = AGR_DESCENT_NAV_RATIO;
		#undef   AGR_DESCENT_NAV_RATIO
		#define   AGR_DESCENT_NAV_RATIO agr_descent_nav_ratio
	#endif
}

/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */
void v_ctl_altitude_loop(void) {
	float altitude_pgain_boost = 1.0;

	#if defined(USE_AIRSPEED) && defined(AGR_CLIMB)
		// Aggressive climb mode (boost gain of altitude loop)
		if ( v_ctl_climb_mode == V_CTL_CLIMB_MODE_AUTO_THROTTLE) {
			float dist = fabs(v_ctl_altitude_error);
			altitude_pgain_boost = 1.0 + (V_CTL_AUTO_AGR_CLIMB_GAIN-1.0)*(dist-AGR_BLEND_END)/(AGR_BLEND_START-AGR_BLEND_END);
			Bound(altitude_pgain_boost, 1.0, V_CTL_AUTO_AGR_CLIMB_GAIN);
		}
	#endif

	v_ctl_altitude_error = estimator_z - v_ctl_altitude_setpoint;
	
	if (v_ctl_airspeed_mode==AS_MODE_STANDARD || v_ctl_airspeed_mode==AS_MODE_VASSILLIS || v_ctl_airspeed_mode==AS_MODE_ASP_CLIMBRATE) {
		v_ctl_climb_setpoint = altitude_pgain_boost * v_ctl_altitude_pgain * v_ctl_altitude_error + v_ctl_altitude_pre_climb;
		BoundAbs(v_ctl_climb_setpoint, V_CTL_ALTITUDE_MAX_CLIMB);
	}
	

	#ifdef AGR_CLIMB
		if ( v_ctl_climb_mode == V_CTL_CLIMB_MODE_AUTO_THROTTLE) {
			float dist = fabs(v_ctl_altitude_error);
			if (dist < AGR_BLEND_END) {
				v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_STANDARD;
			}
			else if (dist > AGR_BLEND_START) {
				v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_AGRESSIVE;
			}
			else {
				v_ctl_auto_throttle_submode = V_CTL_AUTO_THROTTLE_BLENDED;
			}
		}
	#endif
}

void v_ctl_climb_loop(void) {
	
	v_ctl_climb_auto_throttle_loop();
	
// 	switch (v_ctl_climb_mode) {
// 		case V_CTL_CLIMB_MODE_AUTO_THROTTLE:
// 		default:
// 		v_ctl_climb_auto_throttle_loop();
// 		break;
//#ifdef V_CTL_AUTO_PITCH_PGAIN
//		case V_CTL_CLIMB_MODE_AUTO_PITCH:
//		v_ctl_climb_auto_pitch_loop();
//		break;
//#endif
// 	}
}

/**
 * auto throttle inner loop
 * \brief
 */

inline static void v_ctl_climb_auto_throttle_loop(void) {

#ifndef USE_AIRSPEED
	v_ctl_airspeed_mode=AS_MODE_STANDARD;
#endif
	
	if (v_ctl_airspeed_mode==AS_MODE_STANDARD) {
		static float last_err;
		float f_throttle = 0;

		float err = estimator_z_dot - v_ctl_climb_setpoint;
		float d_err = err - last_err;
		last_err = err;
		float controlled_throttle = v_ctl_auto_throttle_cruise_throttle + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint + v_ctl_auto_throttle_pgain * (err + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err + v_ctl_auto_throttle_dgain * d_err);

		/* pitch pre-command */
		float v_ctl_pitch_of_vz = (v_ctl_climb_setpoint + d_err * v_ctl_auto_throttle_pitch_of_vz_dgain) * v_ctl_auto_throttle_pitch_of_vz_pgain;

#if defined AGR_CLIMB
		switch (v_ctl_auto_throttle_submode) {
			case V_CTL_AUTO_THROTTLE_AGRESSIVE:
				if (v_ctl_climb_setpoint > 0) { /* Climbing */
					f_throttle = AGR_CLIMB_THROTTLE;
					nav_pitch = AGR_CLIMB_PITCH;
				}
				else { /* Going down */
					f_throttle = AGR_DESCENT_THROTTLE;
					nav_pitch = AGR_DESCENT_PITCH;
				}
				break;

			case V_CTL_AUTO_THROTTLE_BLENDED: {
				float ratio = (fabs(v_ctl_altitude_error) - AGR_BLEND_END) / (AGR_BLEND_START - AGR_BLEND_END);
				f_throttle = (1-ratio) * controlled_throttle;
				nav_pitch = (1-ratio) * v_ctl_pitch_of_vz;
				v_ctl_auto_throttle_sum_err += (1-ratio) * err;
				BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
				if (v_ctl_altitude_error < 0) {
					f_throttle += ratio * AGR_CLIMB_THROTTLE;
					nav_pitch += ratio * AGR_CLIMB_PITCH;
				} else {
					f_throttle += ratio * AGR_DESCENT_THROTTLE;
					nav_pitch += ratio * AGR_DESCENT_PITCH;
				}
				break;
			}

			case V_CTL_AUTO_THROTTLE_STANDARD:
			default:
#endif
				f_throttle = controlled_throttle;
				v_ctl_auto_throttle_sum_err += err;
				BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
				nav_pitch += v_ctl_pitch_of_vz;
				//Bound(nav_pitch, h_ctl_pitch_min_setpoint, h_ctl_pitch_max_setpoint); //NEW
#if defined AGR_CLIMB
				break;
		} /* switch submode */
#endif
		v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
	}
#if defined USE_AIRSPEED
	else {

		//***********AIRSPEED CONTROL***********************************

		float f_throttle = 0;
		float controlled_throttle = 0;
		float v_ctl_pitch_of_vz = 0;

		// Limit rate of change of climb setpoint (to ensure that airspeed loop can catch-up)
		static float v_ctl_climb_setpoint_last = 0;
		
		//AirSpeed Pitch Acceleration - ASPA
		static float last_airspeed = 0;
		static float err_acceleration = 0;
		static float acceleration = 0;
		float accel_new = 0;
		float v_ctl_accel_setpoint = 0;
		
		float diff_climb = v_ctl_climb_setpoint - v_ctl_climb_setpoint_last;
		Bound(diff_climb, -v_ctl_auto_climb_limit, v_ctl_auto_climb_limit);
		v_ctl_climb_setpoint = v_ctl_climb_setpoint_last + diff_climb;
		v_ctl_climb_setpoint_last = v_ctl_climb_setpoint;

		// Pitch control (input: rate of climb error, output: pitch setpoint)
		float err = estimator_z_dot - v_ctl_climb_setpoint;
		v_ctl_auto_pitch_sum_err += err;
		BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR);
		switch (v_ctl_airspeed_mode) {
			case AS_MODE_VASSILLIS: //Vasilis - v
				v_ctl_pitch_of_vz = v_ctl_auto_airspeed_pitch_pgain_v * (err + v_ctl_auto_airspeed_pitch_igain_v * v_ctl_auto_pitch_sum_err);
				break;
			case AS_MODE_ASP_CLIMBRATE: //AirSpeed Pitch Climbrate - ASPC
				controlled_throttle = v_ctl_auto_airspeed_prethrottle_aspc + v_ctl_auto_airspeed_throttle_pgain_aspc * (err + v_ctl_auto_airspeed_throttle_igain_aspc * v_ctl_auto_pitch_sum_err);
				//controlled_throttle = 0.6;
				break;
			case AS_MODE_ASP_SIMPLE: //AirSpeed Pitch Simple - ASPS
				v_ctl_auto_alt_sum_err += v_ctl_altitude_error;
				BoundAbs(v_ctl_auto_alt_sum_err, V_CTL_AUTO_ALT_MAX_SUM_ERR);
				controlled_throttle = v_ctl_auto_airspeed_prethrottle_asps + v_ctl_auto_airspeed_throttle_pgain_asps * (v_ctl_altitude_error + v_ctl_auto_airspeed_throttle_igain_asps * v_ctl_auto_alt_sum_err);
				break;
			case AS_MODE_ASP_MANUAL: //AirSpeed Manual Power - ASMP
				controlled_throttle = v_ctl_auto_airspeed_throttlesetp_asmp;
				break;
			case AS_MODE_ASP_ACCEL: //AirSpeed Pitch Acceleration - ASPA
				v_ctl_auto_alt_sum_err += v_ctl_altitude_error;
				BoundAbs(v_ctl_auto_alt_sum_err, V_CTL_AUTO_ALT_MAX_SUM_ERR);
				controlled_throttle = v_ctl_auto_airspeed_prethrottle_aspa + v_ctl_auto_airspeed_throttle_pgain_aspa * (v_ctl_altitude_error + v_ctl_auto_airspeed_throttle_igain_aspa * v_ctl_auto_alt_sum_err);
				break;
		}

		// Ground speed control loop (input: groundspeed error, output: airspeed controlled)
		float err_groundspeed = (v_ctl_auto_groundspeed_setpoint - estimator_hspeed_mod);
		v_ctl_auto_groundspeed_sum_err += err_groundspeed;
		BoundAbs(v_ctl_auto_groundspeed_sum_err, V_CTL_AUTO_GROUNDSPEED_MAX_SUM_ERR);
		v_ctl_auto_airspeed_controlled = (err_groundspeed + v_ctl_auto_groundspeed_sum_err * v_ctl_auto_groundspeed_igain) * v_ctl_auto_groundspeed_pgain;

		// Do not allow controlled airspeed below the setpoint
		if (v_ctl_auto_airspeed_controlled < v_ctl_auto_airspeed_setpoint) {
			v_ctl_auto_airspeed_controlled = v_ctl_auto_airspeed_setpoint;
			v_ctl_auto_groundspeed_sum_err = v_ctl_auto_airspeed_controlled	/ (v_ctl_auto_groundspeed_pgain * v_ctl_auto_groundspeed_igain); // reset integrator of ground speed loop
		}

		// Airspeed control loop (input: airspeed controlled, output: throttle controlled)
		float err_airspeed = (v_ctl_auto_airspeed_controlled - estimator_airspeed);
		
		
		// Airspeed downltime (Totzeit)
		if (err_airspeed > v_ctl_auto_airspeed_deadband) {
			err_airspeed = err_airspeed - v_ctl_auto_airspeed_deadband;
		}
		else if (err_airspeed < -v_ctl_auto_airspeed_deadband) {
			err_airspeed = err_airspeed + v_ctl_auto_airspeed_deadband;
		}
		
		
		
		v_ctl_auto_airspeed_sum_err += err_airspeed;
		BoundAbs(v_ctl_auto_airspeed_sum_err, V_CTL_AUTO_AIRSPEED_MAX_SUM_ERR);
		


		switch (v_ctl_airspeed_mode){
			case AS_MODE_VASSILLIS: //Vasilis
				controlled_throttle = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_throttle_igain_v) * v_ctl_auto_airspeed_throttle_pgain_v;
				break;
			case AS_MODE_ASP_CLIMBRATE: //AirSpeed Pitch Climbrate - ASPC
				v_ctl_pitch_of_vz = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_pitch_igain_aspc) * v_ctl_auto_airspeed_pitch_pgain_aspc;
				break;
			case AS_MODE_ASP_SIMPLE: //AirSpeed Pitch Simple - ASPS
				v_ctl_pitch_of_vz = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_pitch_igain_asps) * v_ctl_auto_airspeed_pitch_pgain_asps;
				break;
			case AS_MODE_ASP_MANUAL: //AirSpeed Manual Power - ASMP
				v_ctl_pitch_of_vz = (err_airspeed + v_ctl_auto_airspeed_sum_err * v_ctl_auto_airspeed_pitch_igain_asmp) * v_ctl_auto_airspeed_pitch_pgain_asmp;
				break;
			case AS_MODE_ASP_ACCEL: //AirSpeed Pitch Acceleration - ASPA				
			
				
				//Lowpass acceleration filtering
				accel_new = v_ctl_airspeed_acc_filter_value * acceleration + (1-v_ctl_airspeed_acc_filter_value) * (last_airspeed - estimator_airspeed);
				acceleration = accel_new;
				
				last_airspeed = estimator_airspeed;
				
				v_ctl_accel_setpoint = v_ctl_accel_pgain * err_airspeed;
				BoundAbs(v_ctl_accel_setpoint, v_ctl_altitude_max_accel);
				err_acceleration = v_ctl_accel_setpoint - acceleration;
				v_ctl_auto_acceleration_sum_err += err_acceleration;
				BoundAbs(v_ctl_auto_airspeed_sum_err, V_CTL_AUTO_ACCELERATION_MAX_SUM_ERR);
				
				v_ctl_pitch_of_vz = (err_acceleration + v_ctl_auto_acceleration_sum_err * v_ctl_auto_airspeed_pitch_igain_aspa) * v_ctl_auto_airspeed_pitch_pgain_aspa;
				break;
		}
		
		
		// Done, set outputs
		Bound(controlled_throttle, 0, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
		f_throttle = controlled_throttle;
		nav_pitch = v_ctl_pitch_of_vz;
		v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
		Bound(nav_pitch, h_ctl_pitch_min_setpoint, h_ctl_pitch_max_setpoint);
		RunOnceEvery(10, DOWNLINK_SEND_AS_GUIDANCE(DefaultChannel, &v_ctl_airspeed_mode, &v_ctl_auto_airspeed_sum_err, &err_airspeed, &err_groundspeed, &nav_pitch, &v_ctl_throttle_setpoint, &f_throttle));
	}
#endif
	
}

/**
 * auto pitch inner loop
 * \brief computes a nav_pitch from a climb_setpoint given a fixed throttle
 */
/*#ifdef V_CTL_AUTO_PITCH_PGAIN
 inline static void v_ctl_climb_auto_pitch_loop(void) {
 float err = estimator_z_dot - v_ctl_climb_setpoint;
 v_ctl_throttle_setpoint = nav_throttle_setpoint;
 v_ctl_auto_pitch_sum_err += err;
 BoundAbs(v_ctl_auto_pitch_sum_err, V_CTL_AUTO_PITCH_MAX_SUM_ERR);
 nav_pitch = v_ctl_auto_pitch_pgain *
 (err + v_ctl_auto_pitch_igain * v_ctl_auto_pitch_sum_err);
 Bound(nav_pitch, V_CTL_AUTO_PITCH_MIN_PITCH, V_CTL_AUTO_PITCH_MAX_PITCH);
 }
 #endif
 */
#ifdef V_CTL_THROTTLE_SLEW_LIMITER
#define V_CTL_THROTTLE_SLEW (1./CONTROL_RATE/(V_CTL_THROTTLE_SLEW_LIMITER))
#endif

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW 1.
#endif
/** \brief Computes slewed throttle from throttle setpoint
 called at 20Hz
 */
void v_ctl_throttle_slew(void) {
	pprz_t diff_throttle = v_ctl_throttle_setpoint - v_ctl_throttle_slewed;
	BoundAbs(diff_throttle, TRIM_PPRZ(V_CTL_THROTTLE_SLEW * MAX_PPRZ));
	v_ctl_throttle_slewed += diff_throttle;
}
