/*
 * $Id$
 *
 * Copyright (C) 2011  Bruzzlee
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
/** \file paramter_changer.c
 *  \brief Change parameter min/max of pitch and roll and airspeed-mode
 *
 */

#include "subsystems/navigation/parameter_changer.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

bool_t set_as_mode(uint8_t as_mode_set)
{
	//AS_MODE_STANDARD		0
	//AS_MODE_VASSILLIS		1
	//AS_MODE_ASP_CLIMBRATE		2
	//AS_MODE_ASP_SIMPLE		3
	//AS_MODE_ASP_MANUAL		4
	//AS_MODE_ASP_ACCEL		5
	//AS_MODE_FIX_PITCH		6
	v_ctl_airspeed_mode = as_mode_set;
	
	return FALSE;
}

void set_max_roll(float max_roll)
{
	if(max_roll<10.0){
		h_ctl_roll_max_setpoint = max_roll;
	}
	else
		h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;
	send_params();
}

void set_max_pitch(float max_pitch)
{
	if(max_pitch<10.0){
		h_ctl_pitch_max_setpoint = max_pitch;
	}
	else
		h_ctl_pitch_max_setpoint = H_CTL_PITCH_MAX_SETPOINT;
	send_params();
}

void set_min_pitch(float min_pitch)
{
	if(min_pitch<10.0){
		h_ctl_pitch_min_setpoint = min_pitch;
	}
	else
		h_ctl_pitch_min_setpoint = H_CTL_PITCH_MIN_SETPOINT;
	send_params();
}

void set_approach_params()
{
	//v_ctl_airspeed_mode = AS_MODE_ASP_SIMPLE;
		set_max_roll(99.0);
		set_max_pitch(99.0); 
		set_min_pitch(99.0);
}

void set_measure_params()
{
	//v_ctl_airspeed_mode = AS_MODE_ASP_SIMPLE;
	set_max_roll(NAV_MEASURE_MAX_ROLL);
	set_max_pitch(NAV_MEASURE_MAX_PITCH);
	set_min_pitch(NAV_MEASURE_MIN_PITCH);
}

void set_start_params()
{
	//v_ctl_airspeed_mode = AS_MODE_STANDARD;
	set_max_roll(NAV_START_MAX_ROLL);
	set_max_pitch(NAV_START_MAX_PITCH);
	set_min_pitch(NAV_START_MIN_PITCH);
}

void set_land_params()
{
	//v_ctl_airspeed_mode = AS_MODE_VASSILLIS;
	set_max_roll(NAV_LAND_MAX_ROLL);
	//set_max_pitch(NAV_LAND_MAX_PITCH);
	//set_min_pitch(NAV_LAND_MIN_PITCH);
}

void send_params()
{
	DOWNLINK_SEND_ZHAWPARAMS(DefaultChannel, &h_ctl_roll_max_setpoint, &h_ctl_pitch_max_setpoint, &h_ctl_pitch_min_setpoint, &v_ctl_airspeed_mode);
}
