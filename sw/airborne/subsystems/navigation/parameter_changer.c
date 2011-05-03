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

// #include <inttypes.h>
#include "subsystems/navigation/paramter_changer.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
// #include "generated/airframe.h"

// uint8_t acs_idx;
// uint8_t the_acs_id[NB_ACS_ID];
// struct ac_info_ the_acs[NB_ACS];

// void traffic_info_init( void ) {
//   the_acs_id[0] = 0;  // ground station
//   the_acs_id[AC_ID] = 1;
//   the_acs[the_acs_id[AC_ID]].ac_id = AC_ID;
//   acs_idx = 2;
// }$

bool_t set_max_roll(float max_roll)
{
	if(max_roll<10.0){
		h_ctl_roll_max_setpoint = max_roll;
		return true
	}
	else
		h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;
	return false
}

bool_t set_max_pitch(float max_pitch)
{
	if(max_roll<10.0){
		h_ctl_pitch_max_setpoint = max_pitch;
		return true
	}
	else
		h_ctl_pitch_max_setpoint = H_CTL_PITCH_MAX_SETPOINT;
	return false
}

bool_t set_min_pitch(float min_pitch)
{
	if(max_roll<10.0){
		h_ctl_pitch_min_setpoint = min_pitch;
		return true
	}
	else
		h_ctl_pitch_min_setpoint = H_CTL_PITCH_MIN_SETPOINT;
	return false
}

bool_t set_approach_params()
{
	v_ctl_airspeed_mode = AS_MODE_ASP_SIMPLE;
		if (!set_max_roll(99.0) && !set_max_pitch(99.0) && !set_min_pitch(99.0)) // set default Params
		return true
	else
		return false
}

bool_t set_measure_params()
{
	v_ctl_airspeed_mode = AS_MODE_ASP_SIMPLE;
	if (set_max_roll(NAV_MEASURE_MAX_ROLL) && set_max_pitch(NAV_MEASURE_MAX_PITCH) && set_min_pitch(NAV_MEASURE_MIN_PITCH))
		return true
	else
		return false
}

bool_t set_start_params()
{
	v_ctl_airspeed_mode = AS_MODE_STANDARD;
	if (set_max_roll(NAV_START_MAX_ROLL) && set_max_pitch(NAV_START_MAX_PITCH) && set_min_pitch(NAV_START_MIN_PITCH))
		return true
	else
		return false
}

bool_t set_land_params()
{
	v_ctl_airspeed_mode = AS_MODE_VASSILLIS;
	if (set_max_roll(NAV_LAND_MAX_ROLL) && set_max_pitch(NAV_LAND_MAX_PITCH) && set_min_pitch(NAV_LAND_MIN_PITCH))
		return true
	else
		return false
}


// struct ac_info_ * get_ac_info(uint8_t id) {
//   return &the_acs[the_acs_id[id]];
// }
