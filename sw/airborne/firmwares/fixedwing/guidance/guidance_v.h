/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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
 *
 * fixed wing vertical control
 *
 */

#ifndef FW_V_CTL_H
#define FW_V_CTL_H

#include <inttypes.h>
#include "paparazzi.h"

/* Vertical mode */
#define V_CTL_MODE_MANUAL        0
#define V_CTL_MODE_AUTO_THROTTLE 1
#define V_CTL_MODE_AUTO_CLIMB    2
#define V_CTL_MODE_AUTO_ALT      3
#define V_CTL_MODE_NB            4
extern uint8_t v_ctl_mode;

/* outer loop */
extern float v_ctl_altitude_error;
extern float v_ctl_altitude_setpoint;
extern float v_ctl_altitude_pre_climb;
extern float v_ctl_altitude_pgain;

/* inner loop */
extern float v_ctl_climb_setpoint;
extern uint8_t v_ctl_climb_mode;
#define V_CTL_CLIMB_MODE_AUTO_THROTTLE 0
#define V_CTL_CLIMB_MODE_AUTO_PITCH    1

extern uint8_t v_ctl_auto_throttle_submode;
#define V_CTL_AUTO_THROTTLE_STANDARD  0
#define V_CTL_AUTO_THROTTLE_AGRESSIVE 1
#define V_CTL_AUTO_THROTTLE_BLENDED   2

/* "auto throttle" inner loop parameters */
extern float v_ctl_auto_throttle_nominal_cruise_throttle;
extern float v_ctl_auto_throttle_cruise_throttle;
extern float v_ctl_auto_throttle_climb_throttle_increment;
extern float v_ctl_auto_throttle_pgain;
extern float v_ctl_auto_throttle_igain;
extern float v_ctl_auto_throttle_dgain;
extern float v_ctl_auto_throttle_sum_err;
extern float v_ctl_auto_throttle_pitch_of_vz_pgain;
extern float v_ctl_auto_throttle_pitch_of_vz_dgain;

#ifdef LOITER_TRIM
extern float v_ctl_auto_throttle_loiter_trim;
extern float v_ctl_auto_throttle_dash_trim;
#endif

/* agressive tuning */
#ifdef TUNE_AGRESSIVE_CLIMB
extern float agr_climb_throttle;
extern float agr_climb_pitch;
extern float agr_climb_nav_ratio;
extern float agr_descent_throttle;
extern float agr_descent_pitch;
extern float agr_descent_nav_ratio;
#endif

/* "auto pitch" inner loop parameters */

extern float h_ctl_pitch_min_setpoint;
extern float h_ctl_pitch_max_setpoint;
//extern float v_ctl_auto_pitch_pgain;
//extern float v_ctl_auto_pitch_igain;
extern float v_ctl_auto_pitch_sum_err;

extern pprz_t v_ctl_throttle_setpoint;
extern pprz_t v_ctl_throttle_slewed;

extern void v_ctl_init( void );
extern void v_ctl_altitude_loop( void );
extern void v_ctl_climb_loop ( void );

// Airspeed Modes
#define  AS_MODE_STANDARD	0
#define  AS_MODE_VASSILLIS	1
#define  AS_MODE_ASP_CLIMBRATE	2
#define  AS_MODE_ASP_SIMPLE	3
#define  AS_MODE_ASP_MANUAL	4
#define  AS_MODE_ASP_ACCEL	5
#define  AS_MODE_FIX_PITCH	6
extern uint8_t v_ctl_airspeed_mode;

#ifdef USE_AIRSPEED
extern float v_ctl_auto_airspeed_deadband;
/* "airspeed" inner loop parameters */
extern float v_ctl_auto_airspeed_setpoint;
extern float v_ctl_auto_airspeed_controlled;
extern float v_ctl_auto_airspeed_sum_err;
extern float v_ctl_auto_groundspeed_setpoint;
extern float v_ctl_auto_groundspeed_pgain;
extern float v_ctl_auto_groundspeed_igain;
extern float v_ctl_auto_groundspeed_sum_err;

//Vassillis
extern float v_ctl_auto_airspeed_throttle_pgain_v;
extern float v_ctl_auto_airspeed_throttle_igain_v;
extern float v_ctl_auto_airspeed_pitch_pgain_v;
extern float v_ctl_auto_airspeed_pitch_igain_v;
//AirSpeed Pitch Climbrate
extern float v_ctl_auto_airspeed_throttle_pgain_aspc;
extern float v_ctl_auto_airspeed_throttle_igain_aspc;
extern float v_ctl_auto_airspeed_prethrottle_aspc;
extern float v_ctl_auto_airspeed_pitch_pgain_aspc;
extern float v_ctl_auto_airspeed_pitch_igain_aspc;
//AirSpeed Pitch Simple
extern float v_ctl_auto_airspeed_throttle_pgain_asps;
extern float v_ctl_auto_airspeed_throttle_igain_asps;
extern float v_ctl_auto_airspeed_prethrottle_asps;
extern float v_ctl_auto_airspeed_pitch_pgain_asps;
extern float v_ctl_auto_airspeed_pitch_igain_asps;
//AirSpeed Manual Power
extern float v_ctl_auto_airspeed_throttlesetp_asmp;
extern float v_ctl_auto_airspeed_pitch_pgain_asmp;
extern float v_ctl_auto_airspeed_pitch_igain_asmp;
//AirSpeed Pitch Acceleration
extern float v_ctl_auto_airspeed_throttle_pgain_aspa;
extern float v_ctl_auto_airspeed_throttle_igain_aspa;
extern float v_ctl_auto_airspeed_prethrottle_aspa;
extern float v_ctl_auto_airspeed_pitch_pgain_aspa;
extern float v_ctl_auto_airspeed_pitch_igain_aspa;
extern float v_ctl_airspeed_acc_filter_value;
extern float v_ctl_accel_pgain;
extern float v_ctl_altitude_max_accel;
//"Airspeed" with fixed Pitch
extern float v_ctl_auto_airspeed_pitchsetp_fp;
extern float v_ctl_auto_airspeed_throttle_pgain_fp;
extern float v_ctl_auto_airspeed_throttle_igain_fp;
extern float v_ctl_auto_airspeed_prethrottle_fp;
#endif

/** Computes throttle_slewed from throttle_setpoint */
extern void v_ctl_throttle_slew( void );

#define guidance_v_SetCruiseThrottle(_v) { \
  v_ctl_auto_throttle_cruise_throttle = (_v ? _v : v_ctl_auto_throttle_nominal_cruise_throttle); \
  Bound(v_ctl_auto_throttle_cruise_throttle, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE); \
}

#define guidance_v_SetAutoThrottleIgain(_v) {	\
    v_ctl_auto_throttle_igain = _v;		\
    v_ctl_auto_throttle_sum_err = 0;		\
  }

#endif /* FW_V_CTL_H */
