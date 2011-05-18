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
/** \file parameter_changer.h
 *  \brief Change parameter min/max of pitch and roll and airspeed-mode
 *
 */

#ifndef PARAMETER_CHANGER_H
#define PARAMETER_CHANGER_H

#include "std.h"

//#include "subsystems/navigation/parameter_changer.c"

extern bool_t set_as_mode(uint8_t as_mode_set);

extern void set_max_roll( float max_roll );
extern void set_max_pitch(float max_pitch);
extern void set_min_pitch(float min_pitch);

extern void set_approach_params( void );
extern void set_measure_params( void );
extern void set_start_params( void );
extern void set_land_params( void );
extern void set_fixed_pitch_pitch(float fixedpitch);

void send_params( void );

#endif
