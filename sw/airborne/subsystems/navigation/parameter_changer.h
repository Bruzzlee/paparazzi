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

extern bool_t set_max_roll(float max_roll);
extern bool_t set_max_pitch(float max_pitch)
extern bool_t set_min_pitch(float min_pitch)

extern bool_t set_approach_params()
extern bool_t set_measure_params()
extern bool_t set_start_params()
extern bool_t set_land_params()


