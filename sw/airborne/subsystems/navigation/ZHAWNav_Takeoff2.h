/*
 * $Id$
 *
 * Copyright (C) 2011  langede0
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
/** \file ZHAWNav_Takeoff2.h
 *  \brief Bungee Takeoff NavLine
 *
 */


#ifndef ZHAWNav2_H
#define ZHAWNav2_H

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"


bool_t calculateTakeOffConditionsNavLine(void);
extern bool_t InitializeZHAWBungeeTakeoffNavLine(uint8_t BungeeWP, uint8_t _TP, uint8_t _NP);
extern bool_t ZHAWBungeeTakeoffNavLine(void);


#endif
