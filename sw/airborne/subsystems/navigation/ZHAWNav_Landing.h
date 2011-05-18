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
/** \file ZHAWNav_Landing.h
 *  \brief Advanced landing with ultrasonic sensor
 *
 */



#ifndef ZHAWNav_H_Land
#define ZHAWNav_H_Land

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"


extern bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, uint8_t FPWP, float radius);
extern bool_t ZHAWSkidLanding(void);
bool_t CalculateCheckPoint(void);
bool_t UAVcrossedCheckPoint (void);


#endif
