/*
* Driver for the EagleTree Systems Airspeed Sensor
* Has only been tested with V3 of the sensor hardware
*
* Notes:
* Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
* Sensor should be in the proprietary mode (default) and not in 3rd party mode.
*
* Sensor module wire assignments:
* Red wire: 5V
* White wire: Ground
* Yellow wire: SDA
* Brown wire: SCL
*
* Copyright (C) 2009 Vassilis Varveropoulos
* Modified by Mark Griffin on 8 September 2010 to work with new i2c transaction routines.
* Converted by Gautier Hattenberger to modules (10/2010)
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*
*/

#ifndef AIRSPEED_AMSYS_H
#define AIRSPEED_AMSYS_H

#include "std.h"
#include "mcu_periph/i2c.h"

extern uint16_t airspeed_amsys_raw;
extern float airspeed_amsys_offset;
extern bool_t airspeed_amsys_valid;
extern float airspeed_amsys;
extern float airspeed_scale;
extern float airspeed_filter;

extern struct i2c_transaction airspeed_amsys_i2c_trans;

extern void airspeed_amsys_init( void );
extern void airspeed_amsys_read_periodic( void );
extern void airspeed_amsys_read_event( void );

#define AirspeedAmsysEvent() { if (airspeed_amsys_i2c_trans.status == I2CTransSuccess) airspeed_amsys_read_event(); }

#endif // AIRSPEED_AMSYS_H
