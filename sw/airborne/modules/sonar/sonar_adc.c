/*
 * Copyright (C) 2011 Bruzzlee
 *
 * This file is part of paparazzi.
 * sonar
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

#include "modules/sonar/sonar_adc.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "estimator.h"
//Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

uint16_t adc_sonar_val;
float sonar_offset;
float sonar_dist;
float sonar_raw;
float sonar_scale;
float sonar_filter;
float sonar_old;

// Local variables
//uint16_t sonar_adc_offset;
//bool_t sonar_adc_offset_init;
//uint32_t sonar_adc_offset_tmp;
//uint16_t sonar_adc_cnt;


//Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef SITL // Use ADC if not in simulation

#ifndef ADC_CHANNEL_SONAR
#error "ADC_CHANNEL_SONAR needs to be defined to use sonar_adc module"
#endif

#ifndef ADC_CHANNEL_SONAR_NB_SAMPLES
#define ADC_CHANNEL_SONAR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#ifndef SONAR_ADC_OFFSET
#define SONAR_ADC_OFFSET 0
#endif
#define SONAR_ADC_OFFSET_NBSAMPLES_INIT 40
#define SONAR_ADC_OFFSET_NBSAMPLES_AVRG 60
#define SONAR_ADC_NBSAMPLES_AVRG 10


struct adc_buf buf_sonar;

#endif

void sonar_adc_init( void ) {
#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_SONAR, &buf_sonar, ADC_CHANNEL_SONAR_NB_SAMPLES);
#endif
	sonar_offset = SONAR_ADC_OFFSET;
	sonar_scale = SONAR_ADC_SCALE;
	sonar_filter = SONAR_ADC_FILTER;
}

void sonar_adc_update( void ) {
#ifndef SITL
	adc_sonar_val = buf_sonar.sum / buf_sonar.av_nb_sample;
	sonar_raw = sonar_scale * adc_sonar_val;
// 	sonar_dist = sonar_raw - sonar_offset;
	
	// 	Lowpass filter
	sonar_dist = sonar_filter * sonar_old + (1 - sonar_filter) * (sonar_raw - sonar_offset);
	sonar_old = sonar_dist;	

//DOWNLINK_SEND_SONAR_ADC(DefaultChannel, &adc_sonar_val, &sonar_raw, &sonar_dist)

#ifdef USE_SONAR
	EstimatorSetAltSonar(sonar_dist);
#endif
	
#ifdef SONAR_SYNC_SEND
	DOWNLINK_SEND_SONAR_ADC(DefaultChannel, &adc_sonar_val, &sonar_raw, &sonar_dist);
#else
	RunOnceEvery(10, DOWNLINK_SEND_SONAR_ADC(DefaultChannel, &adc_sonar_val, &sonar_raw, &sonar_dist));
#endif
#else // SITL

	
#endif //SITL

}
