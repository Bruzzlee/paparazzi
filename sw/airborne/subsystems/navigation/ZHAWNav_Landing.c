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
/** \file ZHAWNav_Landing.c
 *  \brief Advanced landing with ultrasonic sensor
 *
 */




#include "subsystems/navigation/ZHAWNav_Landing.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/sonar/sonar_adc.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include "subsystems/navigation/parameter_changer.h"
#include "estimator.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


/************** ZHAW Landing fixed Pitch**********************************************/
/*
Landing Routine


  <section name="ZHAWLanding" prefix="Landing_">
    <define name="SaveHeight" value="3" unit="m"/>
    <define name="SonarHeight" value="6" unit="m"/>
    <define name="TDDistance" value="50" unit="s"/>
    <define name="FlareFactor" value="5"/>
  </section>

 */

#ifndef Landing_SaveHeight		//Height for Failsave
#define Landing_SaveHeight 3
#endif

#ifndef Landing_SonarHeight		//Height to Switch to Sonar
#define Landing_SonarHeight 6
#endif

#ifndef Landing_TDDistance		//Flare Disatnce
#define Landing_TDDistance 50
#endif

#ifndef Landing_FlareFactor		//Faktor mit dem die Sollhöhe beim Flare verkleinert wird
#define Landing_FlareFactor 0.5
#endif



enum LandingStatus { CircleDown, LandingWait, ApproachHeading, DeclineToSonar, Approach, Flare, Stall };
static enum LandingStatus CLandingStatus;
static struct Point2D LandCircle;
static uint8_t AFWaypoint;
static uint8_t TDWaypoint;
static uint8_t CPWaypoint;
static uint8_t msgLandStatus;
static float LandRadius;
static float LandAppAlt;
static float LandCircleQDR;
static float ApproachQDR;
static float LandSlope;
static float DeltaFx;
static float DeltaFy;
static float CheckPx;
static float CheckPy;
static float TDDistance;
static float SonarHeight;
static bool_t AboveCheckPoint;
static bool_t CurrentAboveCheckPoint;


bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, uint8_t CPWP, float radius)
{
	AFWaypoint = AFWP;
	TDWaypoint = TDWP;
	CPWaypoint = CPWP;
	
	CLandingStatus = CircleDown;
	LandRadius = radius;
	LandAppAlt = estimator_z;
	TDDistance = fabs(Landing_TDDistance); 		//TDDistance can only be positive
	SonarHeight = fabs(Landing_SonarHeight);	//SonarHeight can only be positive

	/* 
	// für Airframe****************
	SaveHeight = 3;			//Höhe für Failsave
	SonarHeight = 6;		//Höhe für Approach
	TDDistance = 50;		//Strecke, auf der geflaret wird
	FlareFactor = 0.5;		//Faktor mit dem die Sollhöhe beim Flare verkleinert wird
	// ******************************** 
	*/

	set_approach_params();  // Parameter für Landung setzten (Airspeed, max_roll, ...)
	set_as_mode(3);		// Airspeed Pitch Simple
		
	//Translate distance from AF to TD so that AF is (0/0) 
	float x_0 = waypoints[TDWaypoint].x - waypoints[AFWaypoint].x;
	float y_0 = waypoints[TDWaypoint].y - waypoints[AFWaypoint].y;

	// Unit vector from AF to TD
	float d = sqrt(x_0*x_0+y_0*y_0);	//d=Horizontale Strecke von AF zu TD
	float x_1 = x_0 / d;
	float y_1 = y_0 / d;

	//find the center of LandCircle
	LandCircle.x = waypoints[AFWaypoint].x + y_1 * LandRadius;
	LandCircle.y = waypoints[AFWaypoint].y - x_1 * LandRadius;


	//Compute the QDR-Angles
	LandCircleQDR = atan2(waypoints[AFWaypoint].x-LandCircle.x, waypoints[AFWaypoint].y-LandCircle.y);

	if(LandRadius > 0)
	{
		ApproachQDR = LandCircleQDR-RadOfDeg(90);
		LandCircleQDR = LandCircleQDR-RadOfDeg(45);
	}
	else
	{
		ApproachQDR = LandCircleQDR+RadOfDeg(90);
		LandCircleQDR = LandCircleQDR+RadOfDeg(45);
	}
	

	return FALSE;
}

bool_t ZHAWSkidLanding(void)
{
	switch(CLandingStatus)
	{
	case CircleDown: // Kreisen bis die Höhe, die im AFWaypoint vorgegeben ist erreicht ist (um den Wegpunkt der in InitializeSkidLanding berechnet wurde)
		
		if(NavCircleCount() < .1)
		{
	  		NavVerticalAltitudeMode(LandAppAlt, 0);  
  		}
		else
			NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);


		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius); 

		if(estimator_z < waypoints[AFWaypoint].a + 5) //Drohne hat die Höhe des AF_Waypoints erreicht. 
		{
			CLandingStatus = LandingWait;
			nav_init_stage();
		}
	msgLandStatus=1;
	break;

	case LandingWait: // Höhe halten und weiter um CircleCircle kreisen  
  		NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

	  	if(NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(ApproachQDR)))  // Drohne nähert sich dem Winkel (bzw. Kurs) auf dem Sie fliegen muss um zu landen (45° fehlen)
		{
			CLandingStatus = ApproachHeading;
			nav_init_stage();
		}
	msgLandStatus=2;
	break;


	case ApproachHeading:				//Drohne fliegt den Kreis fertig, bis sie auf Landekurs ist
  		NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0); 	//Sollhöhe geben
		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);	

	  	if(NavQdrCloseTo(DegOfRad(LandCircleQDR)))  //Drohne ist auf Landekurs und auf Höhe AF
		{
			CLandingStatus = DeclineToSonar;
			nav_init_stage();
		}
	msgLandStatus=3;
	break;


	case DeclineToSonar: // Sinken, bis Sonar sich meldet
  		NavVerticalAltitudeMode(waypoints[TDWaypoint].a+Landing_SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);

		if(sonar_dist < (Landing_SonarHeight + 0.5))
		{
			CLandingStatus = Approach;
			estimator_z_mode = SONAR_HEIGHT;
			nav_init_stage();
			AboveCheckPoint = CalculateCheckPoint();
		}
	msgLandStatus=4;
	break;	

	case Approach: //Sonar Höhe (ca. 6m) halten, bis zum FlarePoint, wo die Landung begonnen werden kann. 
  		NavVerticalAltitudeMode(Landing_SonarHeight, 0); //Sonarhöhe ÜBER BODEN!!!!
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);
		
		//find ouf if the UAV has crossed the CheckPoint first time
		if (UAVcrossedCheckPoint() == 1)
		{
			CLandingStatus = Flare;
			nav_init_stage();
			SonarHeight = SonarHeight * Landing_FlareFactor;
			kill_throttle = 1;
			set_fixed_pitch_pitch(0.2);
		}
	msgLandStatus=5;
	break;

	case Flare:
  		NavVerticalAltitudeMode(Landing_SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);


		if (estimator_z < 0.5 && estimator_z > 0.25)
		{
			set_fixed_pitch_pitch(0.2);	
		}
		if (estimator_z < 0.25)
		{
			set_fixed_pitch_pitch(0.3);	
		}

	msgLandStatus=6;
	break;

	case Stall:
		kill_throttle = 1;
		NavVerticalAltitudeMode(Landing_SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);

	msgLandStatus=7;
	break;

	default:

	break;
	}

	RunOnceEvery(5, DOWNLINK_SEND_ZHAWLAND(DefaultChannel, &msgLandStatus, &estimator_z_mode, &AboveCheckPoint, &CurrentAboveCheckPoint, &SonarHeight, &estimator_z_sonar, &estimator_z));

	//Failsave Height
	if ((estimator_z < Landing_SaveHeight) && (CLandingStatus != Flare) && (CLandingStatus != Stall))
	{
		estimator_z_mode=GPS_HEIGHT;
		set_max_pitch(99);
		set_min_pitch(99);
		set_max_roll(99);
		return FALSE;
	}

	//Failsave CheckPoint
	if ((CLandingStatus == DeclineToSonar) && ( UAVcrossedCheckPoint() == 1 ))
	{
		estimator_z_mode=GPS_HEIGHT;
		set_max_pitch(99);
		set_min_pitch(99);
		set_max_roll(99);
		return FALSE;
	}

	return TRUE;
}

bool_t CalculateCheckPoint(void) //TD is (0/0)
{

	//Compute DeltaFx and DeltaFy between AF an TD with TD=(0/0)
	float deltaCX = (waypoints[AFWaypoint].x) - (waypoints[TDWaypoint].x);
	float deltaCY = (waypoints[AFWaypoint].y) - (waypoints[TDWaypoint].y);

	//Find Land line slope and Throttle line slope
	float MLaunch = deltaCY/deltaCX; 

	//Compute Flare Point
	if(DeltaFx < 0)
		CheckPx = TDDistance/sqrt(MLaunch*MLaunch+1);
	else
		CheckPx = - TDDistance/sqrt(MLaunch*MLaunch+1);

	if(DeltaFy < 0)
		CheckPy = sqrt((TDDistance*TDDistance)-(CheckPx*CheckPx));
	else
		CheckPy = - sqrt((TDDistance*TDDistance)-(CheckPx*CheckPx));

	//Find TouchDownLine
	LandSlope = tan(atan2(DeltaFy,DeltaFx)+(3.14/2));			
	float TouchDownB = (CheckPy - (LandSlope*CheckPx));

	//Translate CheckPoint to absolut
	CheckPx= CheckPx + (waypoints[TDWaypoint].x);
	CheckPy= CheckPy + (waypoints[TDWaypoint].y);

	//Set CheckPoint in GCS
	waypoints[CPWaypoint].x= CheckPx;
	waypoints[CPWaypoint].y= CheckPy;

	//Determine whether the UAV is below or above the CheckPoint
	if(DeltaFy > ((LandSlope*deltaCX)+TouchDownB)) 	
		return TRUE;
	else
		return FALSE;

}

bool_t UAVcrossedCheckPoint (void)
{
	//Translate the Current Position so that the FLAREPOINT is (0/0) (wie weit ist die Drohne noch vom FP weg?)
	float Currentx = estimator_x - CheckPx;
	float Currenty = estimator_y - CheckPy;


	//Find out if the UAV is currently above the line
	if(Currenty > (LandSlope*Currentx) + 0)
		CurrentAboveCheckPoint = TRUE;
	else
		CurrentAboveCheckPoint = FALSE;

	if(AboveCheckPoint != CurrentAboveCheckPoint)
	{
		return TRUE;		
	}

	return FALSE;

}
