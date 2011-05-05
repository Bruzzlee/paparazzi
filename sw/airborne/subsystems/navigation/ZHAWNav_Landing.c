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


/************** ZHAW SkidLanding **********************************************/
/*
Landing Routine


  <section name="Landing" prefix="Landing_">
    <define name="AFHeight" value="50" unit="m"/>
    <define name="FinalHeight" value="5" unit="m"/>
    <define name="FinalStageTime" value="5" unit="s"/>
  </section>

 */
/*

#ifndef SonarHeight		//Höhe auf welcher auf Sonar umgeschaltet wird
#define SonarHeight 6
#endif

#ifndef saveHeight		//Höhe bei der in Failsave gegangen werden soll
#define saveHeight 4
#endif
*/


enum LandingStatus { CircleDown, LandingWait, ApproachHeading, DeclineToSonar, Approach, Flare, Stall };
static enum LandingStatus CLandingStatus;
static uint8_t AFWaypoint;
static uint8_t TDWaypoint;
static uint8_t CPWaypoint;
static float LandRadius;
static struct Point2D LandCircle;
static float LandAppAlt;
static float LandCircleQDR;
static float ApproachQDR;
static bool_t AboveCheckPoint;
static bool_t CurrentAboveCheckPoint;

static float FlarePx;
static float FlarePy;
static float LandSlope;
static float DeltaFx;
static float DeltaFy;
static float CheckPx;
static float CheckPy;

static uint8_t FlareStage;
static float Flare_Increment;

static uint8_t msgLandStatus; 



// Variablen für AIRFRAME

static float TDDistance;
static uint8_t FinalLandCount;
static float SonarHeight;
static float SaveHeight;
static float FlareFactor;
static float Land_prePitch;

//*******************

bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, uint8_t CPWP, float radius)
{
	AFWaypoint = AFWP;
	TDWaypoint = TDWP;
	CPWaypoint = CPWP;
	
	CLandingStatus = CircleDown;
	LandRadius = radius;
	LandAppAlt = estimator_z;

	// für Airframe****************
	SaveHeight = 3;			//Höhe für Failsave
	SonarHeight = 6;		//Höhe für Approach
	TDDistance = 25;		//Strecke, auf der geflaret wird
	FlareFactor = 0.5;		//Faktor mit dem die Sollhöhe beim Flare verkleinert wird
	FinalLandCount = 4;		//Anzahl Flareschritte
	Land_prePitch = 0.175		//Soll Pitch für Kill_Throttle schritt
	//********************************
	Flare_Increment=TDDistance/FinalLandCount;


	set_approach_params(); // Parameter für Landung setzten (Airspeed, max_roll, ...)

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
		NavVerticalAutoThrottleMode(0); // No pitch
		
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
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

	  	if(NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(ApproachQDR)))  // Drohne nähert sich dem Winkel (bzw. Kurs) auf dem Sie fliegen muss um zu landen (45° fehlen)
		{
			CLandingStatus = ApproachHeading;
			nav_init_stage();
		}
	msgLandStatus=2;
	break;

//********************** bis hier wie OSAMNav *********************************************



	case ApproachHeading:				//Drohne fliegt den Kreis fertig, bis sie auf Landekurs ist
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0); 	//Sollhöhe geben
		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);	

	  	if(NavQdrCloseTo(DegOfRad(LandCircleQDR)))  //Drohne ist auf Landekurs und auf Höhe AF
		{
			CLandingStatus = DeclineToSonar;
			nav_init_stage();
			AboveCheckPoint = CalculateCheckPoint();
			
			set_max_roll(0.1);
			set_min_pitch(-0.2);
		}
	msgLandStatus=3;
	break;


	case DeclineToSonar: // Sinken, bis Sonar sich meldet
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(waypoints[TDWaypoint].a+SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);

		if(sonar_dist < (SonarHeight + 0.5))
		{
			CLandingStatus = Approach;
			estimator_z_mode = SONAR_HEIGHT;
			nav_init_stage();

			set_max_pitch(0.1);
			set_min_pitch(-0.1);
			set_land_params();
		}
	msgLandStatus=4;
	break;	

	case Approach: //Sonar Höhe (ca. 6m) halten, bis zum FlarePoint, wo die Landung begonnen werden kann. 
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(SonarHeight, 0); //Sonarhöhe ÜBER BODEN!!!!
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);
		
		//find ouf if the UAV has crossed the CheckPoint first time
		if (UAVcrossedCheckPoint() == 1)
		{
			CLandingStatus = Flare;
			nav_init_stage();

			set_max_pitch(0.1);
			set_min_pitch(-0.1);

			TDDistance = TDDistance - Flare_Increment;
			FlareStage = 1;
			SonarHeight = SonarHeight * FlareFactor;
			AboveCheckPoint = CalculateCheckPoint();
		}
	msgLandStatus=5;
	break;

	case Flare:
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);


		if (UAVcrossedCheckPoint() == 1)		//Neuberechnung des CheckPoints wenn überschritten
		{
			TDDistance=TDDistance - Flare_Increment;
			FlareStage++;
			SonarHeight = SonarHeight * FlareFactor;
			AboveCheckPoint = CalculateCheckPoint();
		}


		if(FlareStage == FinalLandCount)
		{
			CLandingStatus = Stall;
			kill_throttle = 1;
			nav_init_stage();
		}
	msgLandStatus=6;
	break;

	case Stall:
		kill_throttle = 1;
		NavVerticalAutoThrottleMode(Land_prePitch); 
  		NavVerticalAltitudeMode(SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);

	msgLandStatus=7;
	break;

	default:

	break;
	}

	RunOnceEvery(1, DOWNLINK_SEND_ZHAWLAND(DefaultChannel, &msgLandStatus, &estimator_z_mode, &AboveCheckPoint, &CurrentAboveCheckPoint, &SonarHeight, &sonar_dist, &estimator_z, ));

	//Failsave Height
	if ((estimator_z < saveHeight) && (CLandingStatus != Flare) && (CLandingStatus != Stall))
	{
		//Mach den Failsave und starte durch!!
		//Möglicherweise muss nur false zurückgegeben werden und im Flightplan muss der Folgepunkt STBY sein!!
		estimator_z_mode=GPS_HEIGHT;
		set_max_pitch(0.3);
		set_min_pitch(0.3);
		set_max_roll(0.5);
		return FALSE;
	}

	//Failsave CheckPoint
	if ((CLandingStatus == DeclineToSonar) && ( UAVcrossedCheckPoint() == 1 ))
	{
		estimator_z_mode=GPS_HEIGHT;
		set_max_pitch(0.3);
		set_min_pitch(0.3);
		set_max_roll(0.5);
		return FALSE;
	}

	return TRUE;
}

bool_t CalculateCheckPoint(void) //TD ist (0/0)
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
	LandSlope = tan(atan2(DeltaFy,DeltaFx)+(3.14/2));			//-1/MLaunch; //90° Drehung der Kurve
	float TouchDownB = (CheckPy - (LandSlope*CheckPx));  				//y-Offset

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
	float Currentx = estimator_x - FlarePx;
	float Currenty = estimator_y - FlarePy;


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
