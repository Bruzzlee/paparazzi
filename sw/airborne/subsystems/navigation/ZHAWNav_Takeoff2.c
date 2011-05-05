#include "subsystems/navigation/ZHAWNav_Takeoff2.h"
#include "firmwares/rotorcraft/autopilot.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


/************** ZHAW Bungee Takeoff **********************************************/

/** Takeoff functions for bungee takeoff.
Run initialize function when the plane is on the bungee, the bungee is fully extended and you are ready to
launch the plane. After initialized, the plane will follow a line drawn by the position of the plane on initialization and the
position of the WP_TakeOffDirection (given in the arguments). Once the plane crosses the throttle line, which is perpendicular to the line the plane is following,
and intersects the position of the ThrottlePoint (which has a fixed distance from the Initial Position (TakeOff_Distance in airframe file) from the bungee just in case the bungee doesn't release directly above the bungee) the prop will come on. The plane will then continue to follow the line until it has reached a specific
height (defined in as Takeoff_Height in airframe file) above the bungee waypoint and speed (defined as Takeoff_Speed in the airframe file).

<section name="Takeoff" prefix="Takeoff_">
  <define name="Speed" value="15" unit="m/s"/>
  <define name="Distance" value="10" unit="m"/>
  <define name="MinSpeed" value="5" unit="m/s"/>
</section>
 */
/*
#ifndef Takeoff_Distance
#define Takeoff_Distance 10
#endif
#ifndef Takeoff_Speed
#define Takeoff_Speed 6
#endif
#ifndef Takeoff_MinSpeed
#define Takeoff_MinSpeed 2
#endif*/

enum TakeoffStatus { Launch, Stabilization, Climb, Finished };
static enum TakeoffStatus CTakeoffStatus;
static float throttlePx;
static float throttlePy;
static float navPx;
static float navPy;
static float initialx;
static float initialy;
static float ThrottleSlope;
static bool_t AboveLines;
static float BungeeAlt;
static float TDistance;
static float NDistance;
static uint8_t TOD;
static uint8_t TP;
static uint8_t NP;
static float TakeOff_Height;
static float ThrottleB;
static float NavB;
static float Takeoff_MinSpeed_local;
static float deltaTY;
static float deltaTX;

float StartThrottle_X;
float StartThrottle_Y;


//Berechnet TrottlePoint, ThrottleLine und Seite auf der die Drohne steht (Seite ist der Rückgabewert)
bool_t calculateTakeOffConditionsNavLine( void )  // TOD ist (0/0)
{	

	//Set InitPos
	initialx = estimator_x;
	initialy = estimator_y;
	
	//Compute deltaTX and deltaTY with TOD=(0/0)
	deltaTX = initialx - (waypoints[TOD].x);
	deltaTY = initialy - (waypoints[TOD].y);


	//Find Launch line slope and Throttle line slope
	float MLaunch = deltaTY/deltaTX; 


	//Compute Throttle Point and Nav Point
	if(deltaTX < 0)
	{
		throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
		navPx = NDistance/sqrt(MLaunch*MLaunch+1);
	}
	else
	{
		throttlePx = - TDistance/sqrt(MLaunch*MLaunch+1);
		navPx = - NDistance/sqrt(MLaunch*MLaunch+1);
	}

	if(deltaTY < 0)
	{
		throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
		navPy = sqrt((NDistance*NDistance)-(navPx*navPx));
	}
	else
	{
		throttlePy = - sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
		navPy = - sqrt((NDistance*NDistance)-(navPx*navPx));
	}

		
	//Find ThrottleLine and NavLine
	ThrottleSlope = tan(atan2(deltaTY,deltaTX)+(3.14/2));		//NavLineSlope the same like ThrottleSlope 
	ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));  	//ThrottleBase
	NavB = (navPy - (ThrottleSlope*navPx));				//NavBase


	//Translate ThrottlePoint and NavPoint to absolut
	throttlePx= throttlePx+initialx;
	throttlePy= throttlePy+initialy;
	navPx= navPx+initialx;
	navPy= navPy+initialy;


	//Set TrottlePoint and NavPoint in GCS
	waypoints[TP].x= throttlePx;
	waypoints[TP].y= throttlePy;
	waypoints[NP].x= navPx;
	waypoints[NP].y= navPy;



	//Determine whether the UAV is below or above the throttle line and the NavLine
	if(deltaTY > ((ThrottleSlope*deltaTX)+ThrottleB)) 	//ist UAV über der ThrottleLine?
		return TRUE;					//UAV ist drüber
	else
		return FALSE;

}


bool_t InitializeZHAWBungeeTakeoffNavLine(uint8_t TODWP, uint8_t _TP, uint8_t _NP)
{
	TOD = TODWP;
	TP = _TP;
	NP = _NP;
	TakeOff_Height = (waypoints[TOD].a);

	Takeoff_MinSpeed_local=3.0;   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test


	//Takeoff_Distance can only be positive
	TDistance = 9.0; 		//fabs(Takeoff_Distance);!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test
	NDistance = 30.0;					//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test

	//Record bungee alt (which should be the ground alt at that point)
	BungeeAlt = (waypoints[_TP].a);


	AboveLines=calculateTakeOffConditionsNavLine();				//Auf welcher Seite ist dir Drohne


	//Enable Launch Status and turn kill throttle on
	CTakeoffStatus = Launch;
	kill_throttle = 1; //MOTOR AUS

	return FALSE;
}


bool_t ZHAWBungeeTakeoffNavLine(void) 
{
	//Translate the Current Position so that the THROTTLEPOINT is (0/0) (wie weit ist die Drohne noch vom TP weg?)
	float CurrentThrottlex = estimator_x - throttlePx;
	float CurrentThrottley = estimator_y - throttlePy;

	//Translate the Current Position so that the NAVPOINT is (0/0) (wie weit ist die Drohne noch vom NP weg?)
	float CurrentNavx = estimator_x - navPx;
	float CurrentNavy = estimator_y - navPy;

	bool_t CurrentAboveThrottleLine;
	bool_t CurrentAboveNavLine;

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(0);				//Set the climb control to auto-throttle with the specified pitch pre-command (navigation.h) -> No Pitch
	  	NavVerticalAltitudeMode(estimator_z + 20, 0.);		//Vorgabe der Sollhöhe
		kill_throttle = 1;					//Motor ausgeschaltet


		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)
			AboveLines=calculateTakeOffConditionsNavLine();	
		


		//Find out if the UAV is currently above the Throttle line
		if(CurrentThrottley > (ThrottleSlope*CurrentThrottlex) + 0)
			CurrentAboveThrottleLine = TRUE;
		else
			CurrentAboveThrottleLine = FALSE;
				

		RunOnceEvery(10, DOWNLINK_SEND_ZHAWTAKEOFF(DefaultChannel, &AboveLines, &CurrentAboveThrottleLine));


		//Find out if UAV has crossed the Throttle Line
		if(AboveLines != CurrentAboveThrottleLine && estimator_hspeed_mod > Takeoff_MinSpeed_local)
		{
			CTakeoffStatus = Stabilization;
			kill_throttle = 0;
			nav_init_stage();
			NavVerticalAltitudeMode(estimator_z+10, 0.); 		//Höhe halten in Stabilisations-Zwischenschritt
					
		}
		break;

	case Stabilization:							//Höhe halten, nicht navigieren, Motor einschalten	
		NavVerticalAutoThrottleMode(AGR_CLIMB_PITCH);
		NavVerticalThrottleMode(9600*(1));		
		kill_throttle = 0;


		//Find out if the UAV is currently above the Nav line
		if(CurrentNavy > (ThrottleSlope*CurrentNavx) + 0)
			CurrentAboveNavLine = TRUE;
		else
			CurrentAboveNavLine = FALSE;
		


		if (AboveLines != CurrentAboveNavLine)
		{
			CTakeoffStatus = Climb;
			nav_init_stage();
			StartThrottle_X=estimator_x;
			StartThrottle_Y=estimator_y;
		}

	case Climb:
		NavVerticalAutoThrottleMode(AGR_CLIMB_PITCH);
		nav_route_xy(StartThrottle_X,StartThrottle_Y,(waypoints[TOD].x),(waypoints[TOD].y));
		NavVerticalAltitudeMode(TakeOff_Height, 0.);

		if(estimator_z > (TakeOff_Height-10))
		{
			CTakeoffStatus = Finished;
			return FALSE;
		}
		else
		{
			return TRUE;
		}
		break;
	default:
		break;
	}
	return TRUE;
}


