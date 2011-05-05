#include "subsystems/navigation/ZHAWNav_Takeoff.h"
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

#ifndef Takeoff_Distance
#define Takeoff_Distance 10
#endif
#ifndef Takeoff_Speed
#define Takeoff_Speed 12
#endif
#ifndef Takeoff_MinSpeed
#define Takeoff_MinSpeed 2
#endif

enum TakeoffStatus { Launch, Throttle, Finished };
static enum TakeoffStatus CTakeoffStatus;
static float throttlePx;
static float throttlePy;
static float initialx;
static float initialy;
static float ThrottleSlope;
static bool_t AboveThrottleLine;
static float BungeeAlt;
static float TDistance;
static uint8_t TOD;
static uint8_t TP;
static float TakeOff_Height;
static float ThrottleB;
static float Takeoff_MinSpeed_local;
static float deltaTY;
static float deltaTX;

float ThrottleX;
float ThrottleY;


//Berechnet TrottlePoint, ThrottleLine und Seite auf der die Drohne steht (Seite ist der Rückgabewert)
bool_t calculateTakeOffConditions( void )  // TOD ist (0/0)
{	

	//Set InitPos
	initialx = estimator_x;
	initialy = estimator_y;
	
	//Compute deltaTX and deltaTY with TOD=(0/0)
	deltaTX = initialx - (waypoints[TOD].x);
	deltaTY = initialy - (waypoints[TOD].y);

	//Find Launch line slope and Throttle line slope
	float MLaunch = deltaTY/deltaTX; 


	//Compute Throttle Point
	if(deltaTX < 0)
		throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
	else
		throttlePx = - TDistance/sqrt(MLaunch*MLaunch+1);

	if(deltaTY < 0)
		throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
	else
		throttlePy = - sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

		
	//Find ThrottleLine
	ThrottleSlope = tan(atan2(deltaTY,deltaTX)+(3.14/2));			//-1/MLaunch; //90° Drehung der Kurve
	ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));  		//y-Offset


	//Translate ThrottlePoint to absolut
	throttlePx= throttlePx+initialx;
	throttlePy= throttlePy+initialy;

	//Set TrottlePoint in GCS
	waypoints[TP].x= throttlePx;
	waypoints[TP].y= throttlePy;


	//Determine whether the UAV is below or above the throttle line
	if(deltaTY > ((ThrottleSlope*deltaTX)+ThrottleB)) 	//ist UAV über der ThrottleLine?
		return TRUE;					//UAV ist drüber
	else
		return FALSE;

}


bool_t InitializeZHAWBungeeTakeoff(uint8_t TODWP, uint8_t _TP)		//uint8_t _NP muss falls navLine nicht verwendet wird noch entfernt werden!!!!
{
	TOD = TODWP;
	TP = _TP;
	TakeOff_Height = (waypoints[TOD].a);

	
	//** Für Airframe **********************************
	Takeoff_MinSpeed_local=3.0;   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test
					//Takeoff_Distance can only be positive
	TDistance = 9.0; 		//fabs(Takeoff_Distance);!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test
	//**************************************************

	//Record bungee alt (which should be the ground alt at that point)
	BungeeAlt = (waypoints[_TP].a);


	AboveThrottleLine=calculateTakeOffConditions();				//Auf welcher Seite ist dir Drohne


	NavVerticalAutoThrottleMode(0.175);

	//Enable Launch Status and turn kill throttle on
	CTakeoffStatus = Launch;
	kill_throttle = 1; //MOTOR AUS

	return FALSE;
}


bool_t ZHAWBungeeTakeoff(void) 
{
	//Translate the Current Position so that the THROTTLEPOINT is (0/0) (wie weit ist die Drohne noch vom TP weg?)
	float Currentx = estimator_x - throttlePx;
	float Currenty = estimator_y - throttlePy;


	bool_t CurrentAboveThrottleLine;

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(0);				//Set the climb control to auto-throttle with the specified pitch pre-command (navigation.h) -> No Pitch
	  	NavVerticalAltitudeMode(TakeOff_Height, 0.);		//Vorgabe der Sollhöhe
		//nav_route_xy(initialx,initialy,(waypoints[TOD].x),(waypoints[TOD].y));	//Vorgabe der Route
		kill_throttle = 1;	//Motor ausgeschaltet


		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)
			AboveThrottleLine=calculateTakeOffConditions();	
		


		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx) + 0)
			CurrentAboveThrottleLine = TRUE;
		else
			CurrentAboveThrottleLine = FALSE;
				

		RunOnceEvery(10, DOWNLINK_SEND_ZHAWTAKEOFF(DefaultChannel, &AboveThrottleLine, &CurrentAboveThrottleLine));


		//Find out if UAV has crossed the line
		if(AboveThrottleLine != CurrentAboveThrottleLine && estimator_hspeed_mod > Takeoff_MinSpeed_local)
		{
			CTakeoffStatus = Throttle;
			kill_throttle = 0;
			nav_init_stage();
			ThrottleX = estimator_x;
			ThrottleY = estimator_y;	
			NavVerticalAutoThrottleMode(0);	
		}
		break;

	case Throttle:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(AGR_CLIMB_PITCH); 	
		NavVerticalThrottleMode(9600*(1));		
		nav_route_xy(ThrottleX,ThrottleY,(waypoints[TOD].x),(waypoints[TOD].y));
		kill_throttle = 0;

		if((estimator_z > TakeOff_Height-10) && (estimator_hspeed_mod > Takeoff_Speed))
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



//***************************** Bungee Takeoff glide******************************************************************


bool_t ZHAWBungeeTakeoff_glide(void)
{
//Translate the Current Position so that the THROTTLEPOINT is (0/0) (wie weit ist die Drohne noch vom TP weg?)
	float Currentx = estimator_x - throttlePx;
	float Currenty = estimator_y - throttlePy;

	bool_t CurrentAboveThrottleLine;

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(0);				//Set the climb control to auto-throttle with the specified pitch pre-command (navigation.h) -> No Pitch
	  	NavVerticalAltitudeMode(TakeOff_Height, 0.);		//Vorgabe der Sollhöhe
		//nav_route_xy(initialx,initialy,(waypoints[TOD].x),(waypoints[TOD].y));	//Vorgabe der Route
		kill_throttle = 1;	//Motor ausgeschaltet


		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)
			AboveThrottleLine=calculateTakeOffConditions();	
		


		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx) + 0)
			CurrentAboveThrottleLine = TRUE;
		else
			CurrentAboveThrottleLine = FALSE;
				

		RunOnceEvery(10, DOWNLINK_SEND_ZHAWTAKEOFF(DefaultChannel, &AboveThrottleLine, &CurrentAboveThrottleLine));


		//Find out if UAV has crossed the line
		if(AboveThrottleLine != CurrentAboveThrottleLine && estimator_hspeed_mod > Takeoff_MinSpeed_local)
		{
			CTakeoffStatus = Finished;
			kill_throttle = 0;
			nav_init_stage();
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


