#include "subsystems/navigation/ZHAWNav.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/sonar/sonar_adc.h"


/************** ZHAW Bungee Takeoff **********************************************/

/** Takeoff functions for bungee takeoff.
Run initialize function when the plane is on the bungee, the bungee is fully extended and you are ready to
launch the plane. After initialized, the plane will follow a line drawn by the position of the plane on initialization and the
position of the WP_TakeOffDirection (given in the arguments). Once the plane crosses the throttle line, which is perpendicular to the line the plane is following,
and intersects the position of the ThrottlePoint (which has a fixed distance from the Initial Position (TakeOff_Distance in airframe file) from the bungee just in case the bungee doesn't release directly above the bungee) the prop will come on. The plane will then continue to follow the line until it has reached a specific
height (defined in as Takeoff_Height in airframe file) above the bungee waypoint and speed (defined as Takeoff_Speed in the airframe file).

<section name="Takeoff" prefix="Takeoff_">
  <define name="Height" value="30" unit="m"/>
  <define name="Speed" value="15" unit="m/s"/>
  <define name="Distance" value="10" unit="m"/>
  <define name="MinSpeed" value="5" unit="m/s"/>
</section>
 */

#ifndef Takeoff_Distance
#define Takeoff_Distance 10
#endif
//#ifndef Takeoff_Height
//#define Takeoff_Height 30
//#endif
#ifndef Takeoff_Speed
#define Takeoff_Speed 15
#endif
#ifndef Takeoff_MinSpeed
#define Takeoff_MinSpeed 5
#endif

enum TakeoffStatus { Launch, Throttle, Finished };
static enum TakeoffStatus CTakeoffStatus;
static float throttlePx;
static float throttlePy;
static float initialx;
static float initialy;
static float ThrottleSlope;
static bool_t AboveLine;
static float BungeeAlt;
static float TDistance;
static uint8_t BungeeDirection;
static float TakeOff_Height;
static float ThrottleB;

bool_t InitializeZHAWBungeeTakeoff(uint8_t DirectionWP, uint8_t _TP)
{
	
	initialx = estimator_x;
	initialy = estimator_y;
	BungeeDirection = DirectionWP;
	TakeOff_Height = (waypoints[BungeeDirection].a);


	//Takeoff_Distance can only be positive
	TDistance = fabs(Takeoff_Distance);

	//Record bungee alt (which should be the ground alt at that point)
	BungeeAlt = (waypoints[_TP].a);

	//Translate the Current Position so that the Initial Position is (0/0)
	float Currentx = (waypoints[BungeeDirection].x)-initialx;
	float Currenty = (waypoints[BungeeDirection].y)-initialy;

	//Find Launch line slope and Throttle line slope
	float MLaunch = Currenty/Currentx; 


	//Compute Throttle Point
	if(Currentx > 0)
		throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
	else
		throttlePx = - TDistance/sqrt(MLaunch*MLaunch+1);

	if(Currenty > 0)
		throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
	else
		throttlePy = - sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

		
	//Find ThrottleLine
	ThrottleSlope = -1/MLaunch; //90° Drehung der Kurve
	ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));  //y-Offset


	//Determine whether the UAV is below or above the throttle line
	if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB)) 	//ist UAV über der ThrottleLine?
		AboveLine = TRUE;				//UAV ist drüber
	else
		AboveLine = FALSE;				//UAV ist drunter

	//Enable Launch Status and turn kill throttle on
	CTakeoffStatus = Launch;
	kill_throttle = 1; //MOTOR AUS

	//Translate the throttle point back
	throttlePx = throttlePx+initialx;
	throttlePy = throttlePy+initialy;

	//Set TrottlePoint in GCS
	waypoints[_TP].x= throttlePx;
	waypoints[_TP].y= throttlePy;

	return FALSE;
}

//bool_t NavSetThrottleWaypoint(uint8_t _wp)
//{
//  waypoints[_wp].x= throttlePx;
//  waypoints[_wp].y= throttlePy;

//  return FALSE;
//}

bool_t ZHAWBungeeTakeoff(uint8_t _TP) 
{
	//Translate current position so Initial point is (0,0)
	float Currentx = (waypoints[BungeeDirection].x) - estimator_x;
	float Currenty = (waypoints[BungeeDirection].y) - estimator_y;

	bool_t CurrentAboveLine;

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(0);				//Set the climb control to auto-throttle with the specified pitch pre-command (navigation.h) -> No Pitch
	  	NavVerticalAltitudeMode(TakeOff_Height, 0.);		//Vorgabe der Sollhöhe
		nav_route_xy(initialx,initialy,(waypoints[BungeeDirection].x),(waypoints[BungeeDirection].y));	//Vorgabe der Route

		kill_throttle = 1;	//Motor ausgeschaltet

		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)
		{
			initialx = estimator_x;
			initialy = estimator_y;

			//Translate the Current Position so that the Initial Position is (0/0)
			Currentx = (waypoints[BungeeDirection].x)-initialx;
			Currenty = (waypoints[BungeeDirection].y)-initialy; 

			//Find Launch line slope
			float MLaunch = Currenty/Currentx;

			//Compute Throttle Point
			if(Currentx > 0)
				throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
			else
				throttlePx = - TDistance/sqrt(MLaunch*MLaunch+1);

			if(Currenty > 0)
				throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
			else
				throttlePy = - sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
			
			//Find ThrottleLine
			ThrottleSlope = -1/MLaunch; 				//90° Drehung der ThrottleLine gegenüber LaunchLine
			ThrottleB = (throttlePy - (ThrottleSlope*throttlePx)); // Linie in der Form y=m*x+b

			
			//Determine whether the UAV is below or above the throttle line
			if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
				AboveLine = TRUE;
			else
				AboveLine = FALSE;

			//Translate the throttle point back
			throttlePx = throttlePx+initialx;
			throttlePy = throttlePy+initialy;

			//Set TrottlePoint in GCS
			waypoints[_TP].x= throttlePx;
			waypoints[_TP].y= throttlePy;
		}


		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx)+ThrottleB)
			CurrentAboveLine = TRUE;
		else
			CurrentAboveLine = FALSE;

		//Find out if UAV has crossed the line
		if(AboveLine != CurrentAboveLine && estimator_hspeed_mod > Takeoff_MinSpeed)
		{
			CTakeoffStatus = Throttle;
			kill_throttle = 0;
			nav_init_stage();		
		}
		break;
	case Throttle:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(AGR_CLIMB_PITCH); 	
		NavVerticalThrottleMode(9600*(1));		
		nav_route_xy(initialx,initialy,(waypoints[BungeeDirection].x),(waypoints[BungeeDirection].y));
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

bool_t ZHAWBungeeTakeoff_glide(uint8_t _TP)
{
	//Translate current position so Initial point is (0,0)
	float Currentx = (waypoints[BungeeDirection].x) - estimator_x;
	float Currenty = (waypoints[BungeeDirection].y) - estimator_y;

	bool_t CurrentAboveLine;

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(0);				//Set the climb control to auto-throttle with the specified pitch pre-command (navigation.h) -> No Pitch
	  	NavVerticalAltitudeMode(BungeeAlt+TakeOff_Height, 0.);	//Vorgabe der Sollhöhe
		nav_route_xy(initialx,initialy,(waypoints[BungeeDirection].x),(waypoints[BungeeDirection].y));	//Vorgabe der Route

		kill_throttle = 1;	//Motor ausgeschaltet

		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)
		{
			initialx = estimator_x;
			initialy = estimator_y;

			//Translate the Current Position so that the Initial Position is (0/0)
			Currentx = (waypoints[BungeeDirection].x)-initialx;
			Currenty = (waypoints[BungeeDirection].y)-initialy; 

			//Find Launch line slope
			float MLaunch = Currenty/Currentx;

			//Compute Throttle Point
			if(Currentx > 0)
				throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
			else
				throttlePx = - TDistance/sqrt(MLaunch*MLaunch+1);

			if(Currenty > 0)
				throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
			else
				throttlePy = - sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
			
			//Find ThrottleLine
			ThrottleSlope = -1/MLaunch; 				//90° Drehung der ThrottleLine gegenüber LaunchLine
			ThrottleB = (throttlePy - (ThrottleSlope*throttlePx)); // Linie in der Form y=m*x+b

			
			//Determine whether the UAV is below or above the throttle line
			if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
				AboveLine = TRUE;
			else
				AboveLine = FALSE;

			//Translate the throttle point back
			throttlePx = throttlePx+initialx;
			throttlePy = throttlePy+initialy;

			//Set TrottlePoint in GCS
			waypoints[_TP].x= throttlePx;
			waypoints[_TP].y= throttlePy;
		}


		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx)+ThrottleB)
			CurrentAboveLine = TRUE;
		else
			CurrentAboveLine = FALSE;

		//Find out if UAV has crossed the line
		if(AboveLine != CurrentAboveLine && estimator_hspeed_mod > Takeoff_MinSpeed)
		{
			CTakeoffStatus = Finished;
			kill_throttle = 0;
			//nav_init_stage();		//???????????
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


/************** ZHAW SkidLanding **********************************************/
/*
Landing Routine


  <section name="Landing" prefix="Landing_">
    <define name="AFHeight" value="50" unit="m"/>
    <define name="FinalHeight" value="5" unit="m"/>
    <define name="FinalStageTime" value="5" unit="s"/>
  </section>

 */

#ifndef Landing_AFHeight
#define Landing_AFHeight 50
#endif
#ifndef Landing_FinalHeight
#define Landing_FinalHeight 5
#endif
#ifndef Landing_FinalStageTime
#define Landing_FinalStageTime 5
#endif

enum LandingStatus { CircleDown, LandingWait, Final, Approach };
static enum LandingStatus CLandingStatus;
static uint8_t AFWaypoint;
static uint8_t TDWaypoint;
static float LandRadius;
static struct Point2D LandCircle;
static float LandAppAlt;
static float LandCircleQDR;
static float ApproachQDR;
static float FinalLandAltitude;
static uint8_t FinalLandCount;

bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, float radius) // Eins zu Eins nach OSAMNav
{
	AFWaypoint = AFWP;
	TDWaypoint = TDWP;
	CLandingStatus = CircleDown;
	LandRadius = radius;
	LandAppAlt = estimator_z;
	FinalLandAltitude = Landing_FinalHeight;
	FinalLandCount = 1;
	waypoints[AFWaypoint].a = waypoints[TDWaypoint].a + Landing_AFHeight;

	//Translate distance from AF to TD so that AF is (0/0) 
	float x_0 = waypoints[TDWaypoint].x - waypoints[AFWaypoint].x;
	float y_0 = waypoints[TDWaypoint].y - waypoints[AFWaypoint].y;

	/* Unit vector from AF to TD */
	float d = sqrt(x_0*x_0+y_0*y_0);	//d=Horizontale Strecke von AF zu TD
	float x_1 = x_0 / d;
	float y_1 = y_0 / d;

	//find the center of LandCircleQDR
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
		NavVerticalAutoThrottleMode(0); /* No pitch */
		
		if(NavCircleCount() < .1)
		{
	  		NavVerticalAltitudeMode(LandAppAlt, 0);  
  		}
		else
			NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);


		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius); 

		if(estimator_z < waypoints[AFWaypoint].a + 5)
		{
			CLandingStatus = LandingWait;
			nav_init_stage();
		}

	break;

	case LandingWait: // Einen eventuell angefangenen Kreis noch fertig fliegen 
		NavVerticalAutoThrottleMode(0); /* No pitch */
  		NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

	  	if(NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(ApproachQDR)))
		{
			CLandingStatus = Approach;
			nav_init_stage();
		}
	break;

//********************** bis hier wie OSAMNav *********************************************



	case Approach:
		kill_throttle = 1;
		NavVerticalAutoThrottleMode(0); /* No pitch */
  		NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0); 	//Sollhöhe geben
		nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);	

	  	if(NavQdrCloseTo(DegOfRad(LandCircleQDR)))
		{
			CLandingStatus = Final;
			nav_init_stage();
		}
	break;

	case Final: //Geradeausflug auf Landebahn zu bei jedem Durchgang wird die FinalLandAltitude (Höhe über dem TD Punkt) halbiert. So nähert sich die Drohne Asymptotisch
		kill_throttle = 1;
		NavVerticalAutoThrottleMode(0); /* No pitch */
  		NavVerticalAltitudeMode(waypoints[TDWaypoint].a+FinalLandAltitude, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);
		if(stage_time >= Landing_FinalStageTime*FinalLandCount)
		{
			FinalLandAltitude = FinalLandAltitude/2;
			FinalLandCount++;
		}
	break;

	default:

	break;
	}
	return TRUE;
}

