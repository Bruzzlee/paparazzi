#include "subsystems/navigation/ZHAWNav.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/sonar/sonar_adc.h"

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
#define Takeoff_Speed 15
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


bool_t InitializeZHAWBungeeTakeoff(uint8_t TODWP, uint8_t _TP)
{
	TOD = TODWP;
	TP = _TP;
	TakeOff_Height = (waypoints[TOD].a);

	Takeoff_MinSpeed_local=3.0;   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test


	//Takeoff_Distance can only be positive
	TDistance = 15.0; 		//fabs(Takeoff_Distance);!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test

	//Record bungee alt (which should be the ground alt at that point)
	BungeeAlt = (waypoints[_TP].a);


	AboveThrottleLine=calculateTakeOffConditions();				//Auf welcher Seite ist dir Drohne


	//Enable Launch Status and turn kill throttle on
	CTakeoffStatus = Launch;
	kill_throttle = 1; //MOTOR AUS

	return FALSE;
}


bool_t ZHAWBungeeTakeoff(uint8_t _TP) 
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
		nav_route_xy(initialx,initialy,(waypoints[TOD].x),(waypoints[TOD].y));	//Vorgabe der Route
		kill_throttle = 1;	//Motor ausgeschaltet


		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)
			AboveThrottleLine=calculateTakeOffConditions();	
		


		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx) + 0)
			CurrentAboveThrottleLine = TRUE;
		else
			CurrentAboveThrottleLine = FALSE;
				

		float stimmtSo = estimator_hspeed_mod; 		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! für den Test
		RunOnceEvery(10, DOWNLINK_SEND_ZHAWTAKEOFF(DefaultChannel, &AboveThrottleLine, &CurrentAboveThrottleLine, &stimmtSo, &Takeoff_MinSpeed_local, &deltaTX, &deltaTY, &Currentx, &Currenty, &ThrottleB, &ThrottleSlope, &throttlePx, &throttlePy));


		//Find out if UAV has crossed the line
		if(AboveThrottleLine != CurrentAboveThrottleLine && estimator_hspeed_mod > Takeoff_MinSpeed_local)
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
		nav_route_xy(initialx,initialy,(waypoints[TOD].x),(waypoints[TOD].y));
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

/*bool_t ZHAWBungeeTakeoff_glide(uint8_t _TP)
{
	//Translate the Current Position so that the THROTTLEPOINT is (0/0) (wie weit ist die Drohne noch vom TP weg?)
	float Currentx = estimator_x - throttlePx;
	float Currenty = estimator_y - throttlePy;

	bool_t CurrentAboveLine;

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		NavVerticalAutoThrottleMode(0);				//Set the climb control to auto-throttle with the specified pitch pre-command (navigation.h) -> No Pitch
	  	NavVerticalAltitudeMode(TakeOff_Height, 0.);		//Vorgabe der Sollhöhe
		nav_route_xy(initialx,initialy,(waypoints[TOD].x),(waypoints[TOD].y));	//Vorgabe der Route

		kill_throttle = 1;	//Motor ausgeschaltet

		//recalculate lines if the UAV is not in Auto2
		if(pprz_mode < 2)	// nie neu berechnen
		{
			AboveLine=calculateTakeOffConditions();	
		}


		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx) + 0)
			CurrentAboveLine = TRUE;
		else
			CurrentAboveLine = FALSE;



		float stimmtSo = estimator_hspeed_mod;
		RunOnceEvery(20, DOWNLINK_SEND_ZHAWTAKEOFF(DefaultChannel, &AboveLine, &CurrentAboveLine, &stimmtSo, &Takeoff_MinSpeed_local, &deltaTX, &deltaTY, &Currentx, &Currenty, &ThrottleB, &ThrottleSlope, &throttlePx, &throttlePy));



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
}*/


/************** ZHAW SkidLanding **********************************************/
/*
Landing Routine


  <section name="Landing" prefix="Landing_">
    <define name="AFHeight" value="50" unit="m"/>
    <define name="FinalHeight" value="5" unit="m"/>
    <define name="FinalStageTime" value="5" unit="s"/>
  </section>

 */

#ifndef Landing_FinalHeight
#define Landing_FinalHeight 5
#endif

#ifndef Landing_FinalStageTime
#define Landing_FinalStageTime 5
#endif

#ifndef KillThrottleHeight
#define KillThrottleHeight 2
#endif

#ifndef SonarHeight		//Höhe auf welche 
#define SonarHeight 6
#endif

#ifndef saveHeight		//Höhe bei der in Failsave gegangen werden soll
#define saveHeight 4
#endif


/*
enum LandingStatus { CircleDown, LandingWait, Final, Approach };
static enum LandingStatus CLandingStatus;
static uint8_t AFWaypoint;
static uint8_t TDWaypoint;
static uint8_t FPWaypoint;
static float LandRadius;
static struct Point2D LandCircle;
static float LandAppAlt;
static float LandCircleQDR;
static float ApproachQDR;
static float FinalLandAltitude;
static uint8_t FinalLandCount;
static bool_t AboveFlareLine;

static float FlarePx;
static float FlarePy;
static float FlareSlope;
static float DeltaFX
static float DeltaFY


bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, uint8_t FPWP, float radius) // Eins zu Eins nach OSAMNav
{
	AFWaypoint = AFWP;
	TDWaypoint = TDWP;
	FPWaypoint = FPWP;
	
	CLandingStatus = CircleDown;
	LandRadius = radius;
	LandAppAlt = estimator_z;
	FinalLandAltitude = Landing_FinalHeight;
	FinalLandCount = 1;

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

	AboveFlareLine = CalculateLandingCondition();
	

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
		}
	break;


	case DeclineToSonar:
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(waypoints[TDWaypoint].a+SonarHeight, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);

		if(estimator_z_sonar < (SonarHeight + 0.5))
		{
			CLandingStatus = Approach;
			estimator_z_mode = sonar;
			nav_init_stage();
		}
	break	

	case Approach: //Sonar Höhe (ca. 6m) halten, bis zum FlarePoint, wo die Landung begonnen werden kann. 
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(SonarHeight, 0); //Sonarhöhe ÜBER BODEN!!!!
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);
		
		//find ouf if the UAV has crossed the FlareLine
		if (UAVcrossedFlareLine() == true)
		{
			CLandingStatus = Flare;
			nav_init_stage();
		}


	case Flare:
		NavVerticalAutoThrottleMode(0); // No pitch
  		NavVerticalAltitudeMode(LandAltitudeOffset, 0);
		nav_route_xy(waypoints[AFWaypoint].x,waypoints[AFWaypoint].y,waypoints[TDWaypoint].x,waypoints[TDWaypoint].y);

		if(stage_time >= Landing_FinalStageTime*FinalLandCount)
		{
			FinalLandAltitude = FinalLandAltitude/2;
			FinalLandCount++;
		}

		if(estimator_z_sonar < KillThrottleHeight)
		{
			kill_throttle = 1;
		}
	break;

	default:

	break;
	}
	//Failsave
	if ((estimator_z_sonar < saveHeight) && (CLandingStatus != Flare) && (CLandingStatus != KillThrottle))
	{
		//Mach den Failsave und starte durch!!
		//Möglicherweise muss nur false zurückgegeben werden und im Flightplan muss der Folgepunkt STBY sein!!
		estimator_z_mode=gps;
		return false;
	}


	return TRUE;
}

bool_t CalculateLandingCondition( void ) //TD ist (0/0)
{

	//Compute deltaFX and deltaFY between AF an TD with TD=(0/0)
	float deltaFX = (waypoints[AFWaypoint].x); - (waypoints[TDWaypoint].x);
	float deltaFY = (waypoints[AFWaypoint].y); - (waypoints[TDWaypoint].y);

	//Find Land line slope and Throttle line slope
	float MLaunch = deltaFY/deltaFX; 

	//Compute Flare Point
	if(deltaFX < 0)
		FlarePx = FlareDistance/sqrt(MLaunch*MLaunch+1);
	else
		FlarePx = - FlareDistance/sqrt(MLaunch*MLaunch+1);

	if(deltaFY < 0)
		FlarePy = sqrt((FlareDistance*FlareDistance)-(FlarePx*FlarePx));
	else
		FlarePy = - sqrt((FlareDistance*FlareDistance)-(FlarePx*FlarePx));

	//Find FlareLine
	FlareSlope = tan(atan2(deltaFY,deltaFX)+(3.14/2));			//-1/MLaunch; //90° Drehung der Kurve
	float FlareB = (FlarePy - (ThrottleSlope*FlarePx));  				//y-Offset

	//Translate ThrottlePoint to absolut
	FlarePx= FlarePx + (waypoints[TDWaypoint].x);
	FlarePY= FlarePy + (waypoints[TDWaypoint].y);

	//Set TrottlePoint in GCS
	waypoints[FPWaypoint].x= FlarePx;
	waypoints[FPWaypoint].y= FlarePy;

	//Determine whether the UAV is below or above the Flare line
	if(deltaFY > ((FlareSlope*deltaFX)+FlareB)) 	
		return TRUE;
	else
		return FALSE;

}

bool_t UAVcrossedFlareLine ( void )
{
	//Translate the Current Position so that the FLAREPOINT is (0/0) (wie weit ist die Drohne noch vom FP weg?)
	float Currentx = estimator_x - FlarePx;
	float Currenty = estimator_y - FlarePy;

	bool_t CurrentAboveFlareLine;

	//Find out if the UAV is currently above the line
	if(Currenty > (FlareSlope*Currentx) + 0)
		CurrentAboveFlareLine = TRUE;
	else
		CurrentAboveFlareLine = FALSE;

	if(AboveFlareLine != CurrentAboveFlareLine)
	{
		return true;		
	}

	return false;

}
*/
