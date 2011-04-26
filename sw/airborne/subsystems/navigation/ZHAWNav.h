#ifndef ZHAWNav_H
#define ZHAWNav_H

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

//struct Point2D {float x; float y;};
//struct Line {float m;float b;float x;}; //wohl unnötig....

bool_t calculateTakeOffConditions(void);
extern bool_t InitializeZHAWBungeeTakeoff(uint8_t BungeeWP, uint8_t _TP);
extern bool_t ZHAWBungeeTakeoff(uint8_t _TP);

extern bool_t ZHAWBungeeTakeoff_glide(uint8_t _TP);

extern bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, uint8_t FPWP, float radius);
extern bool_t ZHAWSkidLanding(void);
bool_t CalculateLandingCondition(void);
bool_t UAVcrossedFlareLine (void);


//extern bool_t NavSetThrottleWaypoint(uint8_t _wp);


#endif
