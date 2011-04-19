#ifndef ZHAWNav_H
#define ZHAWNav_H

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

//struct Point2D {float x; float y;};
//struct Line {float m;float b;float x;}; //wohl unnötig....

extern bool_t InitializeZHAWBungeeTakeoff(uint8_t BungeeWP, uint8_t _TP);
extern bool_t ZHAWBungeeTakeoff(uint8_t _TP);
extern bool_t ZHAWBungeeTakeoff_glide(uint8_t _TP);

extern bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, float radius);
extern bool_t ZHAWSkidLanding(void);

extern bool_t NavSetThrottleWaypoint(uint8_t _wp);


#endif
