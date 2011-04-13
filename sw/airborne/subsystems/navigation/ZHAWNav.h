#ifndef ZHAWNav_H
#define ZHAWNav_H

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

//struct Point2D {float x; float y;};
//struct Line {float m;float b;float x;}; //wohl unnötig....

extern bool_t InitializeZHAWBungeeTakeoff(uint8_t BungeeWP);
extern bool_t ZHAWBungeeTakeoff(void);

extern bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, float radius);
extern bool_t ZHAWSkidLanding(void);


#endif
