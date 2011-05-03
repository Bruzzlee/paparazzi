#ifndef ZHAWNav_H
#define ZHAWNav_H

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"


bool_t calculateTakeOffConditions(void);
extern bool_t InitializeZHAWBungeeTakeoff(uint8_t BungeeWP, uint8_t _TP);
extern bool_t ZHAWBungeeTakeoff(void);

extern bool_t ZHAWBungeeTakeoff_glide(void);



#endif
