#ifndef ZHAWNav2_H
#define ZHAWNav2_H

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"


bool_t calculateTakeOffConditionsNavLine(void);
extern bool_t InitializeZHAWBungeeTakeoffNavLine(uint8_t BungeeWP, uint8_t _TP, uint8_t _NP);
extern bool_t ZHAWBungeeTakeoffNavLine(void);


#endif
