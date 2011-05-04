#ifndef ZHAWNav_H_Land
#define ZHAWNav_H_Land

#include "std.h"
#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"


extern bool_t InitializeZHAWSkidLanding(uint8_t AFWP, uint8_t TDWP, uint8_t FPWP, float radius);
extern bool_t ZHAWSkidLanding(void);
bool_t CalculateCheckPoint(void);
bool_t UAVcrossedCheckPoint (void);


#endif
