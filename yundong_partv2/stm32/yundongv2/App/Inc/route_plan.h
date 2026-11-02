#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { NAV_QR=1,NAV_RAW_1,NAV_COARSE_1,NAV_TEMP_1,
    NAV_RAW_2,NAV_COARSE_2,NAV_STACK,NAV_RETURN } NavLeg;
void RoutePlan_Init(void);
bool RoutePlan_Command(const char *frame);
bool RoutePlan_CanStart(void);
bool RoutePlan_Prepare(uint32_t run);
bool RoutePlan_Run(unsigned leg);
#endif
