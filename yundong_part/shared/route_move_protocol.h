#ifndef ROUTE_MOVE_PROTOCOL_H
#define ROUTE_MOVE_PROTOCOL_H
#include "robot_distance_protocol.h"
#include <math.h>

/* Shared position moves for autonomous routes and individual chassis tests. */
typedef struct { uint32_t sequence,pulses,rpm,ppr; char direction; } RouteMove;
#define ROUTE_MOVE_ACCELERATION 10U
#define ROUTE_MOVE_MAX_RPM 1200U
#define ROUTE_MOVE_STATUS_MS 50U
#define ROUTE_MOVE_START_GUARD_MS 100U
#define ROUTE_MOVE_CONFIRM_SCANS 2U
#define ROUTE_MOVE_FINISH_GRACE_MS 5000U
/* Absolute lower bound at commanded peak RPM, without acceleration estimates.
 * Used only to reject a physically impossible early/stale reached indication. */
static inline uint32_t RouteMove_MinDuration(const RouteMove *c)
{
    if(!c->ppr||!c->rpm) return 0;
    uint64_t divisor=(uint64_t)c->ppr*c->rpm;
    return (uint32_t)(((uint64_t)c->pulses*60000U+divisor-1U)/divisor);
}
static inline uint32_t RouteMove_Duration(const RouteMove *c)
{
    if(!c->ppr||!c->rpm||!c->pulses) return 0;
    double rev=(double)c->pulses/c->ppr, rpm=c->rpm, acc=ROUTE_MOVE_ACCELERATION;
    double seconds=rev<=rpm*rpm/(60.0*acc)?
        2.0*sqrt(rev*60.0/acc):60.0*rev/rpm+rpm/acc;
    /* Keep the 150 s executor and 155 s bridge deadlines consistent. */
    double ms=ceil(seconds*1000.0)+100.0;
    return ms<=140000.0?(uint32_t)ms:0U;
}
static inline bool RouteMove_Parse(const char *frame,RouteMove *c)
{
    if(strncmp(frame,"ROUTE_MOVE,",11U)!=0) return false;
    const char *p=frame+11U;
    if(!RobotProtocol_U32(&p,&c->sequence,',')||!c->sequence||
       !*p||!strchr("FBLRCW",*p)||p[1]!=',') return false;
    c->direction=*p; p+=2;
    return RobotProtocol_U32(&p,&c->pulses,',')&&c->pulses>=1U&&c->pulses<=10000000U&&
        RobotProtocol_U32(&p,&c->rpm,',')&&c->rpm>=5U&&
        c->rpm<=(strchr("CW",c->direction)?120U:ROUTE_MOVE_MAX_RPM)&&
        RobotProtocol_U32(&p,&c->ppr,0)&&c->ppr>=200U&&c->ppr<=51200U&&RouteMove_Duration(c)>0U;
}
#endif
