#ifndef BRIDGE_LINK_PROTOCOL_H
#define BRIDGE_LINK_PROTOCOL_H
#include "gyro_turn_protocol.h"
#include "route_move_protocol.h"

typedef enum { BRIDGE_NONE, BRIDGE_PING, BRIDGE_QUERY, BRIDGE_IMU } BridgeQuery;
static inline BridgeQuery BridgeLink_Query(const char *line,uint32_t *id)
{
    const char *p; BridgeQuery kind;
    if(strncmp(line,"PING,",5U)==0) { p=line+5; kind=BRIDGE_PING; }
    else if(strncmp(line,"QUERY,",6U)==0) { p=line+6; kind=BRIDGE_QUERY; }
    else if(strncmp(line,"QUERY_IMU,",10U)==0) { p=line+10; kind=BRIDGE_IMU; }
    else return BRIDGE_NONE;
    return RobotProtocol_U32(&p,id,0)&&*id?kind:BRIDGE_NONE;
}
static inline bool BridgeLink_Feedback(const char *line)
{
    return strncmp(line,"ACTION,",7U)==0||strncmp(line,"STATUS,",7U)==0||
        strncmp(line,"IMU,",4U)==0||strncmp(line,"PONG,",5U)==0||
        strncmp(line,"MOTOR_DIAG,",11U)==0;
}
/* The manual App can use EXEC for both gyro motion and legacy wheel-distance
 * moves. Do not silently discard the latter at the ESP32 bridge. */
static inline bool BridgeLink_ManualExec(const char *line)
{
    GyroCommand gyro;
    if(GyroProtocol_ParseExec(line,&gyro)) return true;
    if(strncmp(line,"EXEC,",5U)!=0) return false;
    const char *p=line+5; uint32_t id;
    if(!RobotProtocol_U32(&p,&id,',')||!id) return false;
    if(strncmp(p,"ROUTE_MOVE,",11U)==0) {
        char frame[96]; RouteMove c;
        int n=snprintf(frame,sizeof(frame),"ROUTE_MOVE,%lu,%s",(unsigned long)id,p+11U);
        return n>0&&(size_t)n<sizeof(frame)&&RouteMove_Parse(frame,&c);
    }
    if(strncmp(p,"MOVE,",5U)!=0) return false;
    char move[96]; RobotDistanceCommand c;
    int n=snprintf(move,sizeof(move),"MOVE,%lu,%s",(unsigned long)id,p+5);
    return n>0&&(size_t)n<sizeof(move)&&RobotProtocol_ParseMove(move,&c)&&RobotProtocol_MovePulses(&c)>0;
}
#endif
