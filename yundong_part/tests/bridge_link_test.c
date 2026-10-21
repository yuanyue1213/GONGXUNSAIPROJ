#include <assert.h>
#include <stdio.h>
#include "../shared/bridge_link_protocol.h"
int main(void)
{
    uint32_t id;
    assert(BridgeLink_Query("PING,4294967295",&id)==BRIDGE_PING&&id==UINT32_MAX);
    assert(BridgeLink_Query("QUERY,22",&id)==BRIDGE_QUERY&&id==22);
    assert(BridgeLink_Query("QUERY_IMU,23",&id)==BRIDGE_IMU&&id==23);
    const char *invalid[]={"QUERY,0","QUERY,1,junk","PING,-1","PING,4294967296","PING,","QUERY_IMU, 1","QUERY_IMU,1\n"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) assert(BridgeLink_Query(invalid[i],&id)==BRIDGE_NONE);
    assert(BridgeLink_ManualExec("EXEC,1,MOVE,L,50,20,9889"));
    assert(BridgeLink_ManualExec("EXEC,2,MOVE,W,10,20,9889"));
    assert(BridgeLink_ManualExec("EXEC,3,DRIVE,B,989,20,3200"));
    assert(BridgeLink_ManualExec("EXEC,4,TURN,W,90.0,20"));
    assert(BridgeLink_ManualExec("EXEC,5,IMU_ZERO"));
    assert(BridgeLink_ManualExec("EXEC,6,ROUTE_MOVE,F,3200,300,3200"));
    assert(BridgeLink_ManualExec("EXEC,7,ROUTE_MOVE,R,3200,1200,3200"));
    assert(!BridgeLink_ManualExec("EXEC,7,ROUTE_MOVE,R,3200,1201,3200"));
    assert(BridgeLink_ManualExec("EXEC,7,ROUTE_MOVE,C,3200,60,3200"));
    assert(BridgeLink_ManualExec("EXEC,7,TURN,C,90.0,120"));
    assert(!BridgeLink_ManualExec("EXEC,7,TURN,C,90.0,121"));
    assert(!BridgeLink_ManualExec("EXEC,7,ROUTE_MOVE,C,3200,121,3200"));
    assert(!BridgeLink_ManualExec("EXEC,7,ROUTE_MOVE,R,3200,60,3200,junk"));
    assert(!BridgeLink_ManualExec("EXEC,0,MOVE,F,50,20,9889"));
    assert(!BridgeLink_ManualExec("EXEC,1,MOVE,X,50,20,9889"));
    assert(!BridgeLink_ManualExec("EXEC,1,MOVE,F,50,301,9889"));
    assert(!BridgeLink_ManualExec("EXEC,1,MOVE,F,50,20,9889,junk"));
    assert(BridgeLink_Feedback("PONG,3,STM32"));
    assert(BridgeLink_Feedback("STATUS,4,IDLE,0,0"));
    assert(BridgeLink_Feedback("ACTION,5,DONE,0"));
    assert(BridgeLink_Feedback("IMU,6,NOT_READY,INIT_SEARCH"));
    assert(BridgeLink_Feedback("MOTOR_DIAG,CHASSIS,3,POSITION_READ"));
    assert(BridgeLink_Feedback("MOTOR_DIAG,ARM,1,ORIGIN_READ"));
    assert(!BridgeLink_Feedback("CMD,0,S"));
    puts("PASS: manual QUERY/PING/IMU forwarding, EXEC wheel and gyro validation, feedback whitelist");
}
