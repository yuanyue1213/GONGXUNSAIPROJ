#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../esp32s3/hello_world/main/autonomous_task.h"
static char event_line[160],outgoing[192];
static void send_line(const char *s) { snprintf(outgoing,sizeof(outgoing),"%s",s); }
static void event(const char *s) { snprintf(event_line,sizeof(event_line),"%s",s); }
static void config(void) {
    AutoTask_Init(send_line,event);
    AutoTask_Command("AUTO_CFG,10,10000,20000,1200,10,256.0,160.0,321,321,0,90,180",0);
    assert(strstr(event_line,",CONFIGURED,")); AutoTask_Command("AUTO_DRIVE,10,1000,40,3200",0);
    AutoTask_Command("AUTO_OPTIONS,10,0,0,-",0);
}
static void start(void) { AutoTask_Command("AUTO_START,20,10",0); }
int main(void) {
    config(); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_PATH,10,BODY_NEAR_V3",0); start(); assert(AutoTask_Running()); AutoTask_Stop("TEST");
    config(); AutoTask_Command("AUTO_PATH,9,BODY_NEAR_V3",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_PATH,10,BODY_LMR_V2",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_PATH,10,BODY_V1",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_PATH,10,BODY_V2",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_ROUTE,10,0,2250,2200",0); AutoTask_Command("AUTO_PATH,10,BODY_NEAR_V3",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_PATH,10,BODY_NEAR_V3",0); AutoTask_Command("AUTO_DRIVE,10,1000,121,3200",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_PATH,10,BODY_NEAR_V3",0); AutoTask_Command("AUTO_OPTIONS,10,0,1,-",0); start(); assert(!AutoTask_Running());
    config(); AutoTask_Command("AUTO_CFG,11,10000,20000,1201,10,256.0,160.0,321,321,0,90,180",0); assert(strstr(event_line,",ERROR,"));
    config(); AutoTask_Command("AUTO_PATH,10,BODY_NEAR_V3",0); start(); assert(AutoTask_Running()); unsigned id;
    assert(sscanf(outgoing,"QUERY,%u",&id)==1); char reply[96]; snprintf(reply,sizeof(reply),"STATUS,%u,IDLE,0,0",id); AutoTask_Input(reply,0);
    AutoTask_Tick(500); assert(sscanf(outgoing,"EXEC,%u,IMU_ZERO",&id)==1);
    snprintf(reply,sizeof(reply),"ACTION,%u,ERROR,IMU_NOT_READY",id); AutoTask_Input(reply,501); assert(!AutoTask_Running()&&strstr(event_line,"IMU_NOT_READY"));
    config(); AutoTask_Command("AUTO_OPTIONS,10,0,1,156+123+516+231",0);
    assert(strstr(event_line,",OPTIONS,"));
    config(); AutoTask_Command("AUTO_PATH,10,BODY_NEAR_DEFAULT_V1",0); start();
    assert(!AutoTask_Running()&&strstr(event_line,"DEFAULT_TASK_CODE_REQUIRED"));
    config(); AutoTask_Command("AUTO_OPTIONS,10,1,0,156+123+516+231",0);
    AutoTask_Command("AUTO_PATH,10,BODY_NEAR_DEFAULT_V1",0); start();
    assert(AutoTask_Running()); AutoTask_Stop("TEST");
    config(); AutoTask_Command("AUTO_OPTIONS,10,1,0,123+123+321+231",0);
    AutoTask_Command("AUTO_PATH,10,BODY_NEAR_DEFAULT_V1",0); start();
    assert(!AutoTask_Running()&&strstr(event_line,"DEFAULT_TASK_CODE_REQUIRED"));
    puts("PASS: BODY_NEAR_V3 required, stale path/config rejected, speed bounds, origin-free chassis, error propagation");
}
