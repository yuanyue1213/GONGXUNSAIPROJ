#ifndef GYRO_TURN_PROTOCOL_H
#define GYRO_TURN_PROTOCOL_H
#include "robot_distance_protocol.h"
#include <stdio.h>
typedef enum { GYRO_REFERENCE, GYRO_RELATIVE, GYRO_HEADING, GYRO_DRIVE } GyroCommandKind;
typedef struct { uint32_t sequence, rpm; float degrees; char direction; GyroCommandKind kind;
    uint32_t pulses, pulses_per_revolution;
} GyroCommand;
static inline bool GyroProtocol_Angle(const char **cursor,float *angle,char delimiter)
{
    const char *p=*cursor; uint32_t n=0;
    if(*p<'0'||*p>'9') return false;
    do { n=n*10U+(uint32_t)(*p++-'0'); if(n>360U) return false; } while(*p>='0'&&*p<='9');
    unsigned fraction=0;
    if(*p=='.') { ++p; if(*p<'0'||*p>'9') return false; fraction=(unsigned)(*p++-'0'); }
    if(*p!=delimiter||n*10U+fraction>3600U) return false;
    *angle=(float)n+fraction*0.1f; *cursor=p+(delimiter!=0); return true;
}
static inline bool GyroProtocol_Parse(const char *frame,GyroCommand *c)
{
    const char *p;
    if(strncmp(frame,"DRIVE,",6U)==0) {
        p=frame+6U; c->kind=GYRO_DRIVE; c->degrees=0;
        if(!RobotProtocol_U32(&p,&c->sequence,',')||!c->sequence) return false;
        c->direction=*p;
        if((*p!='F'&&*p!='B')||p[1]!=',') return false;
        p+=2;
        return RobotProtocol_U32(&p,&c->pulses,',')&&c->pulses>=1U&&c->pulses<=10000000U&&
            RobotProtocol_U32(&p,&c->rpm,',')&&c->rpm>=5U&&c->rpm<=300U&&
            RobotProtocol_U32(&p,&c->pulses_per_revolution,0)&&
            c->pulses_per_revolution>=200U&&c->pulses_per_revolution<=51200U;
    }
    if(strncmp(frame,"IMU_ZERO,",9U)==0) {
        p=frame+9U; c->kind=GYRO_REFERENCE; c->degrees=0; c->rpm=0; c->direction=0;
        return RobotProtocol_U32(&p,&c->sequence,0)&&c->sequence!=0U;
    }
    if(strncmp(frame,"TURN,",5U)==0) { p=frame+5U; c->kind=GYRO_RELATIVE; }
    else if(strncmp(frame,"HEADING,",8U)==0) { p=frame+8U; c->kind=GYRO_HEADING; }
    else return false;
    if(!RobotProtocol_U32(&p,&c->sequence,',')||c->sequence==0U) return false;
    c->direction=0;
    if(c->kind==GYRO_RELATIVE) {
        c->direction=*p;
        if((*p!='C'&&*p!='W')||p[1]!=',') return false;
        p+=2;
    }
    return GyroProtocol_Angle(&p,&c->degrees,',')&&RobotProtocol_U32(&p,&c->rpm,0)&&
        c->rpm>=5U&&c->rpm<=120U&&(c->kind==GYRO_RELATIVE?c->degrees>=0.1f:c->degrees<360.0f);
}
static inline bool GyroProtocol_ParseExec(const char *frame,GyroCommand *c)
{
    if(strncmp(frame,"EXEC,",5U)!=0) return false;
    const char *p=frame+5U; uint32_t id;
    if(!RobotProtocol_U32(&p,&id,',')||!id) return false;
    const char *comma=strchr(p,','); char command[96]; int n;
    if(comma) {
        if(comma-p>16) return false;
        n=snprintf(command,sizeof(command),"%.*s,%lu,%s",(int)(comma-p),p,(unsigned long)id,comma+1);
    } else {
        if(strcmp(p,"IMU_ZERO")!=0) return false;
        n=snprintf(command,sizeof(command),"IMU_ZERO,%lu",(unsigned long)id);
    }
    return n>=0&&(size_t)n<sizeof(command)&&GyroProtocol_Parse(command,c);
}
#endif
