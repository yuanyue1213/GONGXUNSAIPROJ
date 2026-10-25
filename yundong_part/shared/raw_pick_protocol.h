#ifndef RAW_PICK_PROTOCOL_H
#define RAW_PICK_PROTOCOL_H
#include "robot_distance_protocol.h"
#include <stdio.h>
/* Prepare the normal grab through external positioning, then wait for vision. */
static inline bool RawPick_Ready(const char *frame, RobotSequenceCommand *settings)
{
    if(strncmp(frame,"RAW_READY,",10U)!=0) return false;
    char converted[193U];
    int n=snprintf(converted,sizeof(converted),"STATE_V2,%s",frame+10U);
    return n>0&&(size_t)n<sizeof(converted)&&RobotProtocol_ParseSequence(converted,settings)&&settings->mode=='A';
}
static inline bool RawPick_Take(const char *frame)
{
    if(strncmp(frame,"RAW_TAKE,",9U)!=0) return false;
    const char *p=frame+9U; uint32_t id;
    return RobotProtocol_U32(&p,&id,'\0')&&id!=0U;
}
#endif
