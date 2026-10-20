#ifndef TASK_PROTOCOL_H
#define TASK_PROTOCOL_H
#include "robot_distance_protocol.h"
#include "camera_position_protocol.h"

typedef struct { uint8_t colors[2][3], places[2][3]; } TaskCode;
static inline bool TaskProtocol_Code(const char *s, TaskCode *code)
{
    if (strlen(s) != 15U || s[3]!='+' || s[7]!='+' || s[11]!='+') return false;
    for (unsigned batch=0; batch<2U; ++batch) {
        unsigned colors=0, places=0;
        for (unsigned i=0; i<3U; ++i) {
            char c=s[batch*8U+i], p=s[batch*8U+4U+i];
            if (c<'1'||c>'6'||p<'1'||p>'3') return false;
            unsigned cb=1U<<(c-'1'), pb=1U<<(p-'1');
            if ((colors&cb)||(places&pb)) return false;
            colors|=cb; places|=pb;
            code->colors[batch][i]=(uint8_t)(c-'0');
            code->places[batch][i]=(uint8_t)(p-'0');
        }
        if (batch==1U) {
            unsigned first=0;
            for(unsigned i=0;i<3U;++i) first|=1U<<(code->colors[0][i]-1U);
            if(first!=colors) return false; /* Second batch stacks on the same colors. */
        }
    }
    return true;
}
typedef enum { TASK_QR, TASK_MAT, TASK_RING, TASK_NONE } TaskVisionKind;
typedef struct {
    uint32_t request, target;
    TaskVisionKind kind;
    CameraCenter center;
    TaskCode code;
    char text[16];
} TaskVision;
static inline bool TaskProtocol_Vision(const char *s, TaskVision *v)
{
    if(strncmp(s,"VISION,",7U)!=0) return false;
    const char *p=s+7U;
    if(!RobotProtocol_U32(&p,&v->request,',')||v->request==0U) return false;
    if(strncmp(p,"QR,",3U)==0) {
        v->kind=TASK_QR; p+=3U;
        if(!TaskProtocol_Code(p,&v->code)) return false;
        memcpy(v->text,p,16U); return true;
    }
    if(strcmp(p,"NONE")==0) { v->kind=TASK_NONE; return true; }
    if(strncmp(p,"MAT,",4U)==0) v->kind=TASK_MAT;
    else if(strncmp(p,"RING,",5U)==0) v->kind=TASK_RING;
    else return false;
    p+=v->kind==TASK_MAT?4U:5U;
    return RobotProtocol_U32(&p,&v->target,',') && v->target>=1U &&
        v->target<=(v->kind==TASK_MAT?6U:3U) &&
        CameraProtocol_Pixel(&p,&v->center.x,',',512U) &&
        CameraProtocol_Pixel(&p,&v->center.y,'\0',320U);
}
typedef struct { uint32_t request,target; TaskVisionKind kind; CameraCenter anchor; bool has_anchor; } TaskCameraRequest;
static inline bool TaskProtocol_Camera(const char *s, TaskCameraRequest *v)
{
    if(strncmp(s,"CAM,",4U)!=0) return false;
    const char *p=s+4U;
    v->has_anchor=false;
    if(!RobotProtocol_U32(&p,&v->request,',')||v->request==0U) return false;
    if(strcmp(p,"QR")==0) { v->kind=TASK_QR; v->target=0U; return true; }
    if(strcmp(p,"IDLE")==0) { v->kind=TASK_NONE; v->target=0U; return true; }
    if(strncmp(p,"MAT,",4U)==0) v->kind=TASK_MAT;
    else if(strncmp(p,"RING,",5U)==0) v->kind=TASK_RING;
    else return false;
    p+=v->kind==TASK_MAT?4U:5U;
    bool anchor=strchr(p,',')!=NULL;
    if(!RobotProtocol_U32(&p,&v->target,anchor?',':'\0')||v->target<1U||
        v->target>(v->kind==TASK_MAT?6U:3U)) return false;
    if(!anchor) return true;
    v->has_anchor=true;
    return v->kind==TASK_MAT&&CameraProtocol_Pixel(&p,&v->anchor.x,',',512U)&&
        CameraProtocol_Pixel(&p,&v->anchor.y,'\0',320U);
}
#endif
