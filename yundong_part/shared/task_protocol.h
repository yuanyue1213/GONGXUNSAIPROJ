#ifndef TASK_PROTOCOL_H
#define TASK_PROTOCOL_H
#define TASK_DEFAULT_CODE "156+123+516+231"
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
typedef enum { TASK_QR, TASK_MAT, TASK_RING, TASK_NONE, TASK_TABLE, TASK_OCCUPIED, TASK_PICK, TASK_NEAR } TaskVisionKind;
/* Three distinct task-code color IDs, e.g. 156 = red, black, light_blue. */
static inline bool TaskProtocol_TableColors(uint32_t target)
{
    unsigned a=target/100U,b=target/10U%10U,c=target%10U;
    return a>=1U&&a<=6U&&b>=1U&&b<=6U&&c>=1U&&c<=6U&&a!=b&&a!=c&&b!=c;
}
static inline bool TaskProtocol_Target(TaskVisionKind kind,uint32_t target)
{
    return (kind==TASK_TABLE||kind==TASK_OCCUPIED)?TaskProtocol_TableColors(target):
        target>=1U&&target<=((kind==TASK_MAT||kind==TASK_PICK||kind==TASK_NEAR)?6U:3U);
}
typedef struct {
    uint32_t request, target, position;
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
    if(strncmp(p,"PICK,",5U)==0||strncmp(p,"NEAR,",5U)==0) {
        v->kind=p[0]=='N'?TASK_NEAR:TASK_PICK; p+=5U;
        return RobotProtocol_U32(&p,&v->target,',')&&TaskProtocol_Target(TASK_PICK,v->target)&&
            RobotProtocol_U32(&p,&v->position,'\0')&&v->position>=1U&&v->position<=3U&&(v->kind!=TASK_NEAR||v->position==2U);
    }
    if(strncmp(p,"MAT,",4U)==0) v->kind=TASK_MAT;
    else if(strncmp(p,"RING,",5U)==0) v->kind=TASK_RING;
    else if(strncmp(p,"TABLE,",6U)==0) v->kind=TASK_TABLE;
    else if(strncmp(p,"OCCUPIED,",9U)==0) v->kind=TASK_OCCUPIED;
    else return false;
    p+=v->kind==TASK_OCCUPIED?9U:v->kind==TASK_TABLE?6U:v->kind==TASK_MAT?4U:5U;
    return RobotProtocol_U32(&p,&v->target,',') && TaskProtocol_Target(v->kind,v->target) &&
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
    if(strncmp(p,"PICK,",5U)==0||strncmp(p,"NEAR,",5U)==0) {
        v->kind=p[0]=='N'?TASK_NEAR:TASK_PICK; p+=5U;
        return RobotProtocol_U32(&p,&v->target,'\0')&&TaskProtocol_Target(TASK_PICK,v->target);
    }
    if(strncmp(p,"MAT,",4U)==0) v->kind=TASK_MAT;
    else if(strncmp(p,"RING,",5U)==0) v->kind=TASK_RING;
    else if(strncmp(p,"TABLE,",6U)==0) v->kind=TASK_TABLE;
    else if(strncmp(p,"OCCUPIED,",9U)==0) v->kind=TASK_OCCUPIED;
    else return false;
    p+=v->kind==TASK_OCCUPIED?9U:v->kind==TASK_TABLE?6U:v->kind==TASK_MAT?4U:5U;
    bool anchor=strchr(p,',')!=NULL;
    if(!RobotProtocol_U32(&p,&v->target,anchor?',':'\0')||!TaskProtocol_Target(v->kind,v->target)) return false;
    if(!anchor) return true;
    v->has_anchor=true;
    return (v->kind==TASK_MAT||v->kind==TASK_TABLE||v->kind==TASK_OCCUPIED)&&CameraProtocol_Pixel(&p,&v->anchor.x,',',512U)&&
        CameraProtocol_Pixel(&p,&v->anchor.y,'\0',320U);
}
#endif
