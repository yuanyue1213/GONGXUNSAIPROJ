/* 执行前校验流程表；无状态，不参与步骤执行或游标维护。 */
#include "flow_types.h"
#include "task_protocol.h"

const char *Flow_Check(const FlowStep *steps,size_t count)
{
    if(!steps||count<2U||count>FLOW_MAX_STEPS) return "FLOW_LENGTH";
    if(steps[0].op!=FLOW_OP_PREPARE||steps[count-1U].op!=FLOW_OP_END) return "FLOW_BOUNDARY";
    bool reference=false,scanned=false;
    bool aligned=false;
    for(size_t i=0;i<count;++i) {
        const FlowStep *s=&steps[i];
        bool movement=s->op==FLOW_OP_MOVE||s->op==FLOW_OP_TURN;
        if(!movement&&(s->direction||s->amount)) return "FLOW_PARAMETERS";
        if(s->op!=FLOW_OP_ARM&&(s->arm||s->arm_count)) return "FLOW_PARAMETERS";
        if(s->op!=FLOW_OP_ALIGN&&s->vision) return "FLOW_PARAMETERS";
        switch(s->op) {
        case FLOW_OP_PREPARE:
            if(i) return "FLOW_PREPARE";
            break;
        case FLOW_OP_IMU_ZERO:
            reference=true;
            break;
        case FLOW_OP_STOW:
            break;
        case FLOW_OP_ARM:
            if(!s->arm||!s->arm_count||s->arm_count>FLOW_MAX_STEPS) return "FLOW_ARM_GROUP";
            break;
        case FLOW_OP_MOVE:
            if(aligned) return "FLOW_ALIGNMENT";
            if(s->direction!=FLOW_FORWARD&&s->direction!=FLOW_BACKWARD&&
               s->direction!=FLOW_LEFT&&s->direction!=FLOW_RIGHT) return "FLOW_DIRECTION";
            if(!s->amount||s->amount>10000U) return "FLOW_DISTANCE";
            break;
        case FLOW_OP_TURN:
            if(aligned) return "FLOW_ALIGNMENT";
            if(!reference) return "FLOW_REFERENCE";
            if(s->direction!=FLOW_LEFT&&s->direction!=FLOW_RIGHT) return "FLOW_DIRECTION";
            if(!s->amount||s->amount>360U) return "FLOW_ANGLE";
            break;
        case FLOW_OP_SCAN:
            scanned=true;
            break;
        case FLOW_OP_ALIGN:
            if(aligned) return "FLOW_ALIGNMENT";
            if(s->vision!=TASK_TABLE&&s->vision!=TASK_OCCUPIED&&s->vision!=TASK_RING) return "FLOW_VISION";
            if(s->vision!=TASK_RING&&!scanned) return "FLOW_TASK_CODE";
            aligned=true;
            break;
        case FLOW_OP_UNALIGN:
            if(!aligned) return "FLOW_ALIGNMENT";
            aligned=false;
            break;
        case FLOW_OP_PICK_RAW: case FLOW_OP_PROCESS_PUT: case FLOW_OP_PROCESS_GET:
        case FLOW_OP_STORE: case FLOW_OP_STACK:
            if(!scanned) return "FLOW_TASK_CODE";
            break;
        case FLOW_OP_END:
            if(aligned) return "FLOW_ALIGNMENT";
            if(i!=count-1U) return "FLOW_END";
            break;
        default:
            return "FLOW_OPCODE";
        }
    }
    return NULL;
}
