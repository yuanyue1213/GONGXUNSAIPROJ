#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_flow.h"

static void test_validation(void)
{
    assert(!Flow_Check(mission_flow,FLOW_COUNT(mission_flow)));
    unsigned moves=0,stations=0,scans=0;
    for(size_t i=0;i<FLOW_COUNT(mission_flow);++i) {
        const FlowStep *s=&mission_flow[i];
        if(s->op==FLOW_OP_MOVE||s->op==FLOW_OP_TURN) ++moves;
        if(s->op>=FLOW_OP_PICK_RAW&&s->op<=FLOW_OP_STACK) ++stations;
        if(s->op==FLOW_OP_SCAN) ++scans;
    }
    assert(moves==25&&stations==8&&scans==1);
    const FlowStep simple[]={FLOW_PREPARE(),FLOW_IMU_ZERO(),FLOW_MOVE(FLOW_RIGHT,250),FLOW_END()};
    assert(!Flow_Check(simple,FLOW_COUNT(simple)));
    const FlowStep alignment[]={FLOW_PREPARE(),FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),
        FLOW_ARM(arm_ready),FLOW_STOW(),FLOW_UNALIGN(),FLOW_END()};
    assert(!Flow_Check(alignment,FLOW_COUNT(alignment)));
    const FlowStep no_undo[]={FLOW_PREPARE(),FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),FLOW_END()};
    assert(!strcmp(Flow_Check(no_undo,FLOW_COUNT(no_undo)),"FLOW_ALIGNMENT"));
    const FlowStep move_aligned[]={FLOW_PREPARE(),FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),
        FLOW_MOVE(FLOW_LEFT,100),FLOW_UNALIGN(),FLOW_END()};
    assert(!strcmp(Flow_Check(move_aligned,FLOW_COUNT(move_aligned)),"FLOW_ALIGNMENT"));
    const FlowStep undo_alone[]={FLOW_PREPARE(),FLOW_UNALIGN(),FLOW_END()};
    assert(!strcmp(Flow_Check(undo_alone,FLOW_COUNT(undo_alone)),"FLOW_ALIGNMENT"));
    FlowStep custom[]={FLOW_PREPARE(),FLOW_IMU_ZERO(),FLOW_ARM(arm_ready),FLOW_END()};
    assert(!Flow_Check(custom,FLOW_COUNT(custom)));
    custom[2].arm=NULL;
    assert(!strcmp(Flow_Check(custom,FLOW_COUNT(custom)),"FLOW_ARM_GROUP"));
    custom[2].arm=arm_ready; custom[2].arm_count=0;
    assert(Flow_Check(custom,FLOW_COUNT(custom)));
    custom[2].arm_count=FLOW_MAX_STEPS+1U;
    assert(Flow_Check(custom,FLOW_COUNT(custom)));
    custom[2].arm_count=ARM_COUNT(arm_ready); custom[2].op=FLOW_OP_STOW;
    assert(!strcmp(Flow_Check(custom,FLOW_COUNT(custom)),"FLOW_PARAMETERS"));
    const FlowStep no_reference[]={FLOW_PREPARE(),FLOW_ARM(arm_ready),FLOW_END()};
    assert(!Flow_Check(no_reference,FLOW_COUNT(no_reference)));
    const FlowStep minimal[]={FLOW_PREPARE(),FLOW_END()};
    assert(!Flow_Check(minimal,FLOW_COUNT(minimal)));
    const FlowStep independent[]={FLOW_PREPARE(),FLOW_STOW(),FLOW_MOVE(FLOW_RIGHT,250),
        FLOW_SCAN_TASK(),FLOW_STORE(FLOW_BATCH_1),FLOW_END()};
    assert(!Flow_Check(independent,FLOW_COUNT(independent)));
    const FlowStep turn_no_reference[]={FLOW_PREPARE(),FLOW_TURN(FLOW_LEFT,90),FLOW_END()};
    assert(!strcmp(Flow_Check(turn_no_reference,FLOW_COUNT(turn_no_reference)),"FLOW_REFERENCE"));

    FlowStep edited[FLOW_COUNT(mission_flow)];
    for(unsigned variation=0;variation<8;++variation) {
        memcpy(edited,mission_flow,sizeof(edited));
        switch(variation) {
        case 0: edited[0].op=FLOW_OP_STOW; break;
        case 1: edited[3].amount=0; break;
        case 2: edited[7].direction=FLOW_FORWARD; break;
        case 3: edited[9].direction=FLOW_FORWARD; break;
        case 4: edited[5].op=FLOW_OP_STOW; break; /* 站点前未扫码。 */
        case 5: edited[FLOW_COUNT(edited)-1].op=FLOW_OP_STOW; break;
        case 6: edited[3].op=(FlowOp)99; break;
        case 7: edited[3].amount=UINT32_MAX; break;
        }
        assert(Flow_Check(edited,FLOW_COUNT(edited)));
    }
    assert(Flow_Check(NULL,0));
    assert(Flow_Check(mission_flow,FLOW_MAX_STEPS+1U));
    const FlowStep unfinished[]={FLOW_PREPARE(),FLOW_IMU_ZERO(),FLOW_SCAN_TASK(),
        FLOW_PICK_RAW(FLOW_BATCH_1),FLOW_END()};
    assert(!Flow_Check(unfinished,FLOW_COUNT(unfinished)));
    const FlowStep free_order[]={FLOW_PREPARE(),FLOW_IMU_ZERO(),FLOW_SCAN_TASK(),
        FLOW_STACK(FLOW_BATCH_1),FLOW_STORE(FLOW_BATCH_2),
        FLOW_PROCESS_GET(FLOW_BATCH_2),FLOW_PROCESS_GET(FLOW_BATCH_2),
        FLOW_SCAN_TASK(),FLOW_PICK_RAW(FLOW_BATCH_1),FLOW_IMU_ZERO(),FLOW_END()};
    assert(!Flow_Check(free_order,FLOW_COUNT(free_order)));
    FlowStep unchecked[]={FLOW_PREPARE(),FLOW_IMU_ZERO(),FLOW_SCAN_TASK(),
        FLOW_STORE((FlowBatch)3),FLOW_END()};
    assert(!Flow_Check(unchecked,FLOW_COUNT(unchecked)));
    unchecked[3].batch=(FlowBatch)0;
    unchecked[1].batch=(FlowBatch)99;
    assert(!Flow_Check(unchecked,FLOW_COUNT(unchecked))); /* 校验器不读取 batch 字段。 */
    const FlowStep missing_code[]={FLOW_PREPARE(),FLOW_IMU_ZERO(),FLOW_STORE(FLOW_BATCH_2),FLOW_END()};
    assert(!strcmp(Flow_Check(missing_code,FLOW_COUNT(missing_code)),"FLOW_TASK_CODE"));
}
int main(void)
{
    test_validation();
    puts("PASS: action validation without batch inspection, free station order, repeated scans/actions and required QR/reference");
}
