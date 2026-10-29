/* 机械臂和舵机动作表。r: 0.1 mm，z: mm，角度: 度，等待: ms。
 * 宏展开为数组初始化项，区域位置统一在执行时查表。 */
#ifndef ARM_FLOW_H
#define ARM_FLOW_H
#include "arm_types.h"

#define ARM_PREPARE_STEPS \
    ARM_SET('G',ARM_INITIAL_GRIP), ARM_SET('T',ARM_INITIAL_TURNTABLE), ARM_SET('B',ARM_INITIAL_BASE)

/* 先到准备位。原料在伸出下降前等 NEAR，加工取回不等待。 */
#define ARM_PICK_APPROACH(area,pos) \
    ARM_AREA(area), \
    ARM_SERVO('G',ARM_GRIP_OPEN), \
    ARM_TO_READY(pos)

#define ARM_PICK_FINISH(area,pos) \
    ARM_AREA(area), \
    ARM_TO_PICK(pos), \
    ARM_SERVO('G',ARM_GRIP_CLOSED), \
    ARM_WAIT(ARM_GRIP_SETTLE_MS), \
    ARM_ABOVE_PICK(pos), \
    ARM_MOVE(ARM_SLOT_R,0,ARM_SLOT_BASE), \
    ARM_MOVE(ARM_SLOT_R,ARM_SLOT_Z,ARM_SLOT_BASE), \
    ARM_SERVO('G',ARM_GRIP_OPEN), \
    ARM_WAIT(ARM_RELEASE_SETTLE_MS), \
    ARM_MOVE(ARM_SLOT_R,0,ARM_SLOT_BASE), \
    ARM_SERVO('B',(area)==ARM_AREA_RAW?ARM_RAW_RETURN_BASE:ARM_SLOT_BASE)

#define ARM_PUT_STEPS(area,pos) \
    ARM_AREA(area), \
    ARM_MOVE(ARM_SLOT_R,0,ARM_SLOT_BASE), \
    ARM_SERVO('G',ARM_GRIP_OPEN), \
    ARM_MOVE(ARM_SLOT_R,ARM_SLOT_Z,ARM_SLOT_BASE), \
    ARM_SERVO('G',ARM_GRIP_CLOSED), \
    ARM_WAIT(ARM_GRIP_SETTLE_MS), \
    ARM_MOVE(ARM_SLOT_R,0,ARM_SLOT_BASE), \
    ARM_ABOVE_PUT(pos), \
    ARM_TO_PUT(pos), \
    ARM_SERVO('G',ARM_GRIP_OPEN), \
    ARM_WAIT(ARM_RELEASE_SETTLE_MS), \
    ARM_ABOVE_PUT(pos), \
    ARM_TO_READY(pos)

/* 先保持伸缩和底座抬升，再收臂、旋转，最后转盘回零。 */
#define ARM_STOW_STEPS(r,b) \
    ARM_MOVE_AT(r,0,b,MOTION_STOW_RPM), \
    ARM_MOVE_AT(0,0,ARM_STOW_BASE,MOTION_STOW_RPM), \
    ARM_SERVO('T',ARM_INITIAL_TURNTABLE)

/* 自定义动作组写在这里，在 mission_flow.h 用 FLOW_ARM(arm_ready) 插入。
 * 示例：到准备姿态、张爪、停留；未加入默认比赛路线。 */
static const ArmStep arm_ready[]={
    ARM_AREA(ARM_AREA_RAW),
    ARM_MOVE(ARM_READY_R,0,ARM_RAW_RETURN_BASE),
    ARM_SERVO('G',ARM_GRIP_OPEN),
    ARM_WAIT(ARM_RELEASE_SETTLE_MS),
};

#endif
