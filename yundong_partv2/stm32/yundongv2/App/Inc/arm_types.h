/* 机械臂动作格式和初始化宏；具体动作组写在 arm_flow.h。 */
#ifndef ARM_TYPES_H
#define ARM_TYPES_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motion_speed.h"

typedef enum { ARM_AREA_NONE, ARM_AREA_RAW, ARM_AREA_PROCESS, ARM_AREA_STORE, ARM_AREA_STACK } ArmArea;
typedef enum { ARM_LEFT, ARM_CENTER, ARM_RIGHT } ArmPosition;
typedef struct { int32_t pick_r,put_r,z; uint16_t base; } StationPose;
typedef enum {
    ARM_OP_MOVE, ARM_OP_SERVO, ARM_OP_WAIT, ARM_OP_SET,
    ARM_OP_AREA, ARM_OP_PICK_POSITION, ARM_OP_PUT_POSITION,
    ARM_OP_READY_POSITION, ARM_OP_PICK_ABOVE, ARM_OP_PUT_ABOVE
} ArmOp;
typedef struct ArmStep {
    ArmOp op;
    int32_t r,z;
    uint16_t base,speed;
    char channel;
    uint32_t value;
} ArmStep;

#define ARM_MOVE_AT(r_,z_,b_,rpm_) { .op=ARM_OP_MOVE, .r=(r_), .z=(z_), .base=(b_), .speed=(rpm_) }
#define ARM_MOVE(r,z,b) ARM_MOVE_AT(r,z,b,MOTION_ARM_RPM)
#define ARM_SERVO_AT(c,a,dps) { .op=ARM_OP_SERVO, .channel=(c), .value=(a), .speed=(dps) }
#define ARM_WAIT(ms) { .op=ARM_OP_WAIT, .value=(ms) }
#define ARM_SET(c,a) { .op=ARM_OP_SET, .channel=(c), .value=(a) }
#define ARM_COUNT(a) (sizeof(a)/sizeof((a)[0]))
/* 声明当前动作组的作业区域；区域位置动作只移动，不自动开合夹爪。 */
#define ARM_AREA(area) { .op=ARM_OP_AREA, .value=(area) }
#define ARM_TO_PICK(pos) { .op=ARM_OP_PICK_POSITION, .value=(pos), .speed=MOTION_ARM_RPM }
#define ARM_TO_PUT(pos) { .op=ARM_OP_PUT_POSITION, .value=(pos), .speed=MOTION_ARM_RPM }
#define ARM_TO_READY(pos) { .op=ARM_OP_READY_POSITION, .value=(pos), .speed=MOTION_ARM_RPM }
#define ARM_ABOVE_PICK(pos) { .op=ARM_OP_PICK_ABOVE, .value=(pos), .speed=MOTION_ARM_RPM }
#define ARM_ABOVE_PUT(pos) { .op=ARM_OP_PUT_ABOVE, .value=(pos), .speed=MOTION_ARM_RPM }

/* 宏用于静态数组，默认速度必须是编译期常量。 */
#define ARM_SERVO(c,a) ARM_SERVO_AT(c,a, \
    (c)=='G' ? MOTION_GRIPPER_DPS : \
    (c)=='T' ? MOTION_TURNTABLE_DPS : MOTION_BASE_TRANSFER_DPS)

bool ArmFlow_Run(const ArmStep *steps,size_t count);
bool ArmFlow_Stow(void);
#endif
