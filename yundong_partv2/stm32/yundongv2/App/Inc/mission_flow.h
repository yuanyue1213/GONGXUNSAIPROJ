/* 完整比赛流程的编辑入口：按数组顺序执行，修改后需重新编译并烧录 STM32。
 * MOVE 单位 mm，TURN 单位度；每个站点操作负责所选二维码批次的三件物料。
 * 编写约定见项目根目录 docs/FLOW_AUTHORING.md。 */
#ifndef MISSION_FLOW_H
#define MISSION_FLOW_H
#include "flow_types.h"
#include "arm_flow.h"
#include "task_protocol.h"

/* 自定义机械臂动作组：在 arm_flow.h 写 const ArmStep 数组，
 * 然后在 PREPARE 之后、END 之前插入 FLOW_ARM(数组名)，不要求 IMU。
 * 例如 FLOW_MOVE(FLOW_FORWARD,500), FLOW_ARM(arm_ready), ... */

/* Edit the mission here, then rebuild/flash STM32.
 * Body coordinates: tail initially faces the other start/stop area.
 * Each station action handles all three materials in the selected QR batch.
 * Process PUT and GET are separate entries: place all three before retrieval.
 */
static const FlowStep mission_flow[] = {
    /* 起步准备：建立姿态参考并收拢机械臂，再进入路线。 */
    FLOW_PREPARE(),
    FLOW_IMU_ZERO(),
    FLOW_STOW(),

    /* 出发并读取任务二维码，然后驶向第一批原料区。 */
    FLOW_MOVE(FLOW_LEFT,      100), /* 01 */
    FLOW_MOVE(FLOW_BACKWARD, 1000), /* 02 */
    FLOW_SCAN_TASK(),
    FLOW_MOVE(FLOW_FORWARD,   900), /* 03 */
    FLOW_TURN(FLOW_LEFT,       90), /* 04 */
    FLOW_MOVE(FLOW_FORWARD,  1000), /* 05 */
    FLOW_ALIGN(TASK_TABLE,FLOW_BATCH_1),
    FLOW_PICK_RAW(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第一批加工：先放完三件，再按流程取回三件。 */
    FLOW_TURN(FLOW_LEFT,       90), /* 06 */
    FLOW_MOVE(FLOW_FORWARD,  1900), /* 07 */
    FLOW_TURN(FLOW_LEFT,       90), /* 08 */
    FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),
    FLOW_PROCESS_PUT(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),
    FLOW_ALIGN(TASK_OCCUPIED,FLOW_BATCH_1),
    FLOW_PROCESS_GET(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第一批送往暂存区。 */
    FLOW_MOVE(FLOW_BACKWARD,  900), /* 09 */
    FLOW_TURN(FLOW_RIGHT,      90), /* 10 */
    FLOW_MOVE(FLOW_BACKWARD,  900), /* 11 */
    FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),
    FLOW_STORE(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 返回原料区，开始第二批物料。 */
    FLOW_MOVE(FLOW_BACKWARD,  900), /* 12 */
    FLOW_TURN(FLOW_RIGHT,      90), /* 13 */
    FLOW_MOVE(FLOW_BACKWARD,  900), /* 14 */
    FLOW_ALIGN(TASK_TABLE,FLOW_BATCH_2),
    FLOW_PICK_RAW(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第二批加工，同样将放置与取回分成两个站点步骤。 */
    FLOW_TURN(FLOW_LEFT,       90), /* 15 */
    FLOW_MOVE(FLOW_FORWARD,  1900), /* 16 */
    FLOW_TURN(FLOW_LEFT,       90), /* 17 */
    FLOW_ALIGN(TASK_RING,FLOW_BATCH_2),
    FLOW_PROCESS_PUT(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),
    FLOW_ALIGN(TASK_OCCUPIED,FLOW_BATCH_2),
    FLOW_PROCESS_GET(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第二批送往码垛区。 */
    FLOW_MOVE(FLOW_BACKWARD,  900), /* 18 */
    FLOW_TURN(FLOW_RIGHT,      90), /* 19 */
    FLOW_MOVE(FLOW_BACKWARD,  900), /* 20 */
    FLOW_ALIGN(TASK_OCCUPIED,FLOW_BATCH_2),
    FLOW_STACK(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 完成所有取放后返回终点，END 将全流程置为完成。 */
    FLOW_TURN(FLOW_LEFT,       90), /* 21 */
    FLOW_MOVE(FLOW_FORWARD,  1900), /* 22 */
    FLOW_TURN(FLOW_RIGHT,      90), /* 23 */
    FLOW_MOVE(FLOW_FORWARD,  1000), /* 24 */
    FLOW_MOVE(FLOW_LEFT,      100), /* 25 */
    FLOW_END(),
};
#endif
