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
 * Body coordinates: front initially faces the other start/stop area.
 * Each station action handles all three materials in the selected QR batch.
 * Process PUT and GET are separate entries: place all three before retrieval.
 */
static const FlowStep mission_flow[] = {
    /* 起步准备：建立姿态参考并收拢机械臂，再进入路线。 */
    FLOW_PREPARE(),
    FLOW_IMU_ZERO(),
    FLOW_STOW(),

    /* 起步125/165、两次雷达采样，等待PC完整路径；之后按段执行导航。 */
    FLOW_MAP_PREPARE(),
    FLOW_NAV(NAV_QR),
    FLOW_SCAN_TASK(),
    FLOW_NAV(NAV_RAW_1),
    FLOW_ALIGN(TASK_TABLE,FLOW_BATCH_1),
    FLOW_PICK_RAW(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第一批加工：放完原地取回，最后统一收臂、撤销校准。 */
    FLOW_NAV(NAV_COARSE_1),
    FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),
    FLOW_PROCESS_PUT(FLOW_BATCH_1),
    FLOW_PROCESS_GET(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第一批送往暂存区。 */
    FLOW_NAV(NAV_TEMP_1),
    FLOW_ALIGN(TASK_RING,FLOW_BATCH_1),
    FLOW_STORE(FLOW_BATCH_1),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 返回原料区，开始第二批物料。 */
    FLOW_NAV(NAV_RAW_2),
    FLOW_ALIGN(TASK_TABLE,FLOW_BATCH_2),
    FLOW_PICK_RAW(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第二批加工：放完原地取回，最后统一收臂、撤销校准。 */
    FLOW_NAV(NAV_COARSE_2),
    FLOW_ALIGN(TASK_RING,FLOW_BATCH_2),
    FLOW_PROCESS_PUT(FLOW_BATCH_2),
    FLOW_PROCESS_GET(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 第二批送往码垛区，用下层三色物块定位。 */
    FLOW_NAV(NAV_STACK),
    FLOW_ALIGN(TASK_OCCUPIED,FLOW_BATCH_2),
    FLOW_STACK(FLOW_BATCH_2),
    FLOW_STOW(),
    FLOW_UNALIGN(),

    /* 完成所有取放后返回终点，END 将全流程置为完成。 */
    FLOW_NAV(NAV_RETURN),
    FLOW_END(),
};
#endif
