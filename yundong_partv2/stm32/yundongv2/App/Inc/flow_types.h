/* 流程表的数据格式及初始化宏。MOVE 的 amount 单位为毫米，TURN 为角度。
 * 宏只生成编译期结构体；运行时读取结构体数组，不解析 .h 文件的源文本。 */
#ifndef FLOW_TYPES_H
#define FLOW_TYPES_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    FLOW_OP_PREPARE=1, FLOW_OP_IMU_ZERO, FLOW_OP_STOW,
    FLOW_OP_MOVE, FLOW_OP_TURN, FLOW_OP_SCAN,
    FLOW_OP_PICK_RAW, FLOW_OP_PROCESS_PUT, FLOW_OP_PROCESS_GET,
    FLOW_OP_STORE, FLOW_OP_STACK, FLOW_OP_END, FLOW_OP_ARM,
    FLOW_OP_ALIGN, FLOW_OP_UNALIGN
} FlowOp;
typedef enum {
    FLOW_FORWARD='F', FLOW_BACKWARD='B', FLOW_LEFT='L', FLOW_RIGHT='R'
} FlowDirection;
typedef enum { FLOW_BATCH_1=1, FLOW_BATCH_2=2 } FlowBatch;
struct ArmStep;
typedef struct {
    FlowOp op;
    FlowDirection direction;
    uint32_t amount; /* MOVE: mm; TURN: degrees. Positive values only. */
    FlowBatch batch;
    const struct ArmStep *arm; /* 自定义机械臂动作组。 */
    size_t arm_count;
    unsigned vision;
} FlowStep;

/* Initializers for const arrays; no side effects or run-time C source parsing. */
#define FLOW_PREPARE()       { .op=FLOW_OP_PREPARE }
#define FLOW_IMU_ZERO()      { .op=FLOW_OP_IMU_ZERO }
#define FLOW_STOW()          { .op=FLOW_OP_STOW }
#define FLOW_MOVE(dir,mm)    { .op=FLOW_OP_MOVE, .direction=(dir), .amount=(mm) }
#define FLOW_TURN(dir,deg)   { .op=FLOW_OP_TURN, .direction=(dir), .amount=(deg) }
#define FLOW_SCAN_TASK()     { .op=FLOW_OP_SCAN }
#define FLOW_PICK_RAW(b)     { .op=FLOW_OP_PICK_RAW, .batch=(b) }
#define FLOW_PROCESS_PUT(b)  { .op=FLOW_OP_PROCESS_PUT, .batch=(b) }
#define FLOW_PROCESS_GET(b)  { .op=FLOW_OP_PROCESS_GET, .batch=(b) }
#define FLOW_STORE(b)        { .op=FLOW_OP_STORE, .batch=(b) }
#define FLOW_STACK(b)        { .op=FLOW_OP_STACK, .batch=(b) }
#define FLOW_END()           { .op=FLOW_OP_END }
#define FLOW_ARM(group)      { .op=FLOW_OP_ARM, .arm=(group), .arm_count=sizeof(group)/sizeof((group)[0]) }
#define FLOW_ALIGN(kind,b)   { .op=FLOW_OP_ALIGN, .vision=(kind), .batch=(b) }
#define FLOW_UNALIGN()       { .op=FLOW_OP_UNALIGN }
#define FLOW_COUNT(table) (sizeof(table)/sizeof((table)[0]))
/* 返回 NULL 表示流程合法；在启动运动前调用一次。 */
#define FLOW_MAX_STEPS 128U
const char *Flow_Check(const FlowStep *steps, size_t count);
#endif
