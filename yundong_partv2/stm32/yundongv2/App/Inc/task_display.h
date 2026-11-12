#ifndef TASK_DISPLAY_H
#define TASK_DISPLAY_H
#include "task_protocol.h"
typedef enum { DISPLAY_WAIT,DISPLAY_RUNNING,DISPLAY_DONE,DISPLAY_ERROR } DisplayState;
void TaskDisplay_Init(void);
void TaskDisplay_Task(const TaskCode *code); /* NULL清空上轮任务及进度。 */
/* batch/item从0开始；stage依次为原料抓取、加工放置、加工取回、暂存/堆叠。 */
void TaskDisplay_Progress(unsigned batch,unsigned item,unsigned stage);
void TaskDisplay_State(DisplayState state);
#endif
