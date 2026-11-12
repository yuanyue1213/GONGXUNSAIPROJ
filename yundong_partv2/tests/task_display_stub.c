/* 任务执行回归测试不访问真实GPIO。 */
#include <assert.h>
#include "task_display.h"
unsigned display_progress;
static bool loaded;
void TaskDisplay_Init(void) {}
void TaskDisplay_Task(const TaskCode *code) { loaded=code!=NULL; display_progress=0; }
void TaskDisplay_Progress(unsigned batch,unsigned item,unsigned stage)
{ assert(loaded&&batch<2&&item<3&&stage<4); ++display_progress; }
void TaskDisplay_State(DisplayState state) { assert(state<=DISPLAY_ERROR); }
