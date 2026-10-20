#ifndef AUTONOMOUS_TASK_H
#define AUTONOMOUS_TASK_H
#include <stdint.h>
#include <stdbool.h>
#include "../../../shared/task_protocol.h"

typedef void (*AutoOutput)(const char *line);
void AutoTask_Init(AutoOutput stm, AutoOutput event);
bool AutoTask_Command(const char *line, uint32_t now);
void AutoTask_Input(const char *line, uint32_t now);
void AutoTask_Tick(uint32_t now);
void AutoTask_Stop(const char *reason);
bool AutoTask_Running(void);
#endif
