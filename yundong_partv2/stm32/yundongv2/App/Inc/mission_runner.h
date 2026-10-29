#ifndef MISSION_RUNNER_H
#define MISSION_RUNNER_H
#include <stdbool.h>
#include <stdint.h>
bool MissionRunner_Running(void);
void MissionRunner_Run(uint32_t id);
#endif
