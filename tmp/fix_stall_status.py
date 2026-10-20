from pathlib import Path
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c')
s=p.read_text(encoding='utf-8')
s=s.replace('#include "../../../../shared/arm_vector_math.h"', '#include "../../../../shared/arm_vector_math.h"\n#include "../../../../shared/motor_status_guard.h"')
s=s.replace('static bool s_wheel_move_reached;', 'static bool s_wheel_move_reached;\nstatic MotorStatusGuard s_wheel_status_guard[4], s_arm_status_guard, s_lift_status_guard;')
old='if ((flags & 0x01U) == 0U || (flags & 0x0CU) != 0U)'
assert s.count(old)==2
s=s.replace(old,'if (MotorStatusGuard_Fault(&s_arm_status_guard, flags, HAL_GetTick()))',1)
s=s.replace(old,'if (MotorStatusGuard_Fault(&s_lift_status_guard, flags, HAL_GetTick()))',1)
s=s.replace("((flags & 0x01U) == 0U) || ((flags & 0x0CU) != 0U)", 'MotorStatusGuard_Fault(&s_wheel_status_guard[s_status_wheel], flags, HAL_GetTick())')
# Clear per-command history; a later move must not inherit the previous Cgi timer.
s=s.replace('s_arm_move_reached = false;', 's_arm_status_guard.pending = false;\n    s_arm_move_reached = false;')
s=s.replace('s_lift_angle_reached = false;', 's_lift_status_guard.pending = false;\n    s_lift_angle_reached = false;')
s=s.replace('s_arm_move_stop_pending = s_arm_move_reached = false;', 's_arm_status_guard.pending = false;\n        s_arm_move_stop_pending = s_arm_move_reached = false;')
s=s.replace('s_lift_angle_stop_pending = s_lift_angle_reached = false;', 's_lift_status_guard.pending = false;\n        s_lift_angle_stop_pending = s_lift_angle_reached = false;')
s=s.replace('s_move_started_tick = HAL_GetTick();', 's_move_started_tick = HAL_GetTick(); memset(s_wheel_status_guard, 0, sizeof(s_wheel_status_guard));')
s=s.replace('    s_command_uart = command_uart;', '    memset(s_wheel_status_guard, 0, sizeof(s_wheel_status_guard));\n    s_arm_status_guard = s_lift_status_guard = (MotorStatusGuard){0};\n    s_command_uart = command_uart;')
p.write_text(s,encoding='utf-8')
