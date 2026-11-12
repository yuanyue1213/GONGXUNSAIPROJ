"""Host regression tests for the sequential mission architecture (Python 3 + GCC)."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
app = root / 'stm32/yundongv2/App'
out = root / 'tests/build'
out.mkdir(exist_ok=True)
cases = {
    'task_display_test': ['task_display.c'],
    'forward_test': ['start_link.c'],
    'straight_motion_test': [],
    'navigation_only_test': [],
    'route_plan_test': ['route_plan.c'],
    'mission_arm_test': ['flow_check.c', 'arm_flow.c'],
    'arm_flow_test': ['arm_flow.c'],
    'sequential_mission_test': ['mission_runner.c', 'flow_check.c', 'arm_flow.c'],
    'fixed_qr_mission_test': ['mission_runner.c', 'flow_check.c', 'arm_flow.c'],
    'stack_mission_test': ['mission_runner.c', 'flow_check.c', 'arm_flow.c', 'start_link.c'],
    'sequential_motion_test': ['motion_control.c'],
    'start_link_test': ['start_link.c'],
    'camera_link_test': ['camera_link.c'],
    'flow_check_test': ['flow_check.c'],
    'servo_trajectory_test': ['servo_trajectory.c'],
    'arm_angle_driver_test': ['lift_motor.c'],
    'wheel_driver_test': ['zdt_motor.c', 'zdt_motor_uart.c'],
    'wheel_driver_forward_test': ['zdt_motor.c', 'zdt_motor_uart.c'],
    'imu_link_test': ['imu_link.c'],
}
for name, modules in cases.items():
    source = 'sequential_mission_test' if name == 'fixed_qr_mission_test' else name
    if name == 'wheel_driver_forward_test': source = 'wheel_driver_test'
    flags = ['-DFIXED_QR=1'] if name == 'fixed_qr_mission_test' else []
    if name == 'stack_mission_test':
        flags = ['-DSTACK_TEST=1', '-DFIXED_QR=1']
    sources = [root/'tests'/f'{source}.c', *[app/'Src'/f for f in modules]]
    if 'mission_runner.c' in modules or name == 'mission_arm_test':
        sources.append(root/'tests/task_display_stub.c')
    if name == 'forward_test':
        flags = ['-DFORWARD_TEST=1', '-DNAVIGATION_ONLY=1']
        sources.append(root/'stm32/yundongv2/Tests/forward_test.c')
    if name == 'straight_motion_test':
        flags = ['-DFORWARD_TEST=1', '-DNAVIGATION_ONLY=1']
        sources.append(root/'stm32/yundongv2/Tests/straight_motion.c')
    if name == 'wheel_driver_forward_test':
        flags = ['-DFORWARD_TEST=1', '-DNAVIGATION_ONLY=1']
    exe = out/f'{name}.exe'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    *flags,
                    '-I'+str(root/'tests'), '-I'+str(app/'Inc'), '-I'+str(root/'shared'),
                    *map(str, sources), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=30)
