"""Host regression tests for the sequential mission architecture (Python 3 + GCC)."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
app = root / 'stm32/yundongv2/App'
out = root / 'tests/build'
out.mkdir(exist_ok=True)
cases = {
    'mission_arm_test': ['flow_check.c', 'arm_flow.c'],
    'arm_flow_test': ['arm_flow.c'],
    'sequential_mission_test': ['mission_runner.c', 'flow_check.c', 'arm_flow.c'],
    'sequential_motion_test': ['motion_control.c'],
    'start_link_test': ['start_link.c'],
    'camera_link_test': ['camera_link.c'],
    'flow_check_test': ['flow_check.c'],
    'servo_trajectory_test': ['servo_trajectory.c'],
    'arm_angle_driver_test': ['lift_motor.c'],
    'wheel_driver_test': ['zdt_motor.c', 'zdt_motor_uart.c'],
    'imu_link_test': ['imu_link.c'],
}
for name, modules in cases.items():
    sources = [root/'tests'/f'{name}.c', *[app/'Src'/f for f in modules]]
    exe = out/f'{name}.exe'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root/'tests'), '-I'+str(app/'Inc'), '-I'+str(root/'shared'),
                    *map(str, sources), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=30)
