from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8')
a='static void advance_grab_hold(void) { tick += 1000; RobotControl_Tick(); }'
b=a+'''
static void finish_gripper_open(void)
{
    unsigned r_count = absolute_commands[1], z_count = absolute_commands[0];
    tick += 500; RobotControl_Tick();
    assert(servo_channel == 'G' && servo_angle == 30);
    assert(absolute_commands[1] == r_count && absolute_commands[0] == z_count);
    tick += 500; RobotControl_Tick();
    assert(servo_channel == 'G' && servo_angle == 60);
}
'''
assert a in s;s=s.replace(a,b)
a='    advance_grab_hold(); assert(servo_history[servo_commands-2] == 60 && absolute_target[1] == 0); /* State 7. */'
b='''    advance_grab_hold(); assert(servo_history[servo_commands-2] == 0 && absolute_target[1] == 567); /* State 7 starts slow opening. */
    finish_gripper_open(); assert(absolute_target[1] == 0);'''
assert a in s;s=s.replace(a,b)
a='    advance_grab_hold(); assert(servo_history[servo_commands-2] == 60); /* Step 7 releases. */'
b='''    advance_grab_hold(); assert(servo_history[servo_commands-2] == 0); /* Step 7 starts slow release. */
    finish_gripper_open();'''
assert a in s;s=s.replace(a,b)
# Verify cancel during ramp stops further servo updates and does not start homing.
a='static void test_grab_state(void)'
b='''static void test_gripper_open_cancel(void)
{
    reset(); feed("CMD,200,A\\n"); finish_grab_axes();
    advance_grab_hold(); lift_flags = 3; tick += 50; RobotControl_Tick();
    arm_flags = 3; tick += 50; RobotControl_Tick(); arm_flags = lift_flags = 1;
    advance_grab_hold(); advance_grab_hold(); finish_grab_axes();
    advance_grab_hold(); tick += 2000; RobotControl_Tick();
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    unsigned r_count = absolute_commands[1];
    tick += 500; RobotControl_Tick(); assert(servo_channel == 'G' && servo_angle == 30);
    feed("CMD,201,Z\\n"); unsigned servos = servo_commands;
    tick += 1500; RobotControl_Tick();
    assert(servo_commands == servos && absolute_commands[1] == r_count);
}
'''+a
assert a in s;s=s.replace(a,b).replace('    test_grab_state();','    test_grab_state();\n    test_gripper_open_cancel();');p.write_text(s,encoding='utf-8')
for path in ['output/目前已有状态与扩展说明.md','yundong_part/esp32s3/PROTOCOL.md']:
 p=Path(path);s=p.read_text(encoding='utf-8');s+='\n抓取和放下流程中夹子由 0° 张开到 60° 时，采用 1 秒平滑转动，每 20 ms 更新目标角度。抓取第 7 步张开完成后再回零；放下第 7 步张开完成后保持 1 秒，再进入第 8 步。停止流程可取消张开，手动舵机命令仍按输入角度执行。\n';p.write_text(s,encoding='utf-8')
