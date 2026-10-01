from pathlib import Path
p=Path('F:/GONGXUNSAIPROJ/yundong_part/tests/distance_control_test.c')
s=p.read_text(encoding='utf-8'); a=s.index('static void test_grab_state(void)'); b=s.index('static void test_arm_distance_control',a)
s=s[:a]+'''static void grab_to_retract(void)
{
    reset(); feed("CMD,20,A\\n");
    assert(servo_commands == 3 && arm_angle_commands == 0 && servo_angle == 263);
    tick = 1000; RobotControl_Tick();
    assert(arm_angle_commands == 1 && last_arm_angle.direction == 'E');
    assert(last_arm_angle.angle_tenths == 3118);
    arm_flags = 3; tick = 1050; RobotControl_Tick();
    assert(servo_commands == 6 && servo_history[4] == 0);
    arm_flags = 1; tick = 2050; RobotControl_Tick();
    assert(arm_angle_commands == 2 && last_arm_angle.direction == 'C');
    assert(last_arm_angle.angle_tenths == 2551 && servo_commands == 6);
}
static void grab_to_base_ramp(void)
{
    grab_to_retract();
    arm_flags = 3; tick = 2100; RobotControl_Tick();
    arm_flags = 1;
    assert(servo_commands == 9 && servo_angle == 263);
}
static void test_grab_state(void)
{
    reset(); feed("CMD,20,A\\n");
    feed("CMD,21,A\\n"); tick = 999; RobotControl_Tick();
    assert(servo_commands == 3 && arm_angle_commands == 0);
    tick = 1000; RobotControl_Tick();
    assert(arm_angle_commands == 1 && last_arm_angle.angle_tenths == 3118);
    feed("ARM_MOVE,30,E,10,10,3200\\n"); assert(arm_angle_commands == 1);
    tick = 1500; RobotControl_Tick(); assert(servo_commands == 3); /* Wait for real reached flag. */
    arm_flags = 3; tick = 1550; RobotControl_Tick();
    assert(servo_commands == 6 && servo_history[4] == 0);
    arm_flags = 1; tick = 2550; RobotControl_Tick();
    assert(arm_angle_commands == 2 && last_arm_angle.direction == 'C' && last_arm_angle.angle_tenths == 2551);
    tick = 3050; RobotControl_Tick(); assert(servo_commands == 6);
    arm_flags = 3; tick = 3100; RobotControl_Tick();
    assert(servo_commands == 9 && servo_angle == 263);
    tick = 3119; RobotControl_Tick(); assert(servo_commands == 9);
    tick = 3120; RobotControl_Tick(); assert(servo_angle == 262);
    tick = 4600; RobotControl_Tick(); assert(servo_angle == 203);
    tick = 6080; RobotControl_Tick(); assert(servo_angle == 145);
    tick = 6100; RobotControl_Tick(); assert(servo_angle == 144);
    unsigned reached = servo_commands;
    tick = 7099; RobotControl_Tick(); assert(servo_commands == reached);
    tick = 7100; RobotControl_Tick(); assert(servo_history[servo_commands - 2] == 60);
    tick = 8100; RobotControl_Tick(); assert(servo_history[servo_commands - 3] == 120);
    tick = 9100; RobotControl_Tick(); assert(servo_history[servo_commands - 3] == 240 && servo_angle == 144);
    tick = 10600; RobotControl_Tick(); assert(servo_angle == 204);
    tick = 12100; RobotControl_Tick(); assert(servo_angle == 263);
    tick = 13100; RobotControl_Tick();
    unsigned complete = servo_commands;
    feed("CMD,20,A\\n"); assert(servo_commands == complete && arm_angle_commands == 2);

    grab_to_base_ramp(); tick = 2500; RobotControl_Tick();
    unsigned before_cancel = servo_commands;
    feed("CMD,31,Z\\n"); tick = 5000; RobotControl_Tick();
    assert(servo_commands == before_cancel);
    reset(); feed("CMD,32,A\\n"); tick = 1000; RobotControl_Tick();
    feed("CMD,33,Z\\n"); assert(arm_stops == 1);
    arm_flags = 3; tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    grab_to_retract(); feed("SERVO,34,G,90\\n");
    assert(arm_stops == 1); unsigned manual = servo_commands;
    arm_flags = 3; tick = 5000; RobotControl_Tick(); assert(servo_commands == manual);
    grab_to_retract(); fail_arm_stop = 1; feed("CMD,35,Q\\n");
    assert(arm_stops == 1); fail_arm_stop = 0; tick = 2100; RobotControl_Tick();
    assert(arm_stops == 2 && servo_commands == 6);
    reset(); fail_servo = 1; feed("CMD,40,A\\n");
    tick = 1000; RobotControl_Tick(); assert(servo_commands == 1 && arm_angle_commands == 0);
    reset(); feed("CMD,41,A\\n"); fail_arm_angle = 1; tick = 1000; RobotControl_Tick();
    tick = 5000; RobotControl_Tick(); assert(servo_commands == 3 && arm_stops == 1);
    grab_to_retract(); arm_flags = 0x0D; tick = 2100; RobotControl_Tick();
    tick = 5000; RobotControl_Tick(); assert(servo_commands == 6 && arm_stops == 1);
    grab_to_base_ramp(); fail_servo = 1; tick = 2120; RobotControl_Tick();
    unsigned failed = servo_commands; tick = 5000; RobotControl_Tick(); assert(servo_commands == failed);
}
'''+s[b:];p.write_text(s,encoding='utf-8')
