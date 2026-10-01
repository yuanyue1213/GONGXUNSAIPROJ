from pathlib import Path
path = Path('F:/GONGXUNSAIPROJ/yundong_part/tests/distance_control_test.c')
text = path.read_text(encoding='utf-8')
text = text.replace('servo_history[64]', 'servo_history[512]').replace('servo_channels[64]', 'servo_channels[512]').replace('servo_commands < 64', 'servo_commands < 512')
a = text.index('static void grab_to_retract(void)')
b = text.index('static void test_arm_distance_control(void)', a)
text = text[:a] + '''static void finish_grab_axes(void)
{
    arm_flags = 3; tick += 50; RobotControl_Tick();
    lift_flags = 3; tick += 50; RobotControl_Tick();
    arm_flags = lift_flags = 1;
}
static void advance_grab_hold(void) { tick += 1000; RobotControl_Tick(); }
static void finish_grab_cycle(unsigned seq, uint16_t expected_turntable)
{
    feed("CMD,999,Z\\n"); /* Only cancels; does not reset the completed rotation index. */
    char frame_text[32]; snprintf(frame_text, sizeof(frame_text), "CMD,%u,A\\n", seq); feed(frame_text);
    assert(absolute_target[1] == 0 && servo_angle == 263); /* State 1: home. */
    finish_grab_axes(); assert(absolute_target[0] == 0);
    advance_grab_hold(); assert(absolute_target[1] == 3118); /* State 2: r110,z-50. */
    finish_grab_axes(); assert(absolute_target[0] == 4500);
    unsigned r_count = absolute_commands[1], z_count = absolute_commands[0];
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 0 && servo_angle == 263); /* State 3. */
    assert(absolute_commands[1] == r_count && absolute_commands[0] == z_count);
    advance_grab_hold(); assert(absolute_target[1] == 0); /* State 4. */
    finish_grab_axes(); assert(absolute_target[0] == 0);
    advance_grab_hold(); assert(servo_angle == 263); /* State 5 starts the base ramp. */
    tick += 1500; RobotControl_Tick(); assert(servo_angle == 203);
    tick += 1500; RobotControl_Tick(); assert(servo_angle == 144);
    advance_grab_hold(); assert(absolute_target[1] == 567); /* State 6: r20,z-40. */
    finish_grab_axes(); assert(absolute_target[0] == 3600);
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 60 && absolute_target[1] == 0); /* State 7. */
    finish_grab_axes(); assert(absolute_target[0] == 0);
    advance_grab_hold(); assert(servo_angle == 144); /* State 8. */
    tick += 1500; RobotControl_Tick(); assert(servo_angle == 204);
    tick += 1500; RobotControl_Tick(); assert(servo_angle == 263);
    advance_grab_hold(); assert(servo_channel == 'T' && servo_angle == expected_turntable);
    unsigned before = servo_commands; tick += 1000; RobotControl_Tick(); assert(servo_commands == before);
    feed(frame_text); assert(servo_commands == before); /* Same accepted sequence must not replay. */
}
static void test_grab_state(void)
{
    reset(); finish_grab_cycle(20, 120); finish_grab_cycle(21, 240); finish_grab_cycle(22, 0);
    assert(absolute_commands[1] == 15 && absolute_commands[0] == 15); /* Five coordinate states per cycle. */
    reset(); feed("CMD,30,A\\nCMD,31,A\\nARM_POSE,32,0,0,0,5,3200\\nARM_MOVE,33,E,10,10,3200\\n");
    assert(arm_angle_commands == 1 && servo_commands == 3);
    feed("CMD,34,Z\\n"); assert(arm_stops == 1); tick = 5000; RobotControl_Tick(); assert(lift_angle_commands == 0);
    reset(); feed("CMD,35,A\\n"); arm_flags = 3; tick = 50; RobotControl_Tick();
    assert(lift_angle_commands == 1); feed("CMD,36,H\\n"); assert(lift_stops == 1);
    lift_flags = 3; tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    reset(); feed("CMD,37,A\\nSERVO,38,G,90\\n"); assert(arm_stops == 1 && servo_angle == 90);
    reset(); feed("CMD,39,A\\nCMD,40,Q\\n"); assert(arm_stops == 1);
    reset(); fail_servo = 1; feed("CMD,41,A\\n"); assert(arm_angle_commands == 0);
    reset(); fail_arm_angle = 1; feed("CMD,42,A\\n"); assert(arm_stops == 1);
    reset(); feed("CMD,43,A\\n"); fail_lift_angle = 1; arm_flags = 3; tick = 50; RobotControl_Tick();
    assert(lift_stops == 1); tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    reset(); feed("CMD,44,A\\n"); arm_flags = 3; tick = 50; RobotControl_Tick();
    lift_flags = 0x0D; tick = 100; RobotControl_Tick(); assert(lift_stops == 1);
    tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    reset(); feed("CMD,45,A\\n"); fail_arm_stop = 1; feed("CMD,46,Z\\n");
    assert(arm_stops == 1); fail_arm_stop = 0; tick = 50; RobotControl_Tick(); assert(arm_stops == 2);
}
''' + text[b:]
a = text.index('static void test_grab_status_reply_recovery(void)')
b = text.index('static void test_lift_calibration(void)', a)
text = text[:a] + '''static void test_grab_status_reply_recovery(void)
{
    reset(); feed("CMD,60,A\\n");
    fail_arm_status = 1; tick = 50; RobotControl_Tick(); tick = 200; RobotControl_Tick();
    assert(arm_stops == 0 && lift_angle_commands == 0 && servo_commands == 3);
    fail_arm_status = 0; arm_flags = 3; tick = 250; RobotControl_Tick(); assert(lift_angle_commands == 1);
    fail_lift_status = 1; tick = 300; RobotControl_Tick(); tick = 450; RobotControl_Tick();
    assert(lift_stops == 0 && servo_commands == 3);
    fail_lift_status = 0; lift_flags = 3; tick = 500; RobotControl_Tick();
    tick = 1499; RobotControl_Tick(); assert(servo_commands == 3);
    tick = 1500; RobotControl_Tick(); assert(absolute_target[1] == 3118 && servo_commands == 6);
    reset(); feed("CMD,61,A\\n"); fail_arm_status = 1; tick = 500; RobotControl_Tick();
    assert(arm_stops == 1); fail_arm_status = 0; arm_flags = 3; tick = 5000; RobotControl_Tick();
    assert(lift_angle_commands == 0 && servo_commands == 3);
    reset(); feed("CMD,62,A\\n"); arm_flags = 3; tick = 50; RobotControl_Tick();
    fail_lift_status = 1; tick = 550; RobotControl_Tick(); assert(lift_stops == 1);
    fail_lift_status = 0; lift_flags = 3; tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
}
''' + text[b:]
path.write_text(text, encoding='utf-8')
