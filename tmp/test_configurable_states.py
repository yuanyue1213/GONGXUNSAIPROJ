from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');mark='static void test_absolute_pose(void)'
test='''static void test_configured_sequences(void)
{
    RobotSequenceCommand config;
    assert(RobotProtocol_ParseSequence("STATE,1,A,1155,-55,250,-45,25,15,55", &config));
    assert(config.r1 == 1155 && config.down_rpm == 55);
    const char *bad[] = {
        "STATE,0,A,1100,-50,200,-40,20,20,50",
        "STATE,1,X,1100,-50,200,-40,20,20,50",
        "STATE,1,A,10001,-50,200,-40,20,20,50",
        "STATE,1,A,1100,-401,200,-40,20,20,50",
        "STATE,1,A,1100,-50,200,-40,20,20,61",
        "STATE,1,A,1100,-50,200,-40,20,20,50,junk"
    };
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i)
        assert(!RobotProtocol_ParseSequence(bad[i], &config));
    reset(); feed("STATE,300,A,1155,-55,250,-45,25,15,55\\n");
    assert(last_arm_angle.speed_rpm == 25);
    finish_grab_axes(); assert(last_lift_angle.rpm == 15);
    advance_grab_hold(); assert(absolute_target[0] == 4950 && last_lift_angle.rpm == 55);
    unsigned servos = servo_commands;
    feed("STATE,301,P,100,-10,100,-10,10,10,10\\n"); assert(servo_commands == servos); /* Busy cannot change settings. */
    finish_z_first_axes(); assert(absolute_target[1] == 3274 && last_arm_angle.speed_rpm == 25);
    advance_grab_hold(); advance_grab_hold(); finish_grab_axes(); assert(last_lift_angle.rpm == 15);
    advance_grab_hold(); tick += 2000; RobotControl_Tick();
    advance_grab_hold(); assert(absolute_target[1] == 709);
    finish_grab_axes(); assert(absolute_target[0] == 4050 && last_lift_angle.rpm == 55);
    feed("CMD,302,Z\\n");
    reset(); feed("STATE,400,P,250,-45,1350,-115,26,16,56\\n");
    finish_grab_axes(); advance_grab_hold(); tick += 2000; RobotControl_Tick();
    advance_grab_hold(); assert(absolute_target[0] == 4050 && last_lift_angle.rpm == 56);
    finish_z_first_axes(); assert(absolute_target[1] == 709 && last_arm_angle.speed_rpm == 26);
    advance_grab_hold(); assert(last_lift_angle.rpm == 16); finish_z_first_axes();
    advance_grab_hold(); tick += 2000; RobotControl_Tick(); advance_grab_hold();
    assert(absolute_target[1] == 3827 && last_arm_angle.speed_rpm == 26);
    finish_grab_axes(); assert(absolute_target[0] == 10350 && last_lift_angle.rpm == 56);
    advance_grab_hold(); finish_gripper_open(); advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    servos = servo_commands; feed("STATE,400,P,250,-45,1350,-115,26,16,56\\n");
    assert(servo_commands == servos); /* Duplicate cannot replay. */
    feed("CMD,401,P\\n"); assert(last_arm_angle.speed_rpm == 20); /* Legacy command uses defaults. */
    reset(); feed("STATE,500,A,1100,-50,200,-40,20,20,61\\n"); assert(absolute_commands[1] == 0);
}
'''
assert mark in s;s=s.replace(mark,test+mark).replace('    test_absolute_pose();','    test_absolute_pose();\n    test_configured_sequences();');p.write_text(s,encoding='utf-8')
p=Path('tmp/test_esp_servo_forwarding.py');s=p.read_text(encoding='utf-8');s=s.replace('"ALIGN_RING,1577625120,2,9889,12000",','"ALIGN_RING,1577625120,2,9889,12000", "STATE,1577625121,P,200,-40,1300,-110,20,20,50",');s=s.replace('    puts("PASS:', '    assert(handle_frame("STATE,1,P,200,-40,1300,-110,20,20,61", &lift));\n    puts("PASS:');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt');s=p.read_text(encoding='utf-8').replace('    Text(status, modifier = Modifier.fillMaxWidth())\n','').replace('    Text(note)\n','    if (note.startsWith("请输入") || note.startsWith("请检查")) Text(note, color = MaterialTheme.colorScheme.error)\n');p.write_text(s,encoding='utf-8')
