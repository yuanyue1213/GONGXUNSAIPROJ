from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');s=s.replace('    assert(config.r1 == 1155 && config.down_rpm == 55);','''    assert(config.r1 == 1155 && config.down_rpm == 55 && config.gripper_dps == 60);
    assert(RobotProtocol_ParseSequence("STATE,1,P,250,-45,1350,-115,26,16,56,30", &config));
    assert(config.gripper_dps == 30);
    assert(!RobotProtocol_ParseSequence("STATE,1,P,250,-45,1350,-115,26,16,56,0", &config));''');s=s.replace('STATE,400,P,250,-45,1350,-115,26,16,56\\n','STATE,400,P,250,-45,1350,-115,26,16,56,30\\n');a='    advance_grab_hold(); finish_gripper_open(); advance_grab_hold(); finish_grab_axes(); advance_grab_hold();';b='''    advance_grab_hold(); unsigned r_count = absolute_commands[1];
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 30 && absolute_commands[1] == r_count);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60 && absolute_commands[1] == r_count);
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();''';assert a in s;s=s.replace(a,b);p.write_text(s,encoding='utf-8')
