from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8').replace('== 263','== 248').replace('== 201','== 194').replace('== 202','== 194');lines=s.splitlines();lines=[line.replace(',61,',',121,').replace(',20,20,61',',20,20,121') if any(key in line for key in ['ARM_MOVE,','LIFT_ANGLE,','LIFT_MOVE,','STATE,']) else line for line in lines];s='\n'.join(lines)+'\n'
mark='static void test_absolute_pose(void)';test='''static void test_manual_gripper(void)
{
    RobotGripperCommand cmd;
    assert(RobotProtocol_ParseGripper("GRIP,1,0,60,30", &cmd));
    assert(!RobotProtocol_ParseGripper("GRIP,1,0,271,30", &cmd));
    assert(!RobotProtocol_ParseGripper("GRIP,1,0,60,0", &cmd));
    reset(); feed("GRIP,800,0,60,30\\n"); assert(servo_angle == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 30);
    feed("CMD,801,A\\n"); assert(absolute_commands[1] == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60);
    unsigned servos = servo_commands; feed("GRIP,800,0,60,30\\n"); assert(servo_commands == servos);
    feed("GRIP,802,60,0,60\\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 30);
    feed("GRIP_STOP,803\\n"); servos = servo_commands;
    tick += 1000; RobotControl_Tick(); assert(servo_commands == servos && servo_angle == 30);
    feed("GRIP,804,10,10,30\\n"); assert(servo_angle == 10);
    servos = servo_commands; tick += 1000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("GRIP,805,0,60,30\\nSERVO,806,G,15\\n");
    tick += 2000; RobotControl_Tick(); assert(servo_angle == 15);
    feed("GRIP,807,0,60,30\\nCMD,808,Z\\n");
    servos = servo_commands; tick += 2000; RobotControl_Tick(); assert(servo_commands == servos);
    reset(); feed("ARM_POSE,809,0,10,-10,120,3200\\n"); assert(last_arm_angle.speed_rpm == 120);
    finish_grab_axes(); assert(last_lift_angle.rpm == 120);
}
''';assert mark in s;s=s.replace(mark,test+mark).replace('    test_absolute_pose();','    test_absolute_pose();\n    test_manual_gripper();');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/tests/arm_angle_driver_test.c');s=p.read_text(encoding='utf-8');mark='    puts("PASS:';pos=s.index(mark);s=s[:pos]+'''    reset(0x21);
    assert(LiftMotor_MoveAngle('D', 900, 120, 3200) == HAL_OK);
    assert(position[3] == 0 && position[4] == 120);
    assert(LiftMotor_MoveAngle('D', 900, 121, 3200) != HAL_OK);
    reset(0x25);
    assert(ArmMotor_MoveAngle('E', 900, 120, 3200) == HAL_OK);
    assert(position[7] == 4 && position[8] == 0xB0); /* X speed is RPM * 10. */
'''+s[pos:];p.write_text(s,encoding='utf-8')
p=Path('yundong_part/application/app/src/test/java/com/example/app/SequenceSettingsTest.kt');s=p.read_text(encoding='utf-8').replace('"20", "20", "61"','"20", "20", "121"');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/application/app/src/test/java/com/example/app/RobotTcpClientTest.kt');s=p.read_text(encoding='utf-8').replace('List(19)','List(21)');a='                client.sendConfiguredAlignment(2, 9889, 12000, AlignmentSettings(10, 20, 0))';s=s.replace(a,a+'\n                client.sendGripper(0, 271, 30)\n                client.sendGripper(0, 60, 30)\n                client.stopGripper()');a='                assertTrue(frames[18].startsWith("ALIGN_CFG,") && frames[18].endsWith(",2,9889,12000,10,20,0"))';s=s.replace(a,a+'\n                assertTrue(frames[19].startsWith("GRIP,") && frames[19].endsWith(",0,60,30"))\n                assertTrue(frames[20].startsWith("GRIP_STOP,"))');p.write_text(s,encoding='utf-8')
p=Path('tmp/test_esp_servo_forwarding.py');s=p.read_text(encoding='utf-8').replace('"ALIGN_CFG,1577625123,2,9889,12000,12,30,-7",','"ALIGN_CFG,1577625123,2,9889,12000,12,30,-7", "GRIP,1577625124,0,60,30", "GRIP_STOP,1577625125",');s=s.replace('"STATE,1,P,200,-40,1300,-110,20,20,61"','"STATE,1,P,200,-40,1300,-110,20,20,121"');p.write_text(s,encoding='utf-8')
