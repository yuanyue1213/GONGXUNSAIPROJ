from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');mark='static void test_absolute_pose(void)';test='''static void test_sequence_angles(void)
{
    RobotSequenceCommand cmd;
    assert(RobotProtocol_ParseSequence("STATE,900,P,200,-40,1300,-110,20,20,50,40,30,100,20,250,130", &cmd));
    assert(cmd.theta == 30 && cmd.base_home == 250 && cmd.open_angle == 100);
    assert(!RobotProtocol_ParseSequence("STATE,900,P,200,-40,1300,-110,20,20,50,40,271,100,20,250,130", &cmd));
    reset(); feed("STATE,900,P,200,-40,1300,-110,20,20,50,40,30,100,20,250,130\\n");
    assert(servo_history[0] == 30 && servo_history[1] == 100 && servo_angle == 250);
    finish_grab_axes(); advance_grab_hold(); tick += 1000; RobotControl_Tick(); assert(servo_angle == 190);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 130);
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 100);
    finish_z_first_axes(); assert(servo_angle == 20 && servo_channel == 'G');
    advance_grab_hold(); finish_z_first_axes(); advance_grab_hold();
    tick += 2000; RobotControl_Tick(); assert(servo_angle == 250);
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 100);
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    assert(servo_history[servo_commands-3] == 30);
}
''';assert mark in s;s=s.replace(mark,test+mark).replace('    test_absolute_pose();','    test_absolute_pose();\n    test_sequence_angles();');p.write_text(s,encoding='utf-8')
for name in ['SequenceSettingsTest.kt','RobotTcpClientTest.kt']:
 p=Path('yundong_part/application/app/src/test/java/com/example/app')/name;s=p.read_text(encoding='utf-8').replace('20,20,50,60\\n','20,20,50,60,0,60,0,248,140\\n').replace(',A,1100,-50,200,-40,20,20,50,60"',',A,1100,-50,200,-40,20,20,50,60,0,60,0,248,140"').replace(',P,200,-40,1300,-110,25,15,55,60"',',P,200,-40,1300,-110,25,15,55,60,0,60,0,248,140"');p.write_text(s,encoding='utf-8')
p=Path('tmp/test_esp_servo_forwarding.py');s=p.read_text(encoding='utf-8').replace('"GRIP_STOP,1577625125",','"GRIP_STOP,1577625125", "STATE,1577625126,P,200,-40,1300,-110,120,120,120,40,30,100,20,250,130",');p.write_text(s,encoding='utf-8')
for path in ['output/目前已有状态与扩展说明.md','yundong_part/esp32s3/PROTOCOL.md']:
 p=Path(path);s=p.read_text(encoding='utf-8').replace('263','248').replace('5–60 RPM','5–120 RPM').replace('5–60）','5–120）');s+='''
App 舵机角度改为数字输入并点击设置。基座启动、回零及自动流程默认初始位为 248°。
机械臂伸缩/升降位置指令及自动流程速度上限提高至 120 RPM，默认速度不变；位置修正速度上限仍为 60 RPM。
新增独立夹子开合：起始/结束角度 0–270°、速度 6–300 度/秒；开始时设置起始角度，随后平滑走到结束角度。停止保留当前目标角度。新指令优先取消正在执行的抓取/放下和机械臂绝对定位。
抓取、放下分别可设置转盘角度、夹子张开/夹紧角度、基座初始/翻转位。基座渐变仍为2秒；夹子张开根据设定角度差与度/秒计算时长。抓取结束的转盘120°/240°/0°循环沿用原逻辑。
''';p.write_text(s,encoding='utf-8')
p=Path('yundong_part/esp32s3/PROTOCOL.md');s=p.read_text(encoding='utf-8');s+='''
独立夹子帧：`GRIP,seq,startAngle,endAngle,degreesPerSecond\\n`，停止 `GRIP_STOP,seq\\n`。
STATE 帧可在夹子速度之后附加五个角度：`STATE,seq,A/P,r1,z1,r2,z2,rRPM,upRPM,downRPM,gripperDps,theta,openAngle,closeAngle,baseHome,baseTilt\\n`。旧帧使用默认角度0/60/0/248/140。无回包，帧上限127字节。
''';p.write_text(s,encoding='utf-8')
