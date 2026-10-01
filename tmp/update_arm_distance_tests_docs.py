from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ')
p=root/'yundong_part/tests/distance_control_test.c'
s=p.read_text(encoding='utf-8')
s=s.replace('static RobotArmAngleCommand last_arm_angle;', 'static struct { char direction; uint32_t angle_tenths, speed_rpm, pulses_per_revolution; } last_arm_angle;')
s=s.replace('RobotArmAngleCommand cmd;', 'RobotArmDistanceCommand cmd;').replace('RobotProtocol_ParseArmAngle', 'RobotProtocol_ParseArmDistance').replace('RobotProtocol_ArmAnglePulses(&cmd) == 800', 'RobotProtocol_ArmDistanceAngle(&cmd) == 3600')
s=s.replace('"ARM,', '"ARM_MOVE,').replace(',E,900,', ',E,127,').replace(',F,900,', ',F,127,').replace(',C,900,', ',C,127,').replace(',C,300,', ',C,10,').replace(',E,36001,', ',E,1001,')
s=s.replace('last_arm_angle.angle_tenths == 900', 'last_arm_angle.angle_tenths == 3600')
s=s.replace('test_arm_angle_control','test_arm_distance_control')
s=s.replace('CMD,10,U\\nCMD,11,E\\n', 'CMD,10,U\\nARM_MOVE,11,E,127,10,3200\\n').replace('assert(arm_stops == 1);\n    reset(); feed("MOVE', 'assert(arm_stops == 0);\n    reset(); feed("MOVE')
# Removed continuous commands must not move the front/rear axis.
s=s.replace('    reset(); feed("ARM_MOVE,1,E,127,10,3200\\n");\n    assert(arm_angle_commands == 1', '    reset(); feed("CMD,9,E\\nCMD,10,C\\nARM,11,E,900,10,3200\\n");\n    assert(arm_angle_commands == 0 && arm_stops == 0);\n    feed("ARM_MOVE,1,E,127,10,3200\\n");\n    assert(arm_angle_commands == 1',1)
p.write_text(s,encoding='utf-8')
p=root/'yundong_part/application/app/src/test/java/com/example/app/RobotTcpClientTest.kt'
s=p.read_text(encoding='utf-8').replace("sendArmAngle('E', 900", "sendArmDistance('E', 127").replace("sendArmAngle('C', 300", "sendArmDistance('C', 10").replace('startsWith("ARM,")','startsWith("ARM_MOVE,")').replace(',E,900,10,3200',',E,127,10,3200').replace(',C,300,5,6400',',C,10,5,6400')
p.write_text(s,encoding='utf-8')
p=root/'tmp/test_esp_servo_forwarding.py'
s=p.read_text(encoding='utf-8').replace('int64_t lift = 0, fore = 0;', 'int64_t lift = 0;').replace('handle_frame(commands[i], &lift, &fore)', 'handle_frame(commands[i], &lift)').replace('ARM,1577625095,E,900,10,3200','ARM_MOVE,1577625095,E,127,10,3200').replace('    assert(current_fore_aft == ARM_FORE_AFT_HOLD);\n','')
p.write_text(s,encoding='utf-8')
p=root/'yundong_part/esp32s3/PROTOCOL.md'
s=p.read_text(encoding='utf-8')
a=s.index('### 前后步进电机定角测试')
b=s.index('### 固定状态',a)
s=s[:a]+'''### 机械臂前后定距移动

按实测电机一圈 360° = 12.7 cm = 127 mm，改为前伸/后收定距。
App 删除前后按住遥控和定角测试，保留距离、速度、每圈脉冲输入和停止按钮。

```text
ARM_MOVE,<seq>,<E或C>,<distance_mm>,<speed_rpm>,<pulses_per_rev>\\n
ARM_MOVE,101,E,127,10,3200\\n
```

示例为前伸 127 mm，10 RPM，即电机转一圈。距离是整数 1–1000 mm；
速度 5–60 RPM；每圈脉冲 200–51200，默认 3200（1.8°、16 细分，须匹配实机）。
E 前伸，C 后收；Q 仍是停止前后轴。

STM32 固定换算：`转角(0.1°) = round(距离mm × 3600 / 127)`。
Emm 再换算为微步脉冲，X 直接使用 0.1°角度，均采用 FD 位置模式 02，
相对当前实时位置移动。角度及微步取整会带来小量化误差。
127 mm 对应 360°、3200 脉冲；63 mm 对应约 178.6°。

每次命令只发送一次，无应用回包、无前后轴续期超时。
STM32 每隔至少 50 ms 查询 ID2 到位/故障，运行中忽略新的 ARM_MOVE，
最近尝试执行的相同序号不重放。`CMD,seq,Q` 可打断；停止失败持续重试。
状态通信失败、失能、堵转、超过 180 秒请求停止；连接建立/断开仍请求停止。
原定角 ARM 命令、连续前后 CMD E/C 均已移除。
距离范围是软件参数限制，并不表示机构具有 1000 mm 行程。
请按实测行程输入，默认先测试 10 mm；App 无到位回包，停稳后再发送下一条。

'''+s[b:]
s=s.replace('| CMD,seq,E/C/Q | 伸出/收回/停止 |','| ARM_MOVE,seq,E/C,distance_mm,rpm,pulses_per_rev | 前伸/后收定距 |\n| CMD,seq,Q | 停止前后轴 |')
s=s.replace('升降/伸缩仍按住每 50 ms 发新序号，松手发送 H/Q；分别采用 ESP32 250 ms、\nSTM32 300 ms 超时，定距移动不需要续期，也不给机械臂续期。速度仍为 30 RPM。','升降仍按住每 50 ms 发新序号，松手发送 H；采用 ESP32 250 ms、STM32 300 ms 超时，\n升降速度为 30 RPM。底盘和前后定距不需要续期，也不给升降续期。')
p.write_text(s,encoding='utf-8')
p=root/'yundong_part/esp32s3/hello_world/README.md'
s=p.read_text(encoding='utf-8').replace('机械臂两轴保留 ESP32 250 ms 续期超时', '只有升降保留 ESP32 250 ms 续期超时，前后使用 ARM_MOVE 定距命令')
p.write_text(s,encoding='utf-8')
p=root/'yundong_part/stm32h743/zhukong/Core/Src/robot_control.c'
s=p.read_text(encoding='utf-8').replace('300 ms 续期保护仅用于机械臂两轴','300 ms 续期保护仅用于机械臂升降').replace('升降、伸缩仍分别检查 300 ms 续期','升降仍检查 300 ms 续期，前后轴为自主定距')
p.write_text(s,encoding='utf-8')
