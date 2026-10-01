from pathlib import Path
root = Path('F:/GONGXUNSAIPROJ/yundong_part/esp32s3')
p = root / 'PROTOCOL.md'
s = p.read_text(encoding='utf-8')
a = s.index('TCP 支持拆包/粘包')
b = s.index('序号为', a)
s = s[:a] + 'TCP 支持拆包/粘包；命令单帧最多 63 字节，不包含换行。\n协议只发送命令，ESP32 和 STM32 不返回 HELLO、ACK、EXEC、DONE、ERR、EVENT 或诊断帧。\nTCP 自身的传输确认不属于应用层回包。\n' + s[b:]
a = s.index('MOVE 只发一次')
b = s.index('## App 标定参数', a)
s = s[:a] + '''MOVE 只发一次，无 KEEP 续期，也不等待 ESP32/STM32 执行确认。
App 已删除 2 秒确认超时，不会因缺少回包自动发送 S。
App 的“已发送”只表示写入 TCP 成功，不能确认电机执行或到位。

STM32 每 50 ms 查询一个电机，一轮约 200 ms。四轮均使能、无堵转/堵转保护，
且 3A 状态 bit1 均置位才在本地结束任务。运动或停车重试期间的新 MOVE 直接忽略，
不排队，也不打断当前任务；手动 CMD,S 可以打断。请等小车停稳后再发下一条 MOVE。
STM32 记录最近一次尝试执行的 MOVE 序号，重复序号直接忽略，避免丢失电机 ACK 后重放。
这不是所有历史序号的永久去重。断线前的任务不自动重发。

''' + s[b:]
a = s.index('## 停止与错误')
b = s.index('USART3 使用', a)
s = s[:a] + '''## 停止与本地保护

`CMD,<seq>,S\\n` 取消底盘任务，分别向四轮发送 `ID FE 98 00 6B`。
停车失败持续重试，期间忽略 MOVE。连接建立/断开、App 后台/退出仍请求三轴停止。
定距任务超过 30 分钟，或电机应答/状态通信失败、失能、堵转时，STM32 尝试停车。
非法帧、超长帧直接丢弃；上述结果都不回传给 App。
移除的是 App、ESP32、STM32 之间的应用层回包；电机驱动器的 ACK 和状态查询保留。

''' + s[b:]
s = s.replace('## 保留的机械臂、舵机与诊断', '## 保留的机械臂与舵机').replace('| DIAG,seq | USART2 电机 ID1/2 通信、固件、状态诊断 |\n', '')
a = s.index('成功回 ACK')
b = s.index('## 接线与验证', a)
s = s[:a] + 'USART2 驱动仍识别 Emm/X 固件。DIAG 命令及 App 检测按钮已删除。\n\n' + s[b:]
s = s.replace('| GPIO18 RX | PB10 USART3_TX | 执行/到位回包 |', '| GPIO18 RX | PB10 USART3_TX | 当前不使用，可保留原接线 |')
a = s.index('`RobotTcpClientTest.kt`')
s = s[:a] + '''`RobotTcpClientTest.kt` 使用本地 TCP 服务验证完全没有回包时，超过旧的 2 秒确认期限
仍不自动发停止，且可以手动停止和发送下一条 MOVE。
`../tests/distance_control_test.c` 用模拟 HAL 编译真实驱动/控制代码，验证四向
57 字节帧、四轮完成、忙碌/重复序号、无续期自主移动、总时限、机械臂超时、故障、
丢 ACK、停车重试，并断言 STM32 命令串口从不发送应用层回包。
这些不能代替实车验证。三端须一起更新。先试 100 mm 核对方向，再测量和标定距离。
App 保存最近 16 条本地发送记录，可长按复制。没有回包时无法区分未执行、
运动中、到位或故障停车；实际距离仍需观察和测量。
'''
p.write_text(s, encoding='utf-8')
(root / 'hello_world/README.md').write_text('''# ESP32-S3 手机遥控接收端

此工程使用 ESP-IDF。协议见 [PROTOCOL.md](../PROTOCOL.md)。

1. 在 ESP-IDF 环境进入本目录，执行 `idf.py set-target esp32s3`、`idf.py build`，
   再执行 `idf.py -p <串口> flash monitor`。
2. 手机连接热点 `RobotCar-ESP32S3`，密码 `robotcar123`，TCP 地址 `192.168.4.1:3333`。
3. 发送 `MOVE,1,F,100,45,9889\\n` 发起前进 100 mm；`CMD,2,S\\n` 请求停止底盘。

命令单向转发，无 HELLO/ACK/EXEC/DONE/ERR/诊断回包。App 不等待执行确认，
不因缺少回包自动停车，只显示本地发送状态。请等小车停稳后再发送下一条 MOVE。
STM32 本地维护任务忙碌、到位和故障保护；电机底层 ACK 和状态查询仍保留。

UART1 GPIO17 TX 接 STM32 USART3 PB11 RX，共地，115200-8N1。
GPIO18 RX 到 PB10 TX 的原接线可以保留，目前不读取应用回包。
定距无 KEEP 心跳。机械臂两轴保留 ESP32 250 ms 续期超时，断线请求三轴停止。
Wi-Fi 名称、密码、端口和超时参数在 `main/robot_remote_main.c` 顶部。
''', encoding='utf-8')
