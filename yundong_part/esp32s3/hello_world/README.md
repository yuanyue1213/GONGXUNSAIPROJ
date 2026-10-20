# ESP32-S3 手机遥控接收端

此工程使用 ESP-IDF。协议见 [PROTOCOL.md](../PROTOCOL.md)。

2026-10-18：已接入JY901S PD闭环转向，IMU接STM32 UART7 PF6/PF7，配置115200并输出角度/角速度，建议50Hz。手动定角度和全流程须同步更新三端；全流程启动自动IMU_ZERO，转向使用HEADING，0.96不再参与闭环转角。详见[JY901S接线与使用](../../shared/JY901S_TURN_CONTROL.md)。

起步修订：默认 `(2250,2200) → NE(2200,2200)` 直接右平移50mm，不转车头；只有该段例外，其余路线及视觉微调仍先转向再前进。

2026-10-17：全流程已改为“原地转向→前进”，导航及视觉微调均不横移、不倒车、不斜行。App全流程页新增轴距、轮距、转向修正和转速参数，上传必需的AUTO_DRIVE帧。App与ESP32须一起更新，STM32与视觉程序保持现有协议。最近构建目录为`build_autonomous`，使用`idf.py -B build_autonomous -p <串口> flash monitor`部署，避免误烧其他build目录中的旧固件。

新增一键自动任务：App“全流程”页独立输入路线、视觉与机械臂参数，选择视觉/机械臂开关后启动S1→两批搬运→S2，不依赖调试卡片。两项都关闭时只进行底盘运动。ESP32读取STM32的动作反馈并调度视觉请求；旧手动命令仍不等待反馈。部署及双向接线见 [自动任务协议](../../shared/AUTONOMOUS_TASK_PROTOCOL.md)。下文单向说明仅指旧手动命令。

1. 在 ESP-IDF 环境进入本目录，执行 `idf.py set-target esp32s3`、`idf.py build`，
   再执行 `idf.py -p <串口> flash monitor`。
2. 手机连接热点 `RobotCar-ESP32S3`，密码 `robotcar123`，TCP 地址 `192.168.4.1:3333`。
3. 发送 `MOVE,1,F,100,45,9889\n` 发起前进 100 mm；`CMD,2,S\n` 请求停止底盘。

命令单向转发，无 HELLO/ACK/EXEC/DONE/ERR/诊断回包。App 不等待执行确认，
不因缺少回包自动停车，只显示本地发送状态。请等小车停稳后再发送下一条 MOVE。
STM32 本地维护任务忙碌、到位和故障保护；电机底层 ACK 和状态查询仍保留。

UART1 GPIO17 TX 接 STM32 USART3 PB11 RX，共地，115200-8N1。
GPIO18 RX 到 PB10 TX 的原接线可以保留，目前不读取应用回包。
定距无 KEEP 心跳。只有升降保留 ESP32 250 ms 续期超时，前后使用 ARM_MOVE 定距命令，断线请求三轴停止。
Wi-Fi 名称、密码、端口和超时参数在 `main/robot_remote_main.c` 顶部。


位置修正通过 `ALIGN,seq,forward_ppm,lateral_ppm` 转发至 STM32，ESP32 不直接连接摄像头。UART4 接线、坐标换算与流程见 [位置修正文档](../../shared/CAMERA_ALIGNMENT_PROTOCOL.md)。需要同时更新 App、ESP32 和 STM32。
