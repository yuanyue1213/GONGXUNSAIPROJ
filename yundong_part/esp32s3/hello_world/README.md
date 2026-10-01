# ESP32-S3 手机遥控接收端

此工程使用 ESP-IDF。协议见 [PROTOCOL.md](../PROTOCOL.md)。

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
