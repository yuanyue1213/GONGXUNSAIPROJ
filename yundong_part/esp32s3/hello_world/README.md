# ESP32-S3 手机遥控接收端

此工程使用 ESP-IDF。协议见 [PROTOCOL.md](../PROTOCOL.md)。

1. 在 ESP-IDF 环境中进入本目录，执行 `idf.py set-target esp32s3`，然后执行
   `idf.py build`、`idf.py -p <串口> flash monitor`。
2. 手机上连接 `RobotCar-ESP32S3`，密码 `robotcar123`。手机可能提示热点没有
   互联网，保持连接即可。
3. TCP 连接 `192.168.4.1:3333`。连接成功会收到 `HELLO,1`；发送
   `CMD,1,F\n` 会收到 `ACK,1,F\n`。应答仅确认 ESP32-S3 接收成功。

当前版本不控制电机。运动方向只记录在 ESP32-S3 内部状态及串口日志中；
断连或 250 ms 未收到有效命令后恢复停止。下一步接入 STM32 UART 和手机 App。

Wi-Fi 名称、密码、TCP 端口及超时参数位于
`main/robot_remote_main.c` 文件顶部。调试时请先修改默认热点密码。
