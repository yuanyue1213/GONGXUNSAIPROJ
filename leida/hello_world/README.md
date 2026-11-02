# 合并ESP32-S3固件入口

本目录CMake直接编译 `yundong_partv2/esp32s3/hello_world/main/start_bridge.c`，与运动工程的ESP入口共用源码；旧独立雷达桥已移除。

雷达RX16/TX15，UART2 230400；STM32使用UART1 TX17/RX18 115200。热点RobotCar-ESP32S3，密码robotcar123；UDP3334、App TCP3333、PC TCP3335。

ESP-IDF终端运行 `idf.py build`、`idf.py -p COMx flash`。默认目标ESP32-S3。操作见 [PC导航说明](../../yundong_partv2/docs/PC_NAVIGATION.md)。
