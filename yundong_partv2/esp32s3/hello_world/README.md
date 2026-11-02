# ESP32-S3雷达和控制桥

`main/start_bridge.c`合并N10 UART2→UDP3334、App TCP3333→STM32、PC TCP3335→STM32。热点RobotCar-ESP32S3，密码robotcar123，同时支持PC和手机。雷达RX16/TX15，STM32 TX17/RX18。

在ESP-IDF终端使用 `idf.py -B build_route build`，烧录 `idf.py -B build_route -p COMx flash`。输出 `build_route/robot_start_bridge.bin` 是应用镜像，完整烧录须使用构建目录flash_args或idf.py flash。

PC运行 `python F:/GONGXUNSAIPROJ/leida/n10_wifi.py --robot --zone 1`，再从App启动。参见 [PC导航说明](../../docs/PC_NAVIGATION.md)。
