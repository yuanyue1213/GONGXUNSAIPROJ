# N10 雷达采集、无线转发与监控

本目录实现 N10 二维雷达的完整链路：STM32H723 从雷达读取数据，使用 UART5 将校验后的数据转发给 Seeed XIAO ESP32-C3；ESP32 将每一圈点云通过 Wi-Fi/UDP 发给 PC，Python 程序显示点云。

```text
N10 雷达 --UART4/DMA--> STM32H723 --UART5/DMA--> XIAO ESP32-C3
                                                    |
                                             Wi-Fi / UDP
                                                    |
                                                   PC
```

## 目录

```text
leida_part/
├── stm32/                 STM32H723 工程
├── esp32/                 ESP-IDF 6 的 XIAO ESP32-C3 工程
└── lidar_monitor.py       PC 点云图形监控程序
```

## 雷达与 STM32 接线

N10 使用 5 V 供电、3.3 V TTL UART，串口参数为 `230400, 8N1`。雷达工作时电流约 200 mA，建议 5 V 电源至少能够稳定提供 500 mA。

```text
N10 雷达                  STM32H723
红线 VCC   ------------->  5V
黑线 GND   ------------->  GND
黄线 TX    ------------->  PA1 / UART4_RX
绿线 RX    <-------------  PA0 / UART4_TX（目前可不接）
```

不要将 N10 的 VCC 接到 3.3 V。N10、STM32 和 ESP32-C3 的 GND 必须共地。UART 信号为 TTL 电平，不能接 RS232 接口或 RS232 转换器。

STM32 使用 UART4 的 DMA1 Stream0 循环接收。程序会找到 `A5 5A` 帧头、校验 58 字节 N10 原始帧并解码。调试时可观察 `n10_valid_frame_count` 是否持续增长，`n10_invalid_frame_count` 应接近 0。

## STM32 与 XIAO ESP32-C3 接线

STM32 的 UART5 使用 `230400, 8N1`，DMA1 Stream1 非阻塞发送。XIAO ESP32-C3 的开发板丝印与 GPIO 对应关系中，`D7 = GPIO20`、`D6 = GPIO21`。

```text
STM32H723                   Seeed XIAO ESP32-C3
PB13 / UART5_TX  --------->  D7 / GPIO20 / UART1_RX
PB12 / UART5_RX  <---------  D6 / GPIO21 / UART1_TX（可选）
GND               ---------  GND
```

当前只需要单向转发时，至少连接 `PB13 -> D7` 和 `GND -> GND`。源代码中引脚宏位于 `esp32/main/n10_bridge_main.c`：

```c
#define LIDAR_UART_RX_GPIO  20  /* XIAO D7 */
#define LIDAR_UART_TX_GPIO  21  /* XIAO D6 */
```

STM32 中 `esp_uart_tx_frame_count` 持续增加，且 `esp_uart_tx_overrun_count` 与 `esp_uart_tx_error_count` 保持为 0，说明 STM32 到 ESP32 的串口转发正常。

## 烧录 ESP32-C3

工程使用 ESP-IDF 6，目标芯片为 ESP32-C3。先将 XIAO ESP32-C3 用 USB 连接到 PC，打开已配置 ESP-IDF 环境的终端后执行：

```powershell
cd F:\GONGXUNSAIPROJ\leida_part\esp32
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 换成 XIAO 的实际串口号。若 VS Code 找不到 `driver/uart.h`，先执行一次 `idf.py build` 生成 `build/compile_commands.json`，再执行 `Developer: Reload Window`。

烧录成功后，串口日志会显示：

```text
Wi-Fi AP ready: SSID N10-LiDAR, IP 192.168.4.1
UDP subscription port ready: 3333
```

## PC 点云监控

1. PC 连接 ESP32 创建的 Wi-Fi：

   ```text
   SSID: N10-LiDAR
   密码: lidar12345
   ```

2. 运行监控脚本：

   ```powershell
   cd F:\GONGXUNSAIPROJ\leida_part
   python .\lidar_monitor.py
   ```

脚本默认向 `192.168.4.1:3333` 发送订阅消息，ESP32 收到后会将完整一圈雷达数据用 UDP 分包发送回来。窗口以车头向上（0 度）的方式显示点云。默认显示量程 6000 mm，可通过下列命令调整：

```powershell
python .\lidar_monitor.py --range-mm 4000
```

## 常见排查

| 现象 | 优先检查 |
| --- | --- |
| STM32 的有效帧计数不增长 | N10 是否接 5 V、N10 黄线是否接 PA1、是否共地、UART4 是否为 230400。 |
| 有效帧很少或无效帧持续增长 | 检查雷达 TX/RX 是否接反、供电是否稳定、是否误接 RS232 电平。 |
| ESP32 日志没有订阅提示 | PC 是否已连接 `N10-LiDAR`，监控脚本是否运行，防火墙是否拦截 UDP 3333。 |
| 监控窗口没有点 | 检查 STM32 的转发计数；再检查 ESP32 串口接线应为 `PB13 -> D7`，而非 TX-TX。 |
| CMake 提示 D 盘和 F 盘路径冲突 | 删除 `stm32/build/Debug` 后重新配置或重新构建；该目录是可再生的构建缓存。 |
