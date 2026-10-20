# 当前串口接线（2026-10-18）

| 外设端 | STM32 端 | 参数 |
| --- | --- | --- |
| ESP32 GPIO17 TX | PB12 UART5_RX，AF14 | 115200，8N1 |
| ESP32 GPIO18 RX | PB13 UART5_TX，AF14 | 115200，8N1 |
| JY901S TX | PB11 USART3_RX，AF7 | 9600，8N1 |
| JY901S RX | PB10 USART3_TX，AF7 | 9600，8N1 |
| ESP32/JY901S GND | STM32 GND | 共地 |

JY901S VCC 使用用户当前的 5V 电源，目标为 10Hz 角度和角速度输出。UART7/PF6/PF7 已移除初始化及接收处理。

STM32 中 RobotControl 使用 huart5，ImuLink 使用 huart3，ImuLink_SetBaud 只允许 huart3。两路中断在 MSP 配置：USART3 优先级 1，UART5 优先级 5；上层 RobotControl 不再硬编码 USART3 的中断设置。

ESP32 仍使用 GPIO17/18、115200，协议帧不变，无需为换线更新 ESP32 固件。App 只更新接线与错误提示。

必须烧录本次生成的 STM32 固件后，新接线才会生效。本次未自动烧录。新固件的 UART 基地址和 RAM 符号位置可能变化，调试时应使用当前 ELF 定位，不能复用旧固件的诊断地址。

已通过 STM32 构建、初始化/帧解析模拟和底盘控制回归；实际通信需要烧录后验证。
