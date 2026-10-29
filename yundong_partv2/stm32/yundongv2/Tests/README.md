# STM32到相机串口测试

使用CMake的 `CameraUartTest` 配置并构建，烧录 `build/CameraUartTest/yundongv2.hex`。

VS Code运行和调试下拉框也可选择“STM32 · 相机串口测试（上电发送）”，F5会构建并下载正确的测试固件。如调试器停在main，按F5继续运行；下载后脱离调试器重新上电则直接发送。普通“STM32H743 · ST-Link Debug”配置加载的是比赛固件，不是本测试。

```powershell
cmake --preset CameraUartTest
cmake --build --preset CameraUartTest
```

上电后不需要点击App，UART4直接发 `CAM,1,QR\n`，其中 `\n` 为真实LF字节。未收到扫码结果时每秒补发同一编号；收到结果后回复 `ACK,1\n` 并等待，不再主动请求扫码。相机显示 `ACK RECEIVED / WAIT NEXT COMMAND`。重新复位STM32开始下一次测试时，相机也需重新运行，避免同编号已确认请求被忽略。测试不执行比赛初始化，不启动舵机PWM、不配置IMU、不发电机命令。

接线：STM32 PA0 TX接相机UART2 B1 RX；相机B0 TX接STM32 PA1 RX；两端GND共地。115200、8N1。相机运行当前main.py。

新版测试把UART4内核时钟切到内部HSI 64MHz，重新初始化UART4的115200波特率，用于排除HSE晶振配置差异。USART2/3/UART4/5共享的内核时钟选择只在此独立测试中切换，其他串口不参与测试；普通比赛固件不受影响。调试器中camera_uart_test_clock_hz应为64000000。若PA0波形仍需核对，115200波特率单个位宽约8.68微秒。

- 正常：相机RX持续增加，LAST变为QR#1，显示USB摄像头画面。
- 相机RX不增加：先确认烧录的是测试目录下的固件，再检查PA0到B1方向。
- LAST已经是QR#1，显示USB打开/读帧失败：请求已传到相机，再检查USB摄像头。
- 调试器可观察camera_uart_test_count：0为尚未收到二维码，1为已收到并尝试确认。camera_uart_test_status表示初始化/请求是否成功；HAL_OK不表示对方一定收到了。

测试完成后重新烧录普通Debug或Release固件，恢复完整比赛流程。普通预设默认关闭测试。
