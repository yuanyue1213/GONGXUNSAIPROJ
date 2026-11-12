# 底座0度测试（上电执行）

使用`BaseTest`预设，烧录`build/BaseTest/yundongv2.hex`。上电仅启动PC6/TIM3_CH1底座PWM，输出0度（500微秒/20毫秒）并保持；夹爪和转盘PWM不启动，不执行底盘、伸缩、升降或视觉任务，不等待App。VS Code选择“STM32 · 底座0°测试（上电执行）”，若停在main需继续运行。角度在`robot_config.h`的`BASE_TEST_ANGLE`中。测试后烧回所需任务固件。

# 转盘0度测试（上电执行）

使用`TurntableTest`预设，烧录`build/TurntableTest/yundongv2.hex`。上电仅启动PA8/TIM1_CH1的转盘PWM，输出0度（500微秒/20毫秒）并持续保持；夹爪和底座PWM不启动，不初始化电机业务、不发串口运动命令、不等待App。VS Code选择“STM32 · 转盘0°测试（上电执行）”，若调试停在main需继续运行。测试角度在`robot_config.h`的`TURNTABLE_TEST_ANGLE`中。测试后烧回所需比赛固件。

# 固定任务码屏幕测试（上电显示）

使用`ScreenTest`预设，烧录`build/ScreenTest/yundongv2.hex`。上电只显示123+321+132+312，不初始化电机、舵机PWM或串口任务，不需要App。VS Code选择“STM32 · 固定任务码屏幕测试（上电显示，无电机）”。接线和进度布局见[屏幕说明](../../../docs/TASK_DISPLAY.md)。

# 向前300cm测试（App启动）

使用`ForwardTest`预设，烧录`build/ForwardTest/yundongv2.hex`。VS Code选择“STM32 · 向前300cm测试（App启动）”，F5构建并下载；调试器停在main时继续运行。

保持现有ESP固件和App，手机连接`RobotCar-ESP32S3`后点击启动。上电不自动运动，无需PC、雷达、IMU或相机，不启动机械臂和舵机任务。本预设只编译独立测试逻辑，不替换比赛流程。

每次START先停车、使能四轮，再以200RPM、20档加速度（约84.75RPM/s）发送四条FD相对位置命令，通过FF同步触发，向前移动3000mm。加减速由驱动器完成，主段理论耗时约6.03秒，额外等待50ms稳定后停车。短行程自动按三角形速度曲线估时。

本测试与全流程共用13064脉冲/米：根据300cm指令实测289cm，将原12585乘以300/289；3000mm对应每轮39192脉冲，等待时间按新脉冲数重算。前进、后退共用此标定，横移系数仍为13228脉冲/米。

本测试不初始化或配置IMU，不采样航向，不记录横偏，不执行回正或横移补偿，也不读取电机位置。距离由下发的脉冲数控制，STM按预计耗时等待，未轮询实际到位。命令失败时立即停车并上报`COMMAND_FAILED`；正常停车后上报DONE。再次点击可重新执行，不自动循环。

本版用于单独观察200RPM、20档加速度的位置运动表现；不能仅凭一次轨迹判定偏航根因。参数集中在`App/Inc/robot_config.h`的`FORWARD_TEST_*`项。

```powershell
cmake --preset ForwardTest
cmake --build --preset ForwardTest
```

测试完成后烧回Debug、FixedQr或所需比赛固件。

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
