# 手机遥控调试协议（v1）

## 当前连接方式

ESP32-S3 开启 Wi-Fi 热点，手机连接后通过 TCP 与 `192.168.4.1:3333` 通信。
默认热点名称为 `RobotCar-ESP32S3`，密码为 `robotcar123`。这两个值定义在
`hello_world/main/robot_remote_main.c`；正式使用前应修改密码。

TCP 是字节流，收发双方必须按换行符 `\n` 分帧。一条连接中可以连续发送多帧；
一帧也可能被拆成多次接收。ESP32-S3 允许帧尾使用 `\r\n`。不含换行符的帧不会被执行。
单帧最多 63 字节（不含换行符），各字段使用 ASCII，不包含空格。

## 手机发送

```text
CMD,<seq>,<dir>\n
```

`seq` 是十进制无符号 32 位序号，可从 1 开始递增；`dir` 是一个大写字母：

| `dir` | 含义 |
| --- | --- |
| `F` | 前进 |
| `B` | 后退 |
| `L` | 左转（预定为原地转向） |
| `R` | 右转（预定为原地转向） |
| `S` | 底盘停止 |
| `U` | 机械臂上升，按住期间每 50 ms 重发 |
| `D` | 机械臂下降，按住期间每 50 ms 重发 |
| `H` | 机械臂升降停止 |
| `E` | 机械臂向前（伸出），按住期间每 50 ms 重发 |
| `C` | 机械臂向后（收回），按住期间每 50 ms 重发 |
| `Q` | 机械臂前后停止 |

示例：`CMD,23,F\n`。

可发送 `DIAG,<seq>\n` 读取 USART2 和两个电机的通信状态。
ESP32 回 `ACK,<seq>,DIAG` 只表示转发成功。STM32 收到命令后先回
`STM,DIAG_BEGIN,<seq>`，随后向 ID 1/2 分别发送只读的
驱动参数查询，返回 `STM,DIAG,<seq>,UART2=0/1,ID1=0/1,FW1=E/X/?,S1=xx,ID2=0/1,FW2=E/X/?,S2=xx`。
`E` 表示 Emm 固件，`X` 表示 X 固件；主控会据此选用对应速度帧。
`S1/S2` 是十六进制状态位：bit0 为使能、bit2 为堵转、bit3 为堵转保护；
`FF` 表示状态读取失败（需结合 ID 是否响应判断）。
`UART2=0` 表示串口初始化失败；某个 ID 为 0 表示该电机没有按手册协议响应，
需检查接线、供电、电机地址、通讯模式和波特率。App 会单独保留 STM32 回包。
电机控制失败时，STM32 回 `STM,ERR,<seq>,LIFT_MOTOR,<reason>` 或
`STM,ERR,<seq>,ARM_FORE_AFT_MOTOR,<reason>`；`NO_REPLY` 表示未收到电机回包，
`REJECT_E2` 表示电机因参数或保护条件拒绝，`FORMAT_EE` 表示电机认为帧格式错误。

三个舵机用独立指令设置角度：

```text
SERVO,<seq>,<channel>,<angle>\n
```

`G` 为夹子（PC8，0–270°），`T` 为转盘（PA8，0–270°），
`B` 为基座（PC6，0–360°）。例如 `SERVO,24,G,135\n`。
STM32 输出 50 Hz PWM，并将 0° 和最大角分别映射为 500 µs 和 2500 µs。
舵机在首次收到角度命令前不输出 PWM；断开连接后保持最后的脉宽。

App 按住底盘方向键期间每 50 ms 发送一次新序号命令，松手立即发送 `S`；
按住升降或前后键时同样重发，松手分别发送 `H` 或 `Q`。App 切到后台或主动断开
时发送三种停止命令。ESP32-S3 分别在底盘、升降、前后命令中断 250 ms 后发送
对应停止命令；连接断开时也同时停止三者。
重连后须重新按下方向键，不恢复断线前的运动状态。

## ESP32-S3 回复

新 TCP 连接建立后先发送：

```text
HELLO,1\n
```

每收到一条格式正确的命令，回复：

```text
ACK,<seq>,<dir>\n
```

例如，手机发送 `CMD,23,F\n`，ESP32-S3 回复 `ACK,23,F\n`。
`ACK` 表示 ESP32-S3 已解析命令并成功写入 STM32 的 UART。App 可以用 `seq`
对应请求与回复；它不表示电机已经执行。

无效帧回复 `ERR,0,BAD_FRAME\n`；超长帧回复 `ERR,0,TOO_LONG\n`。
命令超时后，ESP32-S3 主动发送一次 `EVENT,STOP,TIMEOUT\n`。
`EVENT` 是异步消息，App 接收时不能把它当成某条命令的 `ACK`。

## STM32 执行回包

ESP32-S3 会把 STM32 的每条回包转发给手机，格式为：

```text
STM,<STM32 原始回包>\n
```

例如手机发送 `CMD,23,F\n`，会依次收到：

```text
ACK,23,F
STM,EXEC,23,F
```

`STM,EXEC` 表示 STM32 已成功把命令写入四轮电机驱动；
`STM,ERR,<seq>,MOTOR` 表示 STM32 与电机通信失败。STM32 上电后也会发送
`STM,READY`，通信超时停车时发送 `STM,EVENT,STOP,TIMEOUT`。
舵机指令成功时回复 `ACK,<seq>,SERVO,<channel>,<angle>` 和
`STM,EXEC,<seq>,SERVO,<channel>,<angle>`；定时器不可用时返回 `STM,ERR,<seq>,SERVO`。

## ESP32-S3 与 STM32 接线

两端都是 3.3 V UART，115200-8N1：

| ESP32-S3 | STM32H743 | 说明 |
| --- | --- | --- |
| GPIO17（TX） | PB11（USART3_RX） | ESP32 发送命令给 STM32 |
| GPIO18（RX） | PB10（USART3_TX） | STM32 发送执行状态给 ESP32 |
| GND | GND | 必须共地 |

ESP32-S3 的 GPIO17/GPIO18 定义在 `robot_remote_main.c` 文件顶部；若开发板占用了
这两个引脚，可改为其他空闲 GPIO。STM32 在 `USART3` 上使用 PB10/PB11。

底盘、升降和机械臂前后分别计时，互不影响。ESP32 在相应运动命令中断 250 ms
后发送 `S`、`H` 或 `Q`；STM32 在 300 ms 后也独立停止对应运动。
机械臂两个电机接 STM32 USART2（PA2 TX、PA3 RX），波特率 115200：升降 ID 1，
前后 ID 2。它们与 USART1 上的底盘电机处于不同串口。两路速度当前均为 30 RPM；
`lift_motor.c` 中的方向常量可按实际机构调整。机械限位仍需由实际硬件保障。
