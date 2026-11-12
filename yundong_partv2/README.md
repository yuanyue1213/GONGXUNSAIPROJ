# yundong_partv2：顺序执行全流程

视觉校准：原料区仅对图像中心；粗加工、暂存和堆叠直接中心精调，再平行校准，不粗调、不搜索，目标均为(256,160)。识别坐标不加偏移，旧material_offset配置被忽略。每个平行校准步骤按atan(三中心拟合斜率)计算剩余倾角，IMU闭环修正（每次限幅±15°、容差0.05°），最多修正3次，完成或达到3次后直接继续，不追加视觉复核、不因未达标停止。安装比例通过PARALLEL_ANGLE_GAIN调整，默认1.0；实际三次收敛能力需结合相机安装角度验证。平行由STM32使用当前帧计算，不等待连续三帧稳定；中心识别仍由相机判断稳定。任务后先收臂、恢复平行校准前航向，再撤销左右和前后位移；位移保留原车体坐标，不进行旋转换算。`mission_flow.h`在`FLOW_ALIGN`之后写`FLOW_PARALLEL(TASK_RING,批次)`或`FLOW_PARALLEL(TASK_OCCUPIED,批次)`，参数集中在`robot_config.h`的`PARALLEL_*`。需同步更新STM32 Debug固件和`camera/MaixVision/main.py`；相机端测试：`python -B yundong_partv2/tests/parallel_vision_test.py`。

STM32 可调参数统一在 `stm32/yundongv2/App/Inc/robot_config.h`。速度及上限、区域姿态、槽位角度、初始化/收臂角度、夹爪动作时间、视觉校准和搜索、传动比例、转向判定、串口波特率/应答时间、IMU 方向和 PWM 参数均从此处读取。旧 `mission_config.h`、`motion_speed.h` 和 shared 的速度上限头只保留转发包含，不再定义参数。

动作顺序继续在 `mission_flow.h`、`arm_flow.h` 编写。硬件引脚和时钟拓扑由 CubeMX IOC 管理；改 PWM 周期还需同步 IOC 的 ARR，改设备波特率要与外部设备一致。App 和 ESP32 的网络配置不属于此次 STM32 参数集中。

本版本参考 gcs-gold-medal 的顺序调用、预计耗时等待和多轴并行思路，保留本车的 `mission_flow.h`、UART 电机驱动、接线和视觉协议。App只有一键启动，ESP32桥接雷达、PC路径和任务状态，完整流程由STM32执行。当前默认使用PC导航，首次使用见 [自动采样与PC导航](docs/PC_NAVIGATION.md)。

## 当前调用关系

完整模块图和执行说明见 [当前代码架构](docs/ARCHITECTURE.md)。

```text
App → ESP32 → START → start_link.c
                         ↓
                  mission_runner.c
                         ↓ 读取流程表
                  mission_flow.h
                         ↓ 直接调用
                  motion_control.c
                         ↓
          步进 UART 驱动 / 舵机 PWM / IMU

mission_runner.c → CameraLink_Wait() → 识别结果直接返回
TIM6 每 20 ms → 三路舵机五次曲线 → TIM1/TIM3 PWM
```

流程层直接用 for 循环遍历 mission_flow[]，启动前用 Flow_Check() 校验一次，不再维护执行器游标对象。已移除旧的 autonomous_task、robot_control、mission_control、robot_motion、gyro_turn、flow_executor 和内部配置装载模块，以及不再使用的手动命令协议。没有 EXEC/ACTION 字符串动作链、动作反馈队列或运行阶段状态机。

## 遗留功能清理

已按当前全流程调用关系清理业务代码：

- 机械臂仅保留 Emm/X 自动识别、相对角度运动、命令确认和停止；删除连续速度、绝对位置、位置/状态查询、读取重试、错误查询和空故障回调。
- 底盘仅保留使能、FD 同步位置、F6 同步轮速和立即停止；删除旧 AA 打包命令、位置/状态查询、同步停车选项、原始发送公共入口及 SWD 诊断快照。
- IMU 保留启动配置、寄存器校验、角度/角速度接收和有效性判断；删除诊断计数器、状态查询和无调用方的样本序号。
- 删除舵机 Busy 查询、未使用的加速度配置、预置二维码模式、MAT 视觉模式及未使用的 CAM 入站解析。现在任务码必须来自扫码。
- App 保留一键启动和状态显示，删除旧 CALIBRATING 状态分支、示例测试及无使用位置的预览/仪器测试依赖。ESP32 已是纯桥接代码。

保留 `mission_flow.h`、8段下载导航、8 个站点步骤、24 次取放、视觉对准和既有速度。HAL/CMSIS、CubeMX 外设初始化及正常收发所需的错误处理属于当前运行依赖。

## 目录与修改入口

```text
application/                         Android：启动按钮、TCP 和状态显示
esp32s3/hello_world/                  ESP32：WiFi 热点、TCP/UART 桥接
shared/                              启动/视觉协议、速度边界、舵机和轮组数学
stm32/yundongv2/
  App/Inc/mission_flow.h             完整流程表
  App/Inc/arm_flow.h                 机械臂姿态、取放/收臂/初始舵机动作表
  App/Inc/arm_types.h                小动作类型、初始化宏和执行接口声明
  App/Src/arm_flow.c                 小动作顺序执行循环
  App/Inc/flow_types.h               流程操作和初始化宏
  App/Inc/mission_config.h           距离标定、图像方向、预计耗时参数
  App/Inc/motion_speed.h             默认速度
  App/Src/flow_check.c               无状态流程表校验
  App/Src/mission_runner.c           顺序解释流程、站点取放和相机等待
  App/Src/motion_control.c           直接运动函数与预计耗时等待
  App/Src/start_link.c               UART5 启动入口、状态发布及中断分发
  App/Src/camera_link.c              相机收发、等待及结果直接返回
  App/Src/imu_link.c                 IMU 配置及姿态接收
  App/Src/servo_control.c            PWM 和 TIM6 三通道轨迹更新
  App/Src/servo_trajectory.c         单通道轨迹计算
  App/Src/lift_motor.c               USART2 的 Emm/X 机械臂协议
  App/Src/zdt_motor*.c               USART1 底盘协议及接收缓冲
  Core/                             CubeMX 外设和中断入口
  Drivers/                          HAL / CMSIS
  cmake/                            编译工具链及 HAL 源文件清单
tests/                              当前顺序架构的主机回归测试
```

## 如何改流程

视觉校准现已显式编写为 `FLOW_ALIGN(视觉模式,批次)`，站点不再自动校准。取放或自定义机械臂动作之后安排 `FLOW_STOW()`、`FLOW_UNALIGN()`，先恢复平行校准前的车头方向（执行过平行转动时，10 RPM上限、0.05°容差），再抵消包含搜索在内的左右、前后校准净位移。详见 [视觉校准与撤销](docs/VISION_ALIGNMENT.md)。中心校准和返回的速度上限均为 200 RPM。

完整约定、操作参数、示例和校验原因见 [流程编写约定](docs/FLOW_AUTHORING.md)。校验只检查动作格式、运动参数及参考/扫码前置条件，不读取批次字段、不限制比赛顺序。站点宏仍保留，可以任意编排、重复操作、执行部分站点后结束；执行站点时保留二维码数组的越界保护。

只修改 `mission_flow.h` 的 `mission_flow[]`，然后重新编译并烧录 STM32。运行时读取编译后的结构体数组，不解析 C 头文件文本。

也可以在 `arm_flow.h` 编写自己的 `static const ArmStep my_action[]`，再在总流程插入 `FLOW_ARM(my_action)`。执行完整组机械臂/舵机小动作后，继续下一条路线步骤；同一组可重复调用。示例见 [机械臂动作编写](docs/ARM_FLOW.md)。

动作组内用 `ARM_AREA(ARM_AREA_RAW/PROCESS/STORE/STACK)` 显式声明作业区域；随后 `ARM_TO_PICK(ARM_CENTER)` / `ARM_TO_PUT(ARM_CENTER)` 使用该区的工位姿态。直接坐标运动、夹爪和等待仍可自由编写。每个动作组独立声明区域，不沿用上一组的区域。

默认取放也统一使用区域位置动作。`ARM_ABOVE_PICK/PUT()` 移到对应位置上方，`ARM_TO_READY()` 到该区收缩准备位；码垛加高只在放置位执行时计算一次。收臂由动作层的 `ArmFlow_Stow()` 编排，运动层只提供运动和逻辑位置读取。纯机械臂、直线移动、收臂和扫码流程不依赖 IMU，只有转向前需要 `FLOW_IMU_ZERO()`。

| 操作 | 含义 |
|---|---|
| FLOW_PREPARE | 初始化输出姿态，必须位于开头 |
| FLOW_IMU_ZERO | 记录当前 IMU 航向参考 |
| FLOW_STOW | 抬升、收臂、转盘回零 |
| FLOW_MOVE | 车体前后左右移动，单位 mm |
| FLOW_TURN | 左右转向，单位度 |
| FLOW_SCAN_TASK | 读取任务二维码 |
| FLOW_PICK_RAW | 所选批次的三件原料抓取 |
| FLOW_PROCESS_PUT / GET | 放完三件后再取回三件 |
| FLOW_STORE / STACK | 暂存或码垛三件物料 |
| FLOW_END | 结束流程，必须位于末尾 |
| FLOW_ARM(group) | 执行自定义机械臂/舵机动作数组 |

当前路线为PC下发的8段导航、8 个站点步骤、24 次物料转移。车载三个槽位角度为 0/120/240 度；物理左中右工位与车载槽位分别计算。原料在中位准备，等待固定抓取窗口中的目标色通过NEAR判断后夹取；两批加工均放完三件后原地直接取回，按二维码放置工位抓取，不再次校准或请求PICK；空暂存区使用与粗加工相同的圆环定位；码垛使用下层三色物块OCCUPIED校准，刷新工位后逐件用PICK查询同色物料所在位置。

机械臂左/中/右姿态和取放动作统一放在 `App/Inc/arm_flow.h`，编写方法见 [机械臂动作编写](docs/ARM_FLOW.md)：r 单位 0.1 mm，z 单位 mm，底座角度单位度。启动前机械臂应处于约定初始姿态；上电姿态视为逻辑零点，不读取编码器建立原点，也没有限位开关回零。

## 动作执行方式

- 底盘：四轮位置命令同步触发，根据距离、转速和加速度参数估计运行时间，等时间结束再执行下一步。
- 多轴：`Motion_MoveAll(r,z,base,rpm)` 依次发出伸缩、升降和底座舵机命令，三个执行器并行运行，等待三者预计耗时的最大值。
- 舵机：TIM6 中断持续更新独立轨迹，前台等待不阻止舵机插值。默认夹爪 300 度/秒，底座和转盘 360 度/秒；五次曲线按峰值速度计算时长。
- 转向：恢复旧版IMU PD调速，转速=0.8×角度误差−0.18×角速度，并限制在设定RPM内。误差≤1°、角速度≤3°/s时停车，稳定200 ms后继续。
- 视觉：原料TABLE从已HSV标定六色中选择单块，3帧位置稳定后返回整像素中心，避免亚像素抖动触发固定粗步；按误差方向固定粗调前后100 mm、左右5 mm，再循环识别、按像素距离精调，最多修正3次，每次移动后复核，两轴误差都≤1像素时提前结束；达到3次修正上限仍超差，也按最后位置继续取放。抓取时NEAR只检查目标像素周围半径12像素方框，目标色覆盖至少50%、连续3帧才通过。粗加工、暂存及堆叠仅按像素距离精调，不执行粗调或搜索；未识别到目标时等待有效坐标，OCCUPIED最终识别同时刷新工位锚点；允许按第一批二维码的左/中/右颜色工位和等间距假设，从两色推算缺失点，中心精调和平行校准均支持，推算点显示EST。需要同步更新相机main.py和STM32。所有结果按请求编号、种类和目标接收并回复ACK。
- 所有等待期间均服务 UART5、相机和 IMU。START 只登记待运行任务，由主循环进入完整流程，等待服务不会递归启动另一轮。

按本次要求取消了步进到位轮询、多轴反馈完成屏障、动作总超时、视觉总超时及停车重试状态机。相机无串口回复或转向条件不满足时会继续等待；粗调的 NONE 回复会触发下一段搜索，精调的 NONE 仍继续等待；精调后不再检查残余像素误差，直接进入取放；预计时间结束并不证明步进实际到位。相对位置按指令更新逻辑坐标。

底层仍保留 UART 发包结果、短事务应答及 Emm/X 固件格式识别，以适配本车硬件；这与运行中的实际到位判定不同。发包失败会退出本轮并发送一次停车指令。机械臂动作中途失败后，应断电恢复约定初始姿态再启动，避免沿用不准确的逻辑坐标。

## 参数

- 步进软件上限2000 RPM，在 `robot_config.h` 定义，`shared/stepper_limits.h`仅转发包含。
- 当前默认路线300 RPM、20档加速度（约84.75 RPM/s）；视觉中心校准和回原位200 RPM、30档加速度（约88.50 RPM/s）。转向上限150 RPM、机械臂2000 RPM、收臂1500 RPM，在 `stm32/yundongv2/App/Inc/robot_config.h` 修改。转向使用PD连续调速，150 RPM是上限；实际转速仍受角度误差、角速度反馈和加减速影响。
- 距离换算：前后 13064 脉冲/米（按300cm实测289cm标定）、横移 13228 脉冲/米、每圈 3200 脉冲。
- 机械臂：伸缩每转 127 mm，升降每转 40 mm；保持本车既有方向。
- 时间估算采用三角形/梯形计算方法。底盘Emm加速度档位按手册换算：acc非0时为20000/(256-acc) RPM/s，acc=0按匀速估算；不再把档位10当作10 RPM/s。参数在 `stm32/yundongv2/App/Inc/robot_config.h`。机械臂伸缩、升降和收臂的Emm加速度统一使用150档（约188.68 RPM/s），X固件加减速均为400 RPM/s；估时与实际下发参数共用；默认额外等待50 ms。不同驱动固件的参数含义、负载及机械阻力会影响实际时间，需要实车标定。
- 视觉参数：512×320图像，目标点(256,160)，精调及其他站点比例1.090819 mm/像素，图像右方映射B、下方映射L；先前后后左右，零差轴不动。粗调、搜索、中心精调和撤销均为200 RPM、30档加速度，均在 `robot_config.h`。

仅原料区使用粗调搜索。搜索距离在 `robot_config.h` 的 `MISSION_ALIGN_SEARCH_FORWARD_MM` / `MISSION_ALIGN_SEARCH_LATERAL_MM` 中，当前为前后每段 100 mm、左右每段 50 mm。固定按车体前、后、左、右循环，到达每个目标点后先重新识别；仍未找到才执行下一段，原料区每次从前移开始。搜索目标均相对同一个起点，每轮结束停在起点右侧50 mm，下一轮仍使用原起点，不会自动扩大搜索范围。为避免把稳定帧尚未积累完成时的 NONE 当作目标丢失，每次粗调请求先留出 1500 ms 识别窗口；窗口后收到新的 NONE 才移动。没有回复不会因时间经过而触发搜索。搜索持续到取得坐标，没有总次数限制。

## 启动与通信

App 连接热点 `RobotCar-ESP32S3`，密码 `robotcar123`，TCP `192.168.4.1:3333`。App命令仍为 `START,<非零uint32>\n`。运行中返回 BUSY；本次上电内拒绝重复或更小编号。App 仅显示 AUTO/TASK 状态，不提供速度设置或调试操作。

手机断开后 STM32 仍执行；App 通信超时会显示状态未知。无手机远程停止入口，现场使用硬件急停或断电。

| 功能 | STM32 TX | STM32 RX | 配置 |
|---|---|---|---|
| 底盘 USART1 | PB14 | PB15 | 115200，8N1 |
| 机械臂 USART2 | PA2 | PA3 | 115200，8N1 |
| ESP32 UART5 | PB13 → GPIO18 | PB12 ← GPIO17 | 115200，8N1 |
| 相机 UART4 | PA0 | PA1 | 115200，8N1 |
| JY901S USART3 | PB10 | PB11 | IMU 驱动探测并配置 |

夹爪 PC8/TIM3_CH3、转盘 PA8/TIM1_CH1、底座 PC6/TIM3_CH1，50 Hz。串口 TX/RX 交叉并共地。相机保留CAM/VISION/ACK格式，支持QR、TABLE、RING、OCCUPIED、PICK、NEAR、IDLE。本次原料方案需配套相机1.0.10（构建20261029-raw-single），同时更新STM32和相机。

## 构建和验证

STM32 工程在 Windows 自动查找 `%LOCALAPPDATA%/stm32cube/bundles` 下安装的 ARM 编译器和 Ninja，也支持 PATH。使用 CMake 3.22+，在 `stm32/yundongv2` 执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

独立夹爪15°测试：执行 `cmake --preset GripperTest`、`cmake --build --preset GripperTest`，烧录 `build/GripperTest/yundongv2.hex`。上电直接在PC8（TIM3_CH3，夹爪G）输出15°并保持，不等待App，不启动底座、转盘或比赛流程。测试入口为 `Tests/gripper_test.c`；测试后烧回 `build/Debug/yundongv2.hex` 恢复正常流程。GripperTest与CameraUartTest使用独立构建目录，不能同时启用。

独立循环抓取测试：执行 `cmake --preset GrabTest`、`cmake --build --preset GrabTest`，烧录 `build/GrabTest/yundongv2.hex`。VS Code选择“STM32 · 循环抓取测试（上电执行）”。上电设置初始舵机姿态、等待1500ms并收臂，然后循环执行原料中间位抓取→放入0°车载槽位→底座回180°→等待1000ms；不等待App、视觉、雷达或IMU，底盘不运动。复用`arm_flow.h`的抓取动作和正常机械臂速度；两项等待时间在`robot_config.h`的`ARM_GRAB_TEST_*`中设置。启动前将伸缩、升降放在约定零位，物料放在R110mm/Z−40mm/B180°对应的实际抓取位置，车载槽位姿态为R15mm/Z−10mm/B60°；循环中及时取走槽位物料并补料。电机命令失败时停止机械臂，不自动重试。各独立测试选项互斥；测试后烧回原任务固件。

独立循环放置测试：执行 `cmake --preset PlaceTest`、`cmake --build --preset PlaceTest`，烧录 `build/PlaceTest/yundongv2.hex`。VS Code选择“STM32 · 循环放置测试（上电执行）”。上电准备、等待1500ms并收臂，然后循环执行0°车载槽位取料→粗加工中间位放下→回准备位→等待1000ms；不等待App、视觉、雷达或IMU，底盘不运动。复用`ARM_PUT_STEPS(ARM_AREA_PROCESS,ARM_CENTER)`，放置姿态R120mm、Z−126mm、底座170°；等待时间在`robot_config.h`的`ARM_PLACE_TEST_*`中设置。启动前伸缩、升降位于约定零位，每轮向0°槽位补料并清空放置位置。电机命令失败时停止机械臂，不自动重试。与其他独立测试选项互斥。

输出位于 `build/Debug` 和 `build/Release`：`yundongv2.elf/.hex/.bin`。遇到旧缓存可用 CMake 3.24+ 执行 `cmake --fresh --preset Debug`。TIM1/TIM3 只由 CubeMX 初始化，当前定时器时钟为 240 MHz，PSC=239、ARR=19999，得到 1 MHz 计数和 50 Hz PWM；IOC 已同步。`ServoControl_Init(&htim1,&htim3)` 只绑定 PWM 句柄、检查参数并启动 TIM6 轨迹时钟，不重新初始化 PWM。CubeMX 重新生成后需保留 App 源文件清单、shared 包含路径、TIM6 由舵机模块管理的初始化及中断入口和链接脚本的 DTCM/8 KB 栈配置。

主机测试在 v2 根目录执行 `python tests/run_tests.py`，需要 Python 3 和 GCC。共15组，覆盖总流程嵌入及重复调用自定义动作组、机械臂动作表、流程表校验、完整路线和取放、固定任务码固件、App启动堆叠测试、相机顺序问答、多轴预计耗时、启动入口、舵机插值、Emm/X 相对位置帧、实际底盘接收缓冲及 IMU 初始化/配置/保存。未连接实车，主机测试及固件编译不代表机械可达性和预计耗时已经标定。

ESP32 使用 ESP-IDF 6.0，在 `esp32s3/hello_world` 执行 `idf.py build`。目标为 ESP32-S3、8 MB Flash；按 `build/flash_args` 烧录，不将应用 BIN 直接写到地址 0。

Android 使用 JDK 21、Android SDK 36.1，在 `application` 执行 `./gradlew.bat :app:assembleDebug :app:testDebugUnitTest`。APK 位于 `app/build/outputs/apk/debug/app-debug.apk`。

固定等待参数（ms）：MISSION_MOTION_SETTLE_MS=50、MISSION_STEP_SETTLE_MS=20、ARM_GRIP_SETTLE_MS=100、ARM_RELEASE_SETTLE_MS=50，均在robot_config.h。

机械臂Emm估时与底盘一样，将加速度档位换算为RPM/s；X固件按实际下发的400 RPM/s估时；伸缩、升降和收臂的Emm加速度统一使用150档（约188.68 RPM/s）。机械臂运动2000 RPM，收臂1500 RPM；夹紧后停100 ms，释放后停50 ms。

纯底盘导航固件：选择 `NavigationOnly` 预设，输出 `stm32/yundongv2/build/NavigationOnly/yundongv2.hex`；不执行相机识别、机械臂或舵机任务。PC仍用 `--robot`，App启动后自动采样规划并走完八段路线。完整任务固件仍为Debug。

免扫码完整任务固件：执行 `cmake --preset FixedQr`、`cmake --build --preset FixedQr`，输出 `stm32/yundongv2/build/FixedQr/yundongv2.hex`。VS Code选择“STM32 · 固定任务码（跳过扫码）”。仍由App启动并等待PC规划路线，走到二维码区后直接加载 `123+321+132+312` 并通知App，不请求相机扫码。第一批颜色123、放置321；第二批颜色132、放置312。后续原料识别、中心/平行校准和机械臂任务照常执行，因此仍需运行视觉程序。任务码集中在 `robot_config.h` 的 `MISSION_FIXED_QR_TEXT`；正常Debug/Release固件仍实际扫码。


独立堆叠任务固件：选择`StackTest`预设，输出`stm32/yundongv2/build/StackTest/yundongv2.hex`，VS Code选择“STM32 · 堆叠测试（App启动）”。现有App通过ESP发送START后执行一轮：准备/建立IMU参考/收臂→载入固定任务码→下层三色物块中心校准→平行校准（最多修正3次）→按第一批任务码的同色工位直接堆叠→收臂→恢复平行校准前航向→撤销校准平移→完成。校准后不再发PICK查询或视觉复核；平行最多修正3次，结束后直接执行夹取。无需PC规划、雷达或扫码，App/ESP协议不变；需要当前视觉程序与IMU，底盘仍参与校准、搜索和回原位。

测试前手动将车放到暂存区附近，伸缩/升降处于约定零位。沿用`MISSION_FIXED_QR_TEXT=123+321+132+312`的第二批顺序：车载0°槽位放红色1，120°槽位放蓝色3，240°槽位放黄色2；下层按第一批放置顺序摆好：图像左红、中黄、右蓝，相机需完成对应HSV标定。执行时直接按任务码将同色物块放到下层上方，堆叠高度沿用`ARM_STACK_HEIGHT=60mm`。本固件点击一次启动只执行一轮，不自动循环；再次启动前重新补料并恢复约定初始姿态。

任务码屏：已接入ILI9341显示四组任务码和24次物料转移进度。先用`ScreenTest`独立固件上电显示固定任务码，不初始化电机或舵机；完整流程扫码成功后显示真实任务码。接线、80像素大字、状态标记和烧录说明见[任务码屏幕](docs/TASK_DISPLAY.md)。
