# 机械臂动作编写

参数已统一迁移至 `App/Inc/robot_config.h`：下文涉及的区域姿态、速度、槽位、准备位、夹爪角度、等待时间及收臂角度都在该头文件修改；`arm_flow.h` 只安排动作顺序和自定义动作组。

当前初始化底座为90°，车载转盘三个槽位为0°/120°/240°。原料区中间抓取位底座为180°；每件物料放入车载槽位并抬升后，底座回180°，再开始下一件。所有显式视觉校准前底座统一180°。收臂行走角为180°。

整体路线写在 `mission_flow.h`，机械臂和舵机动作写在 `App/Inc/arm_flow.h`。修改后重新编译并烧录 STM32。

## 把自己写的动作组插入总流程

在 `arm_flow.h` 中定义一个命名数组，放在 `ArmStep` 和动作宏定义之后：

```c
static const ArmStep my_action[]={
    ARM_AREA(ARM_AREA_RAW),      // 明确声明在原料区工作
    ARM_TO_READY(ARM_CENTER),   // 该区中间方向的收缩准备位
    ARM_SERVO('G',30),
    ARM_WAIT(200),
    ARM_TO_PICK(ARM_CENTER),    // 使用原料区中间工位的抓取姿态
    ARM_SERVO('G',0),
    ARM_WAIT(500),
    ARM_ABOVE_PICK(ARM_CENTER), // 抓取位上方，升降高度为 0
};
```

然后在 `mission_flow.h` 的 `mission_flow[]` 中插入：

```c
FLOW_MOVE(FLOW_FORWARD,500),
FLOW_ARM(my_action),
FLOW_MOVE(FLOW_LEFT,200),
```

前移预计结束后，执行整组 `my_action`，组内动作依次完成后再左移。同一数组可在总流程多次引用，无需新增操作枚举或修改执行器。头文件已有 `arm_ready` 作为简短示例，默认比赛路线没有插入该示例。

`FLOW_ARM()` 的参数必须是编译期可见的数组名，不能是指针或函数；动作数组使用 `static const`，每组 1～128 步，并放在 `FLOW_PREPARE()` 之后、`FLOW_END()` 之前调用。它不要求 IMU 或扫码，也不会自动视觉对准、选择槽位或更新批次取放计数。这些由现有站点宏负责；需要自主控制时自行编写动作组。

组内任一动作失败会中止总流程并调用停车。启动检查只检查动作组指针和长度，具体运动参数由运动接口检查，不能证明姿态无碰撞。标准站点也已取消固定比赛顺序限制，可自行编排批次和站点，但使用站点前仍须扫码。

`arm_flow.h` 只保存姿态和动作组，类型及初始化宏在 `arm_types.h`，执行循环在 `App/Src/arm_flow.c`。编写动作不需要改执行器。

## 在动作组中显式选择区域

```c
static const ArmStep my_store_action[]={
    ARM_AREA(ARM_AREA_STORE),
    ARM_SERVO('G',0),
    ARM_ABOVE_PUT(ARM_CENTER), // 先到该区放置位上方；避障路径自行编写
    ARM_TO_PUT(ARM_CENTER),    // 当前区域的中间放置位
    ARM_SERVO('G',30),
    ARM_WAIT(200),
    ARM_ABOVE_PUT(ARM_CENTER),
    ARM_TO_READY(ARM_CENTER),
};
```

区域有 `ARM_AREA_RAW`（原料）、`ARM_AREA_PROCESS`（加工）、`ARM_AREA_STORE`（暂存）、`ARM_AREA_STACK`（码垛）。位置使用 `ARM_LEFT`、`ARM_CENTER`、`ARM_RIGHT`，分别为物理左、中、右。

`ARM_TO_PICK()` / `ARM_TO_PUT()` 根据组内最近一次 `ARM_AREA()` 选择区域姿态表，只执行伸缩、升降、底座运动，不自动夹取、放开或视觉对准。没有区域声明时，区域位置动作返回失败；每组执行开始时区域为空，不继承上一组；同一组可以再次声明来切换区域。

`ARM_ABOVE_PICK(pos)` / `ARM_ABOVE_PUT(pos)` 使用该区抓取/放置伸缩位置、底座角度，升降高度为 0；`ARM_TO_READY(pos)` 使用该区底座角度、伸缩 `ARM_READY_R`、升降高度 0。默认站点和自定义组共用这套查表执行逻辑。码垛补偿只在 `ARM_TO_PUT()` 中应用，上方姿态不增加码垛高度。

直接填写坐标的 `ARM_MOVE()` 仍严格使用所写坐标，不按区域增加偏移。`ARM_SERVO()`、`ARM_WAIT()` 也不受区域影响。纯夹爪、转盘、收臂等不依赖区域姿态的动作组可以不写区域声明。

各区姿态在 `ARM_RAW_POSES`、`ARM_PROCESS_POSES`、`ARM_STORE_POSES`、`ARM_STACK_POSES` 中分别设置。当前沿用原来的三工位参数，码垛放置额外增加 `ARM_STACK_HEIGHT=60 mm`，对应初赛图5的一件物料高度。默认站点取放也使用这些区域表；原料 NEAR 等待仍由站点执行函数负责，自定义动作组没有自动视觉等待。

## 可以修改的动作

| 定义 | 用途 |
|---|---|
| `ARM_STATION_POSES` | 各区默认共用的左、中、右抓取/放置伸缩位置、高度、底座角度 |
| `ARM_RAW/PROCESS/STORE/STACK_POSES` | 各区的三工位姿态，可分别修改 |
| `ARM_SLOT_ANGLES` | 三个车载槽位的转盘角度 |
| `ARM_ALIGN_BASE` | 视觉对准时底座角度 |
| `ARM_PREPARE_STEPS` | 启动时直接设置三路舵机输出 |
| `ARM_PICK_APPROACH(area,pos)` | 声明区域、张爪、到工位准备位置 |
| `ARM_PICK_FINISH(area,pos)` | 声明区域、伸出下降、闭爪、抬升、放到车载槽位 |
| `ARM_PUT_STEPS(area,pos)` | 声明区域、从车载槽位取出、放到工位或码垛 |
| `ARM_STOW_STEPS(r,b)` | 保持当前伸缩和底座先抬升，再收臂和转盘回零 |

`area` 是作业区域，`pos` 是该区当前左/中/右位置；收臂的 `r,b` 是进入收臂前的伸缩逻辑位置和底座角度。`ArmFlow_Stow()` 负责取得当前姿态并执行收臂表，运动层不再调用动作执行器。

## 动作写法

```c
ARM_MOVE(110, 0, 57),              // 伸缩、升降、底座并行，到目标姿态后继续
ARM_MOVE_AT(110, -36, 57, 100),    // 指定步进转速 100 RPM
ARM_SERVO('G', 30),               // 夹爪到 30°，默认使用该通道最大速度
ARM_SERVO_AT('B', 170, 180),       // 底座到 170°，峰值速度 180°/s
ARM_WAIT(500),                    // 等待 500 ms
```

伸缩 `r` 单位为 **0.1 mm**，升降 `z` 单位为 **mm**，角度单位为度。MOVE 的 r/z/base 都是相对上电逻辑零点的目标位置，不是本次移动量。舵机通道 G=夹爪、B=底座、T=转盘。

`ARM_SET()` 直接设置初始输出，不做平滑运动，只在启动准备中使用。

动作宏是多行 `#define`：除最后一行外，各行末尾必须保留反斜杠 `\`；相邻动作之间用逗号分开。例如延长抓紧等待：

```c
#define ARM_PICK_FINISH(area,pos) \
    ARM_AREA(area), \
    ARM_TO_PICK(pos), \
    ARM_SERVO('G',0), \
    ARM_WAIT(700), \
    ARM_ABOVE_PICK(pos), \
    ARM_MOVE(110,0,57), \
    ARM_MOVE(110,-36,57), \
    ARM_SERVO('G',30), \
    ARM_WAIT(200), \
    ARM_MOVE(110,0,57)
```

两批原料都先张爪并到准备位（11 mm、0 mm、180°），在 APPROACH 和 FINISH 之间等待相机 `NEAR`，确认后才伸出下降并闭爪。加工取回不等待 NEAR，准备位之后直接伸出下降。扫码、选工位和视觉对准仍由站点流程负责。

`ArmFlow_Run()` 只循环执行数组，任一动作返回失败就立即退出。多轴仍按预计最长时间等待，不增加到位反馈或动作总超时。保留先抬升再收臂的机械避障顺序；修改姿态后需要实车验证。
