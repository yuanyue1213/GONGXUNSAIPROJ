# 雷达采样、PC规划和STM32执行

默认比赛流程已经改用PC路线。任务动作仍在`App/Inc/mission_flow.h`，导航由`FLOW_NAV(段名)`从RAM读取，不再执行旧的25个固定路线动作。

## 使用

1. 更新STM32的`stm32/yundongv2/build/Debug/yundongv2.hex`和合并后的ESP固件；App和相机不需要改。
2. 手机和PC都连接`RobotCar-ESP32S3`，密码`robotcar123`。
3. 在工作区运行：

```powershell
python -m pip install -r F:/GONGXUNSAIPROJ/leida/requirements.txt
python F:/GONGXUNSAIPROJ/leida/n10_wifi.py --robot --zone 1
```

`--zone 1`为右下，车头向上；`--zone 2`为右上，车头向下。也可以启动前在PC界面点右上/右下按钮。运行中锁定起点。没有`--robot`时仅为手动点云查看/虚拟预览，不配置STM32，也不发送路线；`--robot`拒绝`--demo`。

4. PC显示 `Start 1/2 confirmed; press App START` 后，App点击一次启动。PC未连接或起点心跳过期时，STM32返回`PC_NOT_READY`，不会起步。
5. 后续起步、两次完整扫描、规划、下发、取放和返程自动完成。PC实车模式不接受手动采样/播放/重放指令。

## 起步和采样

| 起点 | 起始坐标/朝向 | 车体动作 | 扫描1位姿 | 转向 | 扫描2位姿 |
|---|---|---|---|---|---|
| 右下(1) | 2250,150，向上 | 左移125，前进165 | 2125,315,0° | 左转90° | 2125,315,270° |
| 右上(2) | 2250,2250，向下 | 右移125，前进165 | 2125,2085,180° | 右转90° | 2125,2085,270° |

坐标单位mm；地图右为+X、上为+Y，朝向从上开始顺时针计。雷达默认位于车中心，0°朝车头。上述为指令预计位姿，不是定位反馈。STM32起步参数在`robot_config.h`的`NAV_*`项，PC对应坐标在`n10_robot.py`；改变初始摆放约定时须同时更新。

每次停车后再等待300ms，STM32发送扫描请求。ESP附上当时雷达序号，PC过滤旧包、丢掉第一段不完整的旋转，采集接下来一整圈；至少覆盖35个10°角度区间并有有效测距点才保存。重复、乱序和转向前的旧包不计入。扫描1保存成功并确认后才转向；扫描2确认后STM32原地等待完整路径。

两圈分别用当时的朝向变换到同一地图坐标。两圈只是减少车体遮挡，不能保证消除障碍物背后的盲区。现有格子规则仍是测距点命中则封锁、四个固定加工格封锁，未命中格不代表已观测或必然能通过；没有SLAM和车身扫掠检查。

PC从扫描2的向左朝向开始规划。右下扫描位置Y315与格中心Y350差35mm，路线第一段增加车体右移35mm接驳，不改起步165mm。最后恢复初始朝向、返回原启停区中心，而不是停在格中心。

## 连接与协议

ESP UART2：RX16接雷达TX，TX15可不接，230400 8N1。ESP UART1：TX17接STM32 UART5 RX/PB12，RX18接STM32 UART5 TX/PB13，115200 8N1，共地。UART0保留日志。

UDP3334仍使用`N10_SUB`订阅和`N10U`原始雷达帧。TCP3333只接受App的`START,id`，TCP3335接受PC控制帧；两个通道共用单个ESP UART写入循环，不会交叉拼接帧。ESP只传输，不规划。

控制帧均为ASCII、LF结尾。PC每秒发送`PC_HELLO,zone`；STM32每秒重发未确认的`SCAN_READY,run,scan,x,y,heading`，ESP在末尾加`first_sequence`。PC保存后回复`SCAN_OK,run,scan`。扫描和路径等待均不设置自动继续的超时。

```text
NAV_BEGIN,run,zone,count,CRC32
NAV_STEP,run,index,leg,MOVE,F,毫米
NAV_STEP,run,index,leg,TURN,L,90
NAV_END,run
```

CRC32覆盖ASCII文本`run,zone,count\n`和全部STEP原文（每行含LF），使用`zlib.crc32`约定，BEGIN中为8位十六进制。STEP序号从0连续递增，最多128条，单条移动1..10000mm；MOVE允许F/B/L/R，TURN仅允许L/R和90°。路线段必须按1..8递增排列并覆盖所有段：扫码、原料1、加工1、暂存1、原料2、加工2、堆叠、返程。

BEGIN回复`NAV_ACK,run,BEGIN`，STEP回复`NAV_ACK,run,index`，END校验通过后回复`NAV_READY,run,OK`。PC逐条等待确认，一秒未确认则重发；相同记录只重复确认，不重复保存。数量、CRC或范围错误不会执行，运行中不允许覆盖路线。STM32收到完整路径后自动继续，不需要第二次START。

每条执行前后回传`NAV_STATE,run,leg,index,RUNNING/DONE`。PC标记的是最近完成指令的预计位姿，不用虚拟播放时间驱动车辆，也不将该位置当成SLAM结果。路径已下载后，PC/手机断开不影响STM32自主执行；扫描/下载阶段断开则原地等待。PC在扫描中退出重启、ESP重启或更改实际摆放后，应将整车恢复初始姿态并重新启动，不拼接不同运行的采样。

## 构建

STM32仍用`cmake --preset Debug`和`cmake --build --preset Debug`，输出`build/Debug/yundongv2.hex`。

单独测试底盘导航时，使用`cmake --preset NavigationOnly`、`cmake --build --preset NavigationOnly`，烧录`build/NavigationOnly/yundongv2.hex`。VS Code运行配置选择“STM32 · 纯导航测试（无视觉/机械臂）”。该固件不编译相机与机械臂业务模块、不启动舵机PWM，不扫码、不校准、不取放、不自动收臂；启动前手动放好机械臂。仍需PC `--robot`模式、IMU和App START，先做125/165入场及两次扫描，再连续执行八段导航并返回起点。PC和ESP沿用合并版，不需要为此更改协议。测试后烧回Debug固件恢复完整比赛任务。

在ESP-IDF终端中：

```powershell
cd F:/GONGXUNSAIPROJ/yundong_partv2/esp32s3/hello_world
idf.py -B build_route build
idf.py -B build_route -p COMx flash
```

本机旧build目录绑定另一套Python，新增`build_route`避免清理旧构建。`leida/hello_world`也已指向同一份合并桥源码，两个目录任选其一构建；不要再烧旧的独立雷达应用。

本次另外生成了`build_route/robot_route_full.bin`：包含bootloader、分区表和应用，烧录地址0x0。`robot_start_bridge.bin`只是应用镜像，不能直接当完整镜像烧到0x0。

验证命令：`python -m unittest discover -s leida -p "test_n10_*.py"`、`python yundong_partv2/tests/run_tests.py`。软件测试和编译不能替代实机接线、安装方向、起点摆放和运动距离标定。
