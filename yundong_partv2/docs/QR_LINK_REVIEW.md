# 扫码链路核查（2026-10-28）

## 已得到的实机信息

- 原始 `camera/MaixVision/qrcodes.py` 可以显示USB摄像头并扫码。
- App最终停在“读取任务二维码 / QR”。
- 同一时刻，相机是 `IDLE / RX:1 / LAST:NONE`。
- 用户确认相机使用UART2，B1 RX接STM32 PA0 TX。

结论：已进入STM32扫码等待；相机当前没有解析到有效CAM请求。源码检查不能据此证明物理TX已有输出，也不能证明该1字节是什么。尚未实测串口波形，不把接线、串口占用或运行旧程序中的任何一项断言为唯一根因。

## 当前运行代码核查范围

App的MainActivity、StartViewModel、StartProtocol、Manifest与Gradle配置；ESP的start_bridge与启动协议；STM32的App/Src全部模块、动作及参数头文件、共享协议、Core初始化/中断/引脚时钟与CMake；当前视觉main.py的串口接收、所有识别模式、标定、USB与主循环；同时对照qrcodes.py、USB_camera.py及历史视觉入口的扫码路径。第三方HAL/CMSIS只检查与当前串口链路有关的实现，不宣称已逐行审计整个供应商库。

## 实际调用链及核查结果

| 环节 | 代码行为 | 判断 |
|---|---|---|
| App | TCP发送START，只接收AUTO/TASK状态 | 已收到SCANNING，启动链路已推进 |
| ESP | START转发到UART1 GPIO17/18，不发送相机请求 | 本次已越过启动阶段 |
| STM32流程 | FLOW_SCAN_TASK先报告SCANNING/QR，再调用CameraLink_Wait(TASK_QR) | 状态表示进入扫码步骤，不是相机已确认收到 |
| STM32发送 | CameraLink绑定huart4，115200、8N1、TX/RX，发CAM,id,QR加LF；每秒同编号补发 | 相机收不到时不会自行打开USB |
| STM32引脚 | PA0/PA1，GPIO_AF8_UART4，无其他当前模块重配这两个引脚 | PA0发出、PA1接收；USART2的PA2/PA3用于机械臂 |
| 相机引脚 | /dev/ttyS2，B0 UART2_TX、B1 UART2_RX，初始化时显式pinmap | 与Sipeed官方映射一致 |
| 相机接收 | read()非阻塞，按LF组帧；允许CRLF及分片；非法帧丢弃 | 实测累计只有1字节，不足以组成完整扫码请求 |
| 相机切换 | select收到QR后，UsbQrCamera才创建VideoCapture(0) | 当前LAST:NONE，尚未执行USB打开分支 |
| USB识别 | 与原版相同的设备0、640×480、30fps、cv2image及find_qrcodes | 原版可用不能验证STM32到相机的请求方向 |
| LAB及其他模式 | QR分支不查询LAB标定 | 未标定不能解释这次不切换 |
| IMU及前置动作 | IMU配置失败确实可能阻塞更早流程 | 用户已确认进入SCANNING，这个分支不是本次原因 |
| 应用包 | 检查旧1.0.0包时，其main.py与当时工作区一致；新诊断版另打1.0.1包 | 通过屏幕构建号确认设备实际运行的是哪个版本 |

官方依据：[UART与引脚映射](https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html)、[UART.read默认非阻塞定义](https://github.com/sipeed/MaixCDK/blob/main/components/peripheral/include/maix_uart.hpp)。未读取设备系统配置，不能把UART2被其他进程占用当作已确认事实。

## 本轮修改及验证边界

当前main.py加入构建号 `20261028-uart-check`、UP运行秒数、RX/OK/BAD计数、原始接收字节和异常错误画面，用于区分程序冻结、无数据和格式错误。保留完整比赛请求流程；没有绕过STM32自行扫码，也没有修改UART端口、路线、IMU或电机动作。

29项主机视觉测试通过，包含启动噪声、碎片请求、同编号重发不清空稳定帧、USB画面与二维码框显示。此前11组STM32测试及Debug/Release构建已通过，本轮未改STM32代码。主机测试不能证明PA0到B1的真实电气传输正常。

## 实机结果判读

1. 未出现新构建号：先确认上传/安装及开机自启应用，不能用旧画面判断新程序。
2. UP不增加或VISION ERROR：查看控制台异常堆栈，程序可能已停止。
3. UP增加、RX不增加：读取路径没有新数据；需观测PA0输出或用3.3V USB串口从B1注入 `CAM,1,QR` 加LF。注入时断开STM32到B1的TX，避免两个TX同时驱动。
4. RX增加、BAD增加：看RAW及控制台被拒绝的字节，核对格式/波特率/干扰。
5. LAST:QR#编号：请求已经通过解析，此时才进入USB摄像头打开、读帧和二维码解码排查。
