# 持续 BUSY 和空 IMU 数值字段排查（2026-10-18）

## 板上读取结果

本次通过 ST-Link Hot Plug 读取了换线后、修复前的运行固件，没有复位、烧录或驱动电机。

- GyroTurn g.state=3（GYRO_ERROR）。
- g.reason 指向 STARTUP_STOP 字符串（Flash 实际读取确认）。
- g.stop_mask=0x0D，1、3、4 号驱动器已被 stop_all() 确认为停止或禁用；2 号尚未确认。
- s_exec_active=false，其他被读取的任务 active/pending 标志均为 false。

因此 BUSY 的直接原因是启动停车保护仍占用底盘，并非陀螺仪未就绪或机械臂原点未初始化。2 号驱动器为什么未确认，仍需检查其供电、通信接线、地址 2 和驱动器响应，不能据此认定电机损坏。

旧固件检查位置：g=0x20001B50，stop_mask=0x20001B94，reason 指针=0x20001B54。重新链接后的地址可能改变；后续读取必须匹配实物固件 ELF，不得直接复用这些位置。

## 软件修复

1. CMake 链接选项加入 -u _printf_float。原固件使用 nano.specs 但没有链接浮点格式化，导致 snprintf 的 %.1f 没有输出，出现 IMU,id,READY,,。新 ELF 已确认包含 _printf_float 的已定义实现；实际回包需烧录后验证。
2. BUSY 回包 reason 从统一 ACTIVE 改为具体任务/停车原因，保持原来的忙碌判定条件，不绕过停车确认。
3. App 将具体 BUSY 原因翻译为对应提示，兼容旧版 ACTIVE。

验证：新增模拟 2 号驱动器停止确认失败，确认新动作返回 BUSY,STARTUP_STOP 且没有速度输出；停车重试成功后恢复空闲。底盘控制回归、STM32 构建、App 单元测试和 APK 构建均通过。

生成新 HEX/BIN/APK，未自动烧录。ESP32 固件和接线无需再修改。
