# 底盘停车原始回包记录（2026-10-19）

最初诊断版本只增加观测。2026-10-19 检查后修正错误分类：收到本地址、合法帧尾的 E2/EE 时立即返回 HAL_ERROR，不再等待到 HAL_TIMEOUT；不改变成功 ACK 条件、停车重试或运动保护规则，不增加命令。App、ESP32 不需要更新。只有烧录新 STM32 固件后才能使用本记录。

## 保存内容

`g_zdt_diagnostics[0..3]` 对应底盘地址 1..4；地址 2（右前轮）是 `[1]`。每个地址分别保存最近一次已完成的停车事务 `stop` 和状态查询事务 `status`，查询不会覆盖停车回包。重试会覆盖同类上一次记录，不是无限历史日志。

- `attempts`：本次开机以来该类事务的累计完成次数。
- `started_tick/finished_tick`：事务起止 HAL tick（毫秒）。
- `tx_result`：HAL 发送结果。
- `rx_result`：最后一次 HAL 接收调用的返回结果；HAL_BUSY（2）在本记录中表示未调用接收，可能是发送失败或配置不等待 ACK，并不表示驱动器忙碌。
- `result`：函数最终返回值。当前固件对本地址 E2/EE 返回 HAL_ERROR；无匹配回包则 HAL_TIMEOUT；之前诊断版本把 E2/EE 也归为 HAL_TIMEOUT，读取时需区分实物固件版本。
- `uart_error`：事务结束时 HAL UART ErrorCode；这是位掩码，0 表示 HAL 未记录错误，不是电机的 E2/EE 错误码。
- `tx_length/tx[8]`：发送长度与发送字节；本记录的停车为 5 字节，状态查询为 3 字节。
- `rx_length/rx[32]`：实际成功读到的前至多 32 个字节，先保存再判断帧格式。不会为了记录而额外读取串口。
- `matched`：是否匹配本次期望的成功帧。
- `status_flags`：状态查询匹配成功时的驱动器状态位。

HAL 返回值：0=HAL_OK，1=HAL_ERROR，2=HAL_BUSY，3=HAL_TIMEOUT。

`publish_sequence` 为偶数且前后读取不变，说明记录没有跨越更新。奇数或变化时重新读取；attempts=0 表示该类事务尚未记录。只有事务完成后才发布记录，正在等待的事务不可见。

## 2 号常见原始字节

| 操作 | 发送 | 接收 |
| --- | --- | --- |
| 停车 | 02 FE 98 00 6B | 02 FE 02 6B：接受 |
| 停车 | 同上 | 02 FE E2 6B：条件或参数不满足 |
| 停车 | 同上 | 02 00 EE 6B：命令错误 |
| 查询状态 | 02 3A 6B | 02 3A flags 6B |

rx_length=0 仅表示本次等待期间没有成功从 HAL 读到字节；必须结合 tx_result、rx_result、uart_error 判断，不能一律认定电机离线。有效状态回包说明至少该次读状态通路是通的，即使停车 ACK 没有成功匹配。

## ST-Link 读取

使用本次 ELF 定位：

```text
p/x &g_zdt_diagnostics[1]
p sizeof(ZDT_MotorDiagnostics)
```

本次生成 ELF：2 号记录起始 0x20000628，大小 156 字节；stop=0x2000062C、status=0x20000678；每个原始事务 76 字节。再次构建后必须重新从对应 ELF 定位，并核实板上固件，不能照抄旧地址。

只读命令（执行时板上必须运行匹配版本）：

```powershell
& 'C:\Users\ASUS\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe' -c port=SWD mode=HOTPLUG freq=1000 -r32 0x20000628 156
```

不复位、不运行 GDB 函数、不直接读取 UART 的 RDR。读取 publish_sequence 前后值一致且为偶数后再解释。

## 验证

`tests/zdt_diagnostics_test.c` 覆盖成功 ACK、E2、EE、外地址及不完整字节、零字节超时、发送失败、HAL 接收错误、停车与查询独立保存、不同电机隔离、关闭等待 ACK 时的记录，并验证本地址 E2/EE 明确报错、其他地址的错误帧不影响本机 ACK 匹配。

上述测试、完整底盘控制回归和 STM32 构建均通过。没有自动烧录或让车运动。
