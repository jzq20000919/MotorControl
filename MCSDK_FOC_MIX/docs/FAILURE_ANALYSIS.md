# Qt 位置控制程序失败原因分析

## 结论

刚才出现“ASPEP 握手成功，随后状态轮询触发 NACK”的主要原因，是上位机将 **4 字节 ASPEP 包头和 MCP 负载在同一次 `WriteFile` 中连续发送**。本工程 MCSDK 6.4.2 固件先用 DMA 接收包头，随后才重新配置 DMA 接收负载。主机连续发送时，负载可能在 DMA 重新装载前到达 USART2，造成字节丢失、帧错位，最终由固件返回 ASPEP NACK。

正确做法与参考工程 `E:\Projects\MCSDK_FOC\MCSDK_FOC_2804\QtFocControl` 一致：

1. 单独发送 4 字节 ASPEP 包头。
2. 等待 2 ms。
3. 再发送 MCP 负载。
4. 在收到同步响应前不发送下一条请求。

针对 1,843,200 baud 的 ST-LINK VCP，当前实现还在包头写入后调用
`FlushFileBuffers()`，确保四字节包头已离开 Windows `usbser` 缓冲区，再开始
2 ms 计时。否则两个独立的 `WriteFile()` 仍可能被 USB 串口驱动合并成同一批次。

## 证据链

### 1. 串口和基本 ASPEP 参数是正确的

运行日志如下：

```text
已打开 COM16，1,843,200 baud，发送 ASPEP Beacon
ASPEP Beacon 已确认，发送 Ping
已连接 · MCSDK ASPEP/MCP
握手完成
控制板返回 ASPEP NACK
```

Beacon 和 Ping 都通过了固件的 ASPEP 包头 CRC、协议版本和能力参数检查，因此可以排除以下问题：

- COM 口选择错误；
- 1,843,200 baud 配置完全错误；
- TX/RX 完全不通；
- ASPEP CRC4 算法错误；
- Beacon 能力字段不匹配。

NACK 出现在握手后的第一类 MCP 数据请求阶段，故问题集中在“带负载的数据帧”发送方式。

### 2. 参考工程明确采用分段发送

参考工程 `QtFocControl/src/aspepprotocol.cpp` 定义：

```cpp
constexpr int kHeaderPayloadGapMs = 2;
```

其 `sendNextCommand()` 先调用一次 `serial_.write(headerBytes)`，再通过 `QTimer::singleShot(2, ...)` 发送 `payload`。代码注释明确指出，MCSDK 6.4.2 需要先完成包头 DMA，再为负载重新装载 DMA。

### 3. 当前位置固件具有相同的 DMA 时序

本工程固件中的相关路径：

- `MCSDK_FOC_pos/Src/stm32_mc_common_it.c`：在 2 kHz `SysTick_Handler()` 中检查 USART RX DMA 完成标志，并调用 `ASPEP_HWDataReceivedIT()`。
- `MCSDK_FOC_pos/Src/aspep.c`：收到 `DATA_PACKET` 包头后，调用 `fASPEP_cfg_recept()` 配置负载接收。
- `MCSDK_FOC_pos/Src/usart_aspep_driver.c`：`UASPEP_CFG_RECEPTION()` 会停止 DMA、修改内存地址和长度，再重新启用 DMA。
- `MCSDK_FOC_pos/Inc/parameters_conversion.h`：`SYS_TICK_FREQUENCY` 为 2000 Hz，即 0.5 ms 一个周期。

在 1,843,200 baud、8N1 下，一个串口字节约占：

```text
10 / 1,843,200 ≈ 5.43 μs
```

如果包头与负载连续写入，包头最后一个字节后约 5.43 μs 就会到达第一个负载字节，远短于固件 0.5 ms 的轮询周期。2 ms 间隔可覆盖多个 SysTick 周期，给 DMA 重装留出余量。

## 我刚才实现中的具体错误

旧版 `AspepClient::pumpQueue()` 使用：

```cpp
m_serial.write(makeFrame(kDataFromController, m_active.payload), ...);
```

`makeFrame()` 返回的是“包头 + 负载”的完整字节数组，因此 Windows 会把它作为连续串口数据发送。这与参考工程经过验证的分段发送方式不一致。

## 为什么最初的判断不完整

初次日志中，Beacon 响应约在 610 ms 后到达，而旧版超时为 600 ms。我先把问题归因于超时过短，并将超时增大、轮询频率降低。这可以减少无效重试，但它不是 NACK 的根本原因：

- 超时影响的是主机何时重试；
- NACK 表明固件实际收到了一个无法正确解析的传输层帧；
- 参考工程和固件源码共同指向包头/负载之间缺少 DMA 重装间隔。

所以“仅增加超时”的修正属于缓解措施，分段发送才是协议层修复。

## 其他独立问题

### 串口列表最初为空

这是另一个独立问题。旧实现使用 Qt `QSettings` 读取 `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM`，设备值名中的反斜杠可能被 Qt 当作层级路径，导致 ST-LINK VCP 漏检。系统实际一直识别到：

```text
STMicroelectronics STLink Virtual COM Port (COM16)
```

现已改为 Windows 原生 `RegEnumValueW`，并使用 `QueryDosDeviceW` 兜底，同时允许手动输入 COM 号。

### 编译时无法覆盖 EXE

一次重新构建出现 `Permission denied`，原因是旧程序仍在运行，Windows 锁定了 `MotorPositionControl.exe`。这与串口协议无关。为避免强制结束用户进程，更新版构建到了新的发布目录。

## 已实施修复

- 按参考工程拆分 ASPEP 包头和 MCP 负载，间隔 2 ms；
- 包头后调用 `FlushFileBuffers()`，避免 Windows USB 串口驱动合并两次小写入；
- 保证同一时刻只有一个 MCP 请求在途；
- 快速拖动位置圆盘时合并尚未发送的位置命令，只保留最新目标；
- 启停和故障命令优先排队；
- NACK 显示具体错误码，并自动进入协议重同步；
- 增加参考工程风格的位置圆盘；
- 保留本固件的标准 MCP 寄存器接口：`MC_REG_POSITION_RAMP` 和 `MC_REG_CURRENT_POSITION`。

## 验证标准

连接实际控制板后，应满足：

1. Beacon/Ping 握手完成；
2. 连续读取位置状态至少 60 秒，无 ASPEP NACK；
3. 写入位置命令收到 MCP `0x00` 成功响应；
4. 目标位置、当前位置和状态轮询持续更新；
5. 快速拖动圆盘不会堆积大量过期位置命令。

## 2026-07-31 实机复测记录

### v1.2

用户日志确认分段发送已执行：

```text
TX 启动电机 HEADER [4]  29 00 00 E0
TX 启动电机 PAYLOAD [2]  19 00
启动电机：失败 (控制板响应超时)
```

`19 00` 是正确的 Motor 1 `START_MOTOR` MCP 命令。即使电机状态不允许启动，
固件也应返回一个 MCP 成功或失败状态字，因此“完全没有响应”仍属于传输层问题，
不是普通的电机启动拒绝。

两次 `WriteFile()` 在应用日志中相隔 2 ms，不代表 Windows `usbser` 和 ST-LINK
一定会在物理 UART 上形成两个独立突发。因此 v1.3 在包头写入后增加了
`FlushFileBuffers()`，再开始 2 ms 等待。

### v1.3

自动化实测成功打开 COM16，但控制板在两轮尝试中没有返回 Beacon：

```text
已打开 COM16，1,843,200 baud，发送 ASPEP Beacon
握手超时，重试 1/3
握手超时，重试 2/3
握手超时，重试 3/3
ASPEP 握手超时
```

此时连不带负载的控制帧都无响应，无法继续验证 MCP 数据帧。需要先确认 MCU 正在运行
（未停在 Keil 调试态），并对控制板复位或重新上电，使 ASPEP RX DMA 回到接收包头状态，
再执行上述五项验证标准。
