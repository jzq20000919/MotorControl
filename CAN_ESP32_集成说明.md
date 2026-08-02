# ESP32 与 MCSDK_FOC_MIX 双通信集成说明

## 0. 串口 / CAN 页面与互斥规则

ESP32 首页现在是**串口通信**页：选择波特率后点击 `RECONNECT`，该操作会启用 UART 并停止 CAN。第二页是**CAN 通信**页：点击 `CONNECT CAN 500K` 会启用 CAN 并停止 UART。速度、位置、启动、停止、故障确认和遥测页面都通过当前有效的传输层工作；没有选择任一连接时，控制命令不会发送。

| 通信方式 | ESP32 扩展口 | ESP32 引脚 | 电机板引脚 | 当前固件默认速率 |
|---|---|---|---|---:|
| UART | Port2 | GPIO18 TX、GPIO8 RX | USART3：PC10 RX、PC11 TX | 460800 baud，8N1 |
| CAN | Port1 | GPIO5 TX、GPIO6 RX | FDCAN1 经板载收发器 | 500 kbit/s，经典 CAN |

UART 必须交叉接线并共地：`GPIO18(TX) → PC11(USART3_RX)`，`GPIO8(RX) ← PC10(USART3_TX)`，另接 `GND ↔ GND`。CAN 仍按下文的 SIT1042 接线。两种物理链路可以都接着，但 ESP32 界面会保证一次只启用一种，避免两套心跳/控制命令竞争。

波特率下拉框提供常用值；电机板新增 UART3 桥接的固件默认固定为 **460800**，因此首次连接请选择 `460800`。选择其他值可用于配合后续改动后的电机板固件；与电机板实际速率不一致时，界面会保持 `UART OFFLINE`，不会误显示为已连接。

## 1. 本次实现

- ESP32-S3 使用片上 TWAI 控制器和外接 SIT1042AQT/3 收发器。
- 电机控制板使用 STM32G431 的 FDCAN1 和板载 CAN 收发器。
- 总线采用经典 CAN、11 位标准标识符、500 kbit/s、8 字节数据帧。
- ESP32 可下发模式切换、速度目标、单圈位置目标、启动、停止和故障确认。
- 电机板每 20 ms 回传运行状态、故障、速度、位置以及实际/参考 dq 电流。
- Qt/MCP 与 CAN 共用 `foc_app_protocol` 的控制入口，避免出现两套速度/位置算法。
- ESP32 位置盘只在松开触摸后提交一次目标，电机板将 0–360° 单圈目标换算为离当前位置最近的多圈目标。

Qt/Motor Pilot 继续使用 USART2；ESP32 的 UART 使用新增的 USART3，互不占用。ESP32 内部的 UART 与 CAN 则严格互斥，始终只有当前页面连接成功的那一种能够下发命令和接收遥测。

本次只进行了源码和工程配置修改，未执行 ESP-IDF、Keil 或其他形式的编译。

## 2. 接线

| ESP32 / Port1 | SIT1042AQT/3 模块 | 电机控制板 |
|---|---|---|
| GPIO5（CAN TX） | TXD | — |
| GPIO6（CAN RX） | RXD | — |
| 5 V | VCC | — |
| GND | GND | GND |
| — | CANH | CANH |
| — | CANL | CANL |

注意：CAN 控制器与收发器之间不是 UART，不要交叉连接 TXD/RXD。GPIO5 接 TXD，GPIO6 接 RXD。模块的 `S` 应保持低电平以进入高速模式；使用图片所示模块时，`EN` 可按模块默认方式悬空。

DNESP32S3B 原理图中的两组 4 针扩展口并不相同：

- J2/Port1：针脚 1=5V、2=GND、3=GPIO5/TX、4=GPIO6/RX；当前固件使用这一组。
- J3/Port2：针脚 1=5V、2=GND、3=GPIO18/TX、4=GPIO8/RX；当前 UART 页面使用这一组，必须与电机板 USART3 的 PC11/PC10 交叉连接。

SIT1042AQT/3 模块的 VCC 按模块说明接 5 V，它的 TXD/RXD 接口标注兼容 3.3 V/5 V。ESP32 与电机控制板必须共地。

## 3. 终端电阻

CAN 总线物理链路的两个端点各需要一个 120 Ω 终端，但不能在多个位置重复接入。断电后测量 CANH 与 CANL：

- 约 60 Ω：通常表示两个 120 Ω 终端均已接入。
- 约 120 Ω：通常只有一个终端，需要在另一端补充一个。
- 明显低于 60 Ω：可能接入了过多终端。

已核对电机控制板原理图：板端 R50 是固定接入 CANH/CANL 的 120 Ω 终端。当前只有 ESP32 模块和电机板两个节点，因此 ESP32 模块的 SW1 必须在断电状态下拨到 ON，组成第二个 120 Ω 终端；断电测得 CANH-CANL 应约为 60 Ω。不要在带电状态下测电阻或切换终端。

## 4. STM32 引脚与位时序

- `PA11`：FDCAN1_RX
- `PB9`：FDCAN1_TX
- FDCAN 内核时钟：PCLK1，170 MHz
- Nominal Prescaler：17
- Nominal TimeSeg1：15 tq
- Nominal TimeSeg2：4 tq
- Nominal SJW：4 tq
- 计算结果：`170 MHz / 17 / (1 + 15 + 4) = 500 kbit/s`
- 采样点：80%

电机板使用轮询方式读取 RX FIFO，不增加新的 FDCAN 中断，不影响现有电机控制中断优先级。

## 5. CAN 报文协议

所有多字节整数均为小端序。

### 5.1 ESP32 → 电机板：控制帧 0x100

| 字节 | 内容 |
|---|---|
| 0 | 协议版本，当前为 1 |
| 1 | ESP32 递增序号 |
| 2 | 命令码 |
| 3–6 | 命令参数 |
| 7 | 保留，置 0 |

命令码：

| 命令码 | 功能 | 参数 |
|---:|---|---|
| 0 | NOP | 无 |
| 1 | 设置模式 | byte3：0 速度，1 位置 |
| 2 | 设置速度 | byte3–4：int16 RPM |
| 3 | 设置位置 | byte3–6：int32，0.01°，ESP32 使用 0–35999 |
| 4 | 启动电机 | 无 |
| 5 | 停止电机 | 无 |
| 6 | 确认故障 | 无 |
| 7 | 零点命令 | 为安全起见，电机板明确拒绝 |
| 8 | 心跳 Ping | 无，ESP32 每 100 ms 发送 |

### 5.2 电机板 → ESP32：状态帧 0x180

| 字节 | 内容 |
|---|---|
| 0 | 协议版本 |
| 1 | 最近收到的序号 |
| 2 | 最近收到的命令码 |
| 3 | 状态位 |
| 4–5 | int16 实际速度 RPM |
| 6–7 | uint16 当前 MCSDK 故障码 |

状态位：bit0 位置模式，bit1 RUN，bit2 故障，bit3 链路有效，bit4 最近命令被拒绝。

### 5.3 电机板 → ESP32：参考与位置帧 0x181

| 字节 | 内容 |
|---|---|
| 0–1 | int16 参考速度 RPM |
| 2–3 | uint16 当前单圈位置，0.01° |
| 4–5 | uint16 目标单圈位置，0.01° |
| 6–7 | int16 位置误差，0.01° |

### 5.4 电机板 → ESP32：电流帧 0x182

| 字节 | 内容 |
|---|---|
| 0–1 | int16 实际 Iq，mA |
| 2–3 | int16 实际 Id，mA |
| 4–5 | int16 参考 Iq，mA |
| 6–7 | int16 参考 Id，mA |

## 6. 操作顺序

1. 分别编译并下载 `MCSDK_FOC_MIX` 和 `ESP32_LVGL/ESP32_LVGL`。
2. 上电后 ESP32 首页应由 `CAN OFFLINE` 变为 `CAN ONLINE`，RX 计数持续增长。
3. 点击 SPEED 或 POSITION 模式按钮；ESP32 会先设置模式，再发送启动命令。
4. 等待电机完成对齐并进入 RUN。
5. 速度模式拖动滑条即可下发速度；位置模式拖动角度盘，松开后提交一次位置目标。
6. 首页 `STOP` 可直接发送停机命令；排除故障原因后可点击 `ACK FAULT` 确认故障。
7. 若首页显示 `CMD REJECTED`，常见原因是电机尚未进入 RUN、当前模式不匹配或存在未确认故障。

## 7. 无通信时检查

1. 确认 ESP32 GPIO5→TXD、GPIO6←RXD，没有按 UART 方式交叉。
2. 确认模块 VCC 为 5 V、三方 GND 相连、`S` 为低电平高速模式。
3. 断电测量 CANH-CANL 终端电阻是否约为 60 Ω。
4. 确认 CANH 对 CANH、CANL 对 CANL；交换 CANH/CANL 会导致完全无帧。
5. 确认两端均使用经典 CAN 500 kbit/s，而不是 CAN FD 或其他波特率。
6. 若 ESP32 TX 计数增长而 RX 始终为 0，优先检查电机板固件是否已更新、PA11/PB9 是否与板载收发器相连、终端和共地。
7. 若 TX ERR 持续增长，通常表示总线上没有其他节点 ACK，应检查电机板供电、收发器使能、波特率和物理接线。
8. 新固件启动时会打印 `Port1 RXD idle level=x`，正常空闲状态应为 1；若 RXD 为 0，应先检查 SIT1042 模块 VCC/VIO、S 引脚、CANH/CANL 短路或接反。TWAI TXD 是外设矩阵输出，未开启 GPIO 输入通道，不能用 `gpio_get_level()` 自读；旧日志中的 `TXD=0` 是无效诊断值，不代表 TXD 实际被拉低。
9. 新固件每秒最多打印一次 CAN 错误分类。若 `ACK=1` 而 `BIT/FORM/STUFF=0`，说明 ESP32 帧已发出但电机板没有应答；若 `BIT=1`，则优先检查 TXD 到收发器、CANH/CANL 接线、终端和两端位时序。
10. 针对 `BIT=1`，固件会在启用 TWAI 前进行一次 10 us 的收发器本地回环检查，正常结果为 `Transceiver self-test RXD: idle=1 dominant=0 release=1 -> PASS`。若 dominant 仍为 1，则 SIT1042 没有把 GPIO5 的低电平转换到 CAN 总线并反馈至 RXD，应检查 GPIO5/TXD、GPIO6/RXD、5V、VIO/EN 和 S。SIT1042AQT/3 的 VIO 是数字接口电源且带欠压保护；若模块上的 R1 没有把 VIO 默认连接到 5V，必须在模块 `EN`（实际为 VIO）脚提供与 ESP32 I/O 匹配的 3.3V。若自检 PASS 但运行时仍 `BIT=1`，局部收发器正常，故障位于 CANH/CANL、电机板收发器或两端位时序。
11. 自检 FAIL 现在只作为 `LOCAL=WARN` 诊断，不再禁止 TWAI 发送。这样即使模块的本地回读行为与预期不同，ESP32 仍会发送 Ping，并通过真实总线的 ACK、BIT、FORM、STUFF 和 Bus-Off 状态继续定位；收到第一帧有效 STM32 状态报文后会自动清除本地收发器告警。

### 7.3 Flash 容量警告

DNESP32S3B 实际 Flash 为 16 MB，原工程镜像头配置为 2 MB，因此启动时出现 `Detected size(16384k) larger than ... (2048k)`。工程的 `CONFIG_ESPTOOLPY_FLASHSIZE` 已改为 16 MB，并写入 `sdkconfig.defaults`。这条警告与 CAN 故障无关，重新构建并烧录 bootloader 后应消失；当前仍使用单应用分区，不会因为容量配置修改而改变应用偏移。

### 7.1 本次 CAN OFFLINE / ERR 持续上涨原因与修复

原电机板代码在尚未收到 ESP32 报文时就每 20 ms 主动发送三帧遥测。如果两块板的启动顺序不同、ESP32 正在复位，或者总线暂时断开，电机板发送端会因长期收不到 ACK 进入 Bus-Off；旧代码没有恢复该状态。此时 ESP32 后续 Ping 同样无人 ACK，所以界面一直显示 `CAN OFFLINE`，`ERR` 持续上涨。

修复后：

- 电机板只有在收到有效 ESP32 Ping/命令、确认链路存在后才发送遥测。
- 电机板检测 Bus-Off，并通过停止/重启 FDCAN 自动恢复，再等待新的 Ping 建链。
- CAN 初始化偶发失败时由主循环每 500 ms 重试，不再进入 `Error_Handler()` 锁死整个固件。
- ESP32 仍每 100 ms 发送 Ping，因此任意一端重启后都能自动重新建立连接。
- ESP32 进入 Bus-Off 后暂停命令出队和 TX 等待，恢复完成后再继续发送；界面会明确显示 `CAN BUS-OFF / CHECK CAN WIRING`，避免驱动持续打印 `node is bus off`。

如果刷入修复后的两端固件后 ERR 仍持续上涨，这不再是启动次序造成的锁死，应按第 2、3、7 节检查物理层；当前两节点接法尤其要确认 ESP32 模块 SW1 已拨到 ON，且断电测得 CANH-CANL 约 60 Ω。

### 7.2 ESP32 黑屏并报告 main task stack overflow

完整界面会连续创建五个页面、图表、控件和触摸输入。ESP-IDF 原来的 `app_main` 栈只有 3584 字节，日志在 `LVGL display registered` 后出现 `A stack overflow in task main has been detected`，说明程序在界面创建过程中已经重启，因此不会执行到背光开启，表现为屏幕完全无显示。

工程已把 `CONFIG_ESP_MAIN_TASK_STACK_SIZE` 提高到 12288 字节，并在 `sdkconfig.defaults` 中保留同一设置。界面创建后只标记整屏需要更新，实际刷新由 LVGL 任务完成，以减少 `app_main` 的峰值栈占用。正常启动日志应继续出现触摸初始化信息以及 `Motor HMI started successfully`，且不再打印 stack overflow 和重启信息。

### 7.4 实际 CAN 调试记录（2026-08-02，已连接成功）

本次调试在没有万用表、示波器和 CAN 分析仪的条件下完成，主要依靠 ESP32 日志、STM32 USART2 诊断计数和两端源码/原理图交叉核对。

1. 初始现象是 ESP32 显示 `CAN OFFLINE`，发送计数变化但接收为 0。首先核对两端协议，确认均为经典 CAN、标准 11 位 ID、500 kbit/s、8 字节数据帧和小端序；命令 ID 为 `0x100`，遥测 ID 为 `0x180..0x182`，因此排除了协议和位速率不一致。
2. 核对硬件手册后确认 ESP32 CAN 实际使用 `J2/Port1`，其中 GPIO5 接收发器 TXD、GPIO6 接收发器 RXD；原 LVGL 页面曾错误显示为 Port2，容易导致按物理插座接错。页面和日志现已统一标记为 `J2/PORT1 GPIO5/6`。
3. 发现 ESP32 的收发器 GPIO 自检曾作为硬门禁：自检失败时 `motor_can_init()` 仍返回成功，但 TX 任务永久停止，造成界面显示已连接/正在连接而实际一个 Ping 都没有发送。修复后自检仅设置 `LOCAL=WARN`，真实 TWAI 收发和错误诊断始终继续运行。
4. STM32 端曾可能因无节点 ACK 而进入 Bus-Off。修复为只有收到有效 Ping/命令后才发送遥测，并在 Bus-Off 后停止/重启 FDCAN、等待新的 Ping，因此两块板可以任意顺序上电并自动重新建链。
5. 后续日志出现 `CAN error flags=0x02 ACK=0 BIT=1`，随后出现 `Bus-off ... starting recovery` 和 `CAN bus recovered`。这证明恢复状态机有效，但物理层仍存在发送位与回读位不一致。调试重点因此转向 J2/Port1、TXD/RXD不交叉、VCC/VIO、S低电平、共地、CANH/CANL同名连接以及两端120Ω终端。
6. 为无仪器调试增加了 `RXD=x LOCAL=PASS/WARN`：`LOCAL=WARN` 优先检查 ESP32 本地收发器供电/模式/数字引脚；`LOCAL=PASS` 且仍有 `BIT=1` 时优先检查 CANH/CANL、共地、终端和电机板侧；空闲时 `RXD=0` 表示总线可能被持续拉成显性。
7. 在修正软件门禁、Port1标识并完成物理链路检查后，CAN 已由用户实际确认连接成功。验收标准是 ESP32 显示 `CAN ONLINE`、RX/TX 均持续增长、ERR 不再周期性快速增长，STM32 能接收命令并回传 `0x180..0x182` 遥测。

保留结论：若以后再次看到周期性的 `BIT=1 → Bus-Off → recovered`，自动恢复本身没有故障；应优先复查物理层，而不是修改已经对齐的 500 kbit/s、ID 或数据格式。

## 8. 主要源码位置

- ESP32 CAN 驱动：`ESP32_LVGL/ESP32_LVGL/main/motor_can.c`
- ESP32 UART 驱动：`ESP32_LVGL/ESP32_LVGL/main/motor_uart.c`
- ESP32 UART/CAN 互斥分发：`ESP32_LVGL/ESP32_LVGL/main/motor_link.c`
- ESP32 CAN 协议：`ESP32_LVGL/ESP32_LVGL/main/motor_can_protocol.h`
- ESP32 界面：`ESP32_LVGL/ESP32_LVGL/main/motor_ui.c`
- 电机板 CAN 驱动：`MCSDK_FOC_MIX/Src/motor_can.c`
- 电机板 UART3 驱动：`MCSDK_FOC_MIX/Src/motor_uart.c`
- 电机板 CAN 协议：`MCSDK_FOC_MIX/Inc/motor_can_protocol.h`
- Qt/CAN 公共控制入口：`MCSDK_FOC_MIX/Src/foc_app_protocol.c`
