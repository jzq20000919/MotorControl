# MCSDK 位置控制与 Qt 上位机

## 上次修改为何使位置控制变得可用

上次有效修改的核心不是改动原速度环或电流环，而是撤销了错误的混合/转矩保持方案，
恢复成标准串级位置伺服：

1. 恢复原工程的速度单位、速度 PI、编码器速度滤波和 Iq/Id 电流 PI。
2. 位置模块只计算位置误差并调用 `MC_ProgramSpeedRampMotor1_F()` 写入 RPM。
3. 使用“位置 PD → 原速度 PI → 原电流 PI”，不切换转矩模式、不直接写 Iq，
   也不清除速度 PI 的积分状态。
4. 位置环以 1 kHz 运行，取消原先人为加入的 500 RPM 限速、加速度斜率、到位死区
   和 ±5°/±7°模式切换，仅保留电机 2600 RPM 硬件边界。
5. 位置积分设为 0，避免与原速度 PI 的积分形成双积分低频振荡；位置微分直接使用
   已滤波的实测转速，提供接近目标时的阻尼。
6. Qt 目标角度输入不再被 20 Hz 遥测覆盖，按钮会发送 MCP 位置命令 3。

该可用基线的全部 `Src/*.c` 已提交到本地 Git，提交号为 `a1b59ff`。

## 本次位置环改进

- 已撤销上一版自行累加单圈角度的实现。MCSDK 的 `SPD_GetMecAngle()` 返回值本身就是
  连续32位机械角 `wMecAngle`，重复展开会在负向最短路径进入310°～360°时产生
  不一致状态。
- 位置反馈使用 MCSDK 原生连续32位机械角，对外显示才归一化到0～360°。
- 单圈圆盘控制在每个位置环周期重新计算圆周最短误差，误差始终保持在
  -180°～+180°，避免跨圈或大误差后继续旋转。
- 恢复引入TinyFOC前的串级位置控制：位置PD输出RPM参考，再使用原MCSDK速度PI和
  原MCSDK电流PI。参数为Kp=40 RPM/°、Ki=0、Kd=0.08。
- 位置模式不直接写Iq，不切换转矩模式，不使用TinyFOC位置环。
- 本次仍未修改任何既有速度环或电流环参数。

## 工程结构

- `Src/position_control.c` / `Inc/position_control.h`
  - 独立的标准位置外环。
  - 位置模式使用“位置 PD → 原速度PI → 原电流PI”。
  - 速度模式继续使用原MCSDK速度PI和电流PI。
  - 位置/速度模式在固件侧互斥。
- `Src/foc_app_protocol.c` / `Inc/foc_app_protocol.h`
  - 复用 USART2 上现有的 MCSDK ASPEP/MCP。
  - 使用 MCP User Callback 0，不直接抢占 HAL UART 或 DMA。
- `Src/mc_app_hooks.c`
  - Boot Hook 初始化位置环和 MCP 回调。
  - Post Medium Frequency Hook 周期执行位置环。
- `QtFocControl`
  - Qt 6 Widgets 上位机。
  - Windows 串口、ASPEP 握手、MCP 用户命令、实时遥测和控制界面。

## 与当前硬件配置的对应关系

- 电机资料给出的最大转速为 2600 RPM、极对数为 7；固件和 Qt 转速范围均按
  ±2600 RPM 限制。
- MT6701 资料给出的 ABZ 默认分辨率为 1024 线；当前 MCSDK 工程使用
  TIM3/PC6/PC7 的正交编码器输入，`M1_ENCODER_PPR` 为 1024，与之匹配。
- 当前工程没有使用 Z 相，也没有读取 MT6701 的 I2C/SSI 绝对角度。因此圆盘显示的是
  当前上电会话内的单圈相对机械角；“当前位置置零”只在本次运行中有效，断电后不会
  保留绝对零点。如需掉电后保持机械绝对位置，应另行接入 MT6701 的 I2C/SSI 角度。

## 当前串口设置

固件和 Qt 默认均为 **115200, 8-N-1**，与当前 Workbench/CubeMX 配置保持一致。
原工程使用 1843200；为提高 ST-Link VCP 和不同 USB-UART 驱动的兼容性，现已统一
降低为 115200。该速率足够承载当前 20 Hz 遥测。
开发板使用 PB3（TX）和 PB4（RX），通信复用 MCSDK
已有的 DMA + ASPEP/MCP，不应再用 HAL 串口接收代码抢占该外设。

如需改用其他波特率：

1. 在 CubeMX/`.ioc` 中修改 USART2 波特率并重新生成，或同步修改
   `Src/main.c` 的 `huart2.Init.BaudRate`。
2. 同步更新 `Inc/foc_app_protocol.h` 中用于记录工程约定的
   `FOC_APP_UART_BAUD_RATE`。
3. 在 Qt 连接区选择相同波特率（下拉框也支持直接输入）。
4. 重新编译并烧录 STM32 固件。

波特率选择无法在线改变 MCU 当前波特率；两端必须在连接前一致。

## 编译和运行 Qt

```powershell
cd QtFocControl
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/mingw_64
cmake --build build
.\build\QtFocControl.exe
```

也可以直接运行已部署好 Qt 运行库的
`QtFocControl/dist_fixed/QtFocControl.exe`。

程序连接流程：

1. 选择串口和波特率。
2. 点击“连接”。黄色状态表示串口已打开、正在进行 ASPEP 握手；绿色表示协议连接完成。
3. 必要时点击“故障复位”，再点击“启动电机”。
4. 选择“转速控制”使用 ±2600 RPM 滑条；选择“位置控制”后可点击或拖动圆盘，
   也可在“输入目标角度”中输入 0～359.99°，再点击“转到该角度”。
   输入框内容不会再被周期遥测覆盖；点击按钮后输出框会显示“提交位置目标”。

当前 MCSDK 编码器启动流程会以 2 A 执行约 700 ms 的转子对齐。首次调试应让机构
处于可安全移动状态，并准备随时断开母线电源。

## 串口输出与通信排障

Qt 窗口底部的“串口输出 / 协议诊断”区域会显示：

- 实际打开的 COM 口、波特率和 8-N-1 参数
- `TX BEACON`、`RX RAW`、`TX PING` 和 `TX MCP CMD` 的十六进制字节
- ASPEP 帧类型、长度、CRC 和握手阶段
- MCP 返回状态以及遥测解析结果

“清空信息”仅清空显示内容，不会关闭串口。正常连接应看到 BEACON、PING 和
“遥测接收正常”。用户点击模式、启动、停止等操作时，才会额外显示
`TX MCP CMD ... HEADER` 和约 2 ms 后的 `TX MCP CMD ... PAYLOAD`。

连接成功后，20 Hz 周期遥测的 TX/RX 原始帧和逐帧成功消息会自动静默。输出框只继续
显示用户操作、连接变化和错误；连续遥测错误仅显示第 1 次及每第 20 次，避免大量刷新。

MCSDK 6.4.2 固件先用 DMA 接收 4 字节 ASPEP 头，再由 2 kHz SysTick 检查完成状态并
把 DMA 切换到负载缓冲。USART RX FIFO 在当前工程中处于关闭状态，因此 Qt 对数据帧
采用“先发头、等待 2 ms、再发负载”的方式，避免连续发送时丢失 MCP 负载。

- 只有 `TX BEACON`、完全没有 `RX RAW`：确认已经烧录本次生成的 115200 固件，
  检查 COM 口是否为目标 ST-Link/USB-UART，并检查 TX、RX、GND。
- 有 `RX RAW`，但持续报告头 CRC 无效：两端波特率不一致或线路存在干扰。
- MCP 状态为 `0x0D`：板上仍是未注册用户回调的旧固件，需要重新烧录。
- ASPEP 握手成功但 MCP 超时：检查 `MC_APP_BootHook()` 是否调用
  `FocAppProtocol_Init()`，并确认 `foc_app_protocol.c` 已加入固件工程。

## 位置环参数与移植

通常只需修改 `Src/position_control.c` 顶部的
`PositionControl_DefaultConfig`：

| 参数 | 说明 | 当前值 |
|---|---|---:|
| `executionFrequencyHz` | MCSDK Medium Frequency Hook 频率 | 1000 Hz |
| `loopDivider` | 位置环分频 | 1（1 kHz） |
| `positionFeedbackSign` | 位置反馈方向，反向时改为 -1 | 1 |
| `kpRpmPerDegree` | 位置误差到转速给定的比例增益 | 40 RPM/° |
| `kiRpmPerDegreeSecond` | 位置积分；内环已有积分，因此关闭 | 0 |
| `kdRpmSecondPerDegree` | 基于实测转速的位置微分阻尼 | 0.08 |
| `motorSpeedLimitRpm` | 速度模式的电机硬件转速上限 | 2600 RPM |
| `speedRampDurationMs` | 转速滑条命令的 MCSDK 斜坡时间 | 100 ms |

位置环调用 `MC_ProgramSpeedRampMotor1_F()` 给出RPM参考，保留原MCSDK速度PI的积分保持能力
和原Iq/Id电流PI。原速度环、电流环参数均不修改。

移植到其他由 MCSDK Workbench 生成的单电机编码器速度工程：

1. 复制 `position_control.c/.h`。
2. 将两个文件加入目标 IDE 工程。
3. 在 Boot Hook 调用 `PositionControl_Init()`。
4. 在 1 kHz Medium Frequency Post Hook 调用 `PositionControl_Tick()`。
5. 修改上表参数；如果目标速度环频率不是 1 kHz，同时修改
   `executionFrequencyHz`。若目标工程的正转使编码器角度减小，将
   `positionFeedbackSign` 改为 `-1`。

模块依赖 MCSDK 常见符号 `ENCODER_M1` 和 Motor 1 API。无编码器的速度工程必须
先提供可靠机械位置反馈，不能仅靠调参获得静态位置控制。

### 修改位置 PID

Qt 不提供 PID 在线修改。Kp、Ki、Kd 位于
`Src/position_control.c` 顶部的 `PositionControl_DefaultConfig`：

```c
.kpRpmPerDegree = 40.0F,
.kiRpmPerDegreeSecond = 0.0F,
.kdRpmSecondPerDegree = 0.08F,
```

修改后必须重新编译并烧录 STM32 固件。

建议空载、低速整定：

1. 当前参数针对文件夹内的电机资料和现有编码器工程设置；首次仍应空载确认方向。
2. 位置环输出RPM参考，并使用已调好的MCSDK速度环和电流环。
3. 速度环和电流环属于原工程既有参数，位置模块不修改它们。
4. `IQMAX_A=2` 是资料中的最大电流，不是连续保持电流；资料给出的额定连续电流为
   0.5 A，持续外力使 Iq 长时间超过 0.5 A 会增加电机温升。

如果误差持续增大而不是收敛，应立即停止。这是方向/相序问题，不应继续增大 PID。

## 遥测内容

Qt 以 20 Hz 轮询以下数据：

- Iq、Id 测量值（A）
- Iq Ref、Id Ref（A）
- Uq、Ud（V，使用实时母线电压换算）
- 转速参考与转速测量（RPM）
- 目标位置与当前位置（机械角度）
- MCSDK 状态、当前故障和历史故障字

## 首次上电安全建议

先脱开机械负载确认编码器方向与电机方向一致。若位置误差增大、转子持续加速，
应立即停止电机，并将
`positionFeedbackSign` 改为 `-1` 后重新编译；不要通过增大位置环增益掩盖方向错误。
