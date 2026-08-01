# MCSDK_FOC_MIX

该目录是位置控制与速度控制的独立合成工程，基于已经调通的
`MCSDK_FOC_pos` 固件创建，并保留 `MCSDK_FOC_Speed` 中相同的 MCSDK
原生速度 PI 参数。

## 目录

- `MDK-ARM/MCSDK_FOC_MIX.uvprojx`：STM32/MCSDK 固件工程。
- `QtFocMixControl/`：Qt 6 速度/位置合成控制台。
- `Inc/foc_app_protocol.h`、`Src/foc_app_protocol.c`：Qt 与固件之间的
  MCP 用户命令协议及模式切换逻辑。
- `docs/`：之前的串口/MCP失败原因、角度测量修正和直接位置环修正记录。
  其中 `UI_CONTROL_FIX.md` 记录速度滑块和位置圆盘的遥测竞争修正。

## 控制模式

- 速度模式（mode 0）：关闭位置调节器，使用 MCSDK 原生速度 PI 输出
  转矩参考。
- 位置模式（mode 1）：关闭速度控制接管，使用 MCSDK 原生位置 PID
  直接输出转矩参考；没有引入额外串级位置环。
- 固件上电默认位置模式。启动并完成编码器对齐后，位置目标锁定为当时
  的实测位置。
- 切换到速度模式时先发送 0 RPM；切回位置模式时锁定切换瞬间的实测
  角度并清除位置 PID 历史，降低切换冲击。

## 关键参数

- USART2 / ASPEP：1,843,200 baud，8-N-1。
- MCP 用户命令头：`0x0101`。
- 协议版本：2。
- 速度范围：-2600 至 +2600 RPM。
- 位置 PID 输出电流限制：±2.0 A。
- Qt 的位置页面只显示 0 至 360°，向固件发送时会选择距离当前位置最近
  的等效多圈目标，避免跨越 0°/360° 时绕远路。

## 编译说明

本次合成仅完成源码和工程配置，没有执行固件或 Qt 编译。请先编译并
烧录 `MDK-ARM/MCSDK_FOC_MIX.uvprojx`，再自行构建 `QtFocMixControl`。
