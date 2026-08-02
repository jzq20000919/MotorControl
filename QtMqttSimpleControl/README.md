# QtMqttSimpleControl

独立的 Qt 6 Widgets MQTT 命令发送器。它只连接 Mosquitto 并向
`motor/control/command` 发布命令，不订阅电机遥测主题。窗口会显示本程序
与 MQTT Broker 的连接状态，但不显示 ESP32 或 STM32 的反馈数据。

## 功能

- 输入 Broker IPv4 地址和端口并连接或断开。
- 红/黄/绿状态灯显示 MQTT 离线、连接中和在线状态。
- 使用滑块发送速度命令，范围 `-2600..2600 RPM`。
- 使用滑块发送单圈位置命令，范围 `0.00..359.99°`。
- 发送电机启动和停止命令。
- MQTT 3.1.1、QoS 1，20 秒 Keep Alive，每 10 秒发送 PINGREQ。
- TCP 强制使用 `QNetworkProxy::NoProxy`，不继承 Windows 系统代理。

发送格式：

```json
{"id":1,"cmd":"set_speed","value":500}
```

```json
{"id":2,"cmd":"set_position","value":9000}
```

位置的 `value` 单位为 0.01 度，因此 `90.00°` 发送为 `9000`。

## 构建

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/mingw_64
cmake --build build
```

启动 Mosquitto 后，填写电脑在局域网中的 IPv4 地址（不要填写
`127.0.0.1`，除非 Broker 和本程序在同一台电脑且只做本机测试）。
