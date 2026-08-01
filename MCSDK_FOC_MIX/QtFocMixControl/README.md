# Qt 无线电机控制台

该程序现通过 MQTT 与 ESP32-S3 通信，电脑端不再打开 STM32 串口。

通信路径为：

```text
QtFocMixControl --Wi-Fi/MQTT--> Mosquitto <--Wi-Fi/MQTT-- ESP32-S3
                                                        |
                                                        +-- USART/CAN --> STM32
```

## 使用方法

1. 让电脑与 ESP32-S3 连接到同一个 Wi-Fi。
2. 使用仓库根目录的 `mosquitto-dev.conf` 启动 Mosquitto。该配置监听局域网的 TCP 1883 端口，仅用于开发测试。
3. 在 ESP32 的 MQTT 页面中把 Broker 设置为电脑的 WLAN IPv4 地址并连接。
4. 启动 Qt 程序，在顶部 Broker 输入同一个电脑 WLAN IPv4 地址（本机当前为 `192.168.10.4`），端口填写 `1883`。
5. 点击“连接”。黄色表示 Qt 已连接 Mosquitto、正在等待 ESP32/STM32；绿色表示 ESP32 遥测正常且 STM32 链路在线。

不要在 ESP32 中填写 `127.0.0.1`，该地址指向 ESP32 自己而不是电脑。

## MQTT 主题

| 方向 | Topic | QoS | 用途 |
| --- | --- | ---: | --- |
| Qt → ESP32 | `motor/control/command` | 1 | 模式、速度、位置、启停与故障复位命令 |
| ESP32 → Qt | `motor/control/telemetry` | 0 | 100 ms 周期的电机遥测与链路状态 |
| ESP32 → Qt | `motor/control/ack` | 1 | 控制命令执行结果 |

MQTTX 可以订阅 `motor/control/#` 检查无线控制的命令、应答和遥测。

## 构建

需要 Qt 6 的 `Widgets` 和 `Network` 模块：

```powershell
cmake -S . -B build
cmake --build build -j 8
```

项目内置了轻量 MQTT 3.1.1 客户端，因此不依赖额外的 Qt MQTT 模块。
