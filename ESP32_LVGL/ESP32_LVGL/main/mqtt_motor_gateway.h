#ifndef MQTT_MOTOR_GATEWAY_H
#define MQTT_MOTOR_GATEWAY_H

#include "esp_err.h"

#define MQTT_MOTOR_TELEMETRY_TOPIC "motor/control/telemetry"
#define MQTT_MOTOR_ACK_TOPIC       "motor/control/ack"

/**
 * @brief 启动 MQTT 到 motor_link 的命令网关及遥测发布器。
 * @return 工作任务和 MQTT 回调注册成功时返回 ESP_OK。
 */
esp_err_t mqtt_motor_gateway_init(void);

#endif /* MQTT_MOTOR_GATEWAY_H */
