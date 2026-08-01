#ifndef MQTT_MOTOR_GATEWAY_H
#define MQTT_MOTOR_GATEWAY_H

#include "esp_err.h"

#define MQTT_MOTOR_TELEMETRY_TOPIC "motor/control/telemetry"
#define MQTT_MOTOR_ACK_TOPIC       "motor/control/ack"

esp_err_t mqtt_motor_gateway_init(void);

#endif /* MQTT_MOTOR_GATEWAY_H */
