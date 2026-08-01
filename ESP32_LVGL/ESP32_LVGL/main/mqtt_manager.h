#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define MQTT_MANAGER_URI_MAX_LEN 95U
#define MQTT_MANAGER_TOPIC_MAX_LEN 63U
#define MQTT_MANAGER_PAYLOAD_MAX_LEN 127U

typedef struct
{
    bool initialized;
    bool connecting;
    bool connected;
    uint32_t revision;
    uint32_t transmitted_messages;
    uint32_t received_messages;
    char broker_uri[MQTT_MANAGER_URI_MAX_LEN + 1U];
    char status[64];
    char last_topic[MQTT_MANAGER_TOPIC_MAX_LEN + 1U];
    char last_payload[MQTT_MANAGER_PAYLOAD_MAX_LEN + 1U];
} mqtt_manager_snapshot_t;

esp_err_t mqtt_manager_init(void);
esp_err_t mqtt_manager_connect_async(const char *broker_uri);
esp_err_t mqtt_manager_disconnect_async(void);
esp_err_t mqtt_manager_publish(
    const char *topic,
    const char *payload);
void mqtt_manager_get_snapshot(mqtt_manager_snapshot_t *snapshot);

#endif /* MQTT_MANAGER_H */
