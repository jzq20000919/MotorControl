#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define MQTT_MANAGER_URI_MAX_LEN 95U
#define MQTT_MANAGER_TOPIC_MAX_LEN 63U
#define MQTT_MANAGER_PAYLOAD_MAX_LEN 511U
#define MQTT_MANAGER_TEST_RX_TOPIC "motor/hmi/test/rx"
#define MQTT_MANAGER_CONTROL_TOPIC "motor/control/command"

typedef void (*mqtt_manager_message_callback_t)(
    const char *topic,
    const char *payload,
    void *context);

typedef struct
{
    bool initialized;
    bool connecting;
    bool connected;
    uint32_t revision;
    uint32_t transmitted_messages;
    uint32_t received_messages;
    char broker_uri[MQTT_MANAGER_URI_MAX_LEN + 1U];
    char status[96];
    char last_topic[MQTT_MANAGER_TOPIC_MAX_LEN + 1U];
    char last_payload[MQTT_MANAGER_PAYLOAD_MAX_LEN + 1U];
} mqtt_manager_snapshot_t;

/** @brief 初始化互斥锁、队列和 MQTT 管理工作任务。 */
esp_err_t mqtt_manager_init(void);
/** @brief 为 @p broker_uri 排队一个非阻塞连接请求。 */
esp_err_t mqtt_manager_connect_async(const char *broker_uri);
/** @brief 排队一个非阻塞 MQTT 客户端断开请求。 */
esp_err_t mqtt_manager_disconnect_async(void);
/** @brief 使用管理器默认服务质量策略发布消息。 */
esp_err_t mqtt_manager_publish(
    const char *topic,
    const char *payload);
/** @brief 发布一条即发即弃的 QoS 0 消息。 */
esp_err_t mqtt_manager_publish_qos0(
    const char *topic,
    const char *payload);
/** @brief 注册接收完整 MQTT 入站消息的应用回调。 */
void mqtt_manager_set_message_callback(
    mqtt_manager_message_callback_t callback,
    void *context);
/** @brief 复制最新的线程安全 MQTT 连接与流量快照。 */
void mqtt_manager_get_snapshot(mqtt_manager_snapshot_t *snapshot);

#endif /* MQTT_MANAGER_H */
