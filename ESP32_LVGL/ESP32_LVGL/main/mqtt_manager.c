#include "mqtt_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#define MQTT_MANAGER_COMMAND_QUEUE_LENGTH 4U
#define MQTT_MANAGER_WORKER_STACK_SIZE 4096U
#define MQTT_MANAGER_WORKER_PRIORITY 4U

typedef enum
{
    MQTT_MANAGER_COMMAND_CONNECT = 0,
    MQTT_MANAGER_COMMAND_DISCONNECT,
} mqtt_manager_command_type_t;

typedef struct
{
    mqtt_manager_command_type_t type;
    char broker_uri[MQTT_MANAGER_URI_MAX_LEN + 1U];
} mqtt_manager_command_t;

static const char *TAG = "MQTT_MANAGER";

static SemaphoreHandle_t s_lock;
static QueueHandle_t s_command_queue;
static esp_mqtt_client_handle_t s_client;
static mqtt_manager_snapshot_t s_snapshot;
static char s_active_uri[MQTT_MANAGER_URI_MAX_LEN + 1U];
static mqtt_manager_message_callback_t s_message_callback;
static void *s_message_callback_context;

static void mqtt_manager_lock(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void mqtt_manager_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static void mqtt_manager_set_status_locked(const char *status)
{
    strlcpy(s_snapshot.status, status, sizeof(s_snapshot.status));
    s_snapshot.revision++;
}

static void mqtt_manager_copy_event_text(
    char *destination,
    size_t destination_size,
    const char *source,
    int source_length)
{
    if (destination_size == 0U) {
        return;
    }
    if (source == NULL || source_length <= 0) {
        destination[0] = '\0';
        return;
    }
    size_t length = (size_t)source_length;
    if (length >= destination_size) {
        length = destination_size - 1U;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
}

static void mqtt_manager_event_handler(
    void *handler_argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)handler_argument;
    (void)event_base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_BEFORE_CONNECT:
        mqtt_manager_lock();
        s_snapshot.connecting = true;
        mqtt_manager_set_status_locked("Connecting to broker...");
        mqtt_manager_unlock();
        break;

    case MQTT_EVENT_CONNECTED:
        mqtt_manager_lock();
        s_snapshot.connecting = false;
        s_snapshot.connected = true;
        mqtt_manager_set_status_locked("Connected - RX topic ready");
        mqtt_manager_unlock();
        if (event != NULL) {
            (void)esp_mqtt_client_subscribe(
                event->client,
                MQTT_MANAGER_TEST_RX_TOPIC,
                1);
            (void)esp_mqtt_client_subscribe(
                event->client,
                MQTT_MANAGER_CONTROL_TOPIC,
                1);
        }
        ESP_LOGI(TAG, "Connected to %s", s_active_uri);
        break;

    case MQTT_EVENT_DISCONNECTED:
        mqtt_manager_lock();
        s_snapshot.connected = false;
        s_snapshot.connecting = true;
        mqtt_manager_set_status_locked("Disconnected - retrying");
        mqtt_manager_unlock();
        break;

    case MQTT_EVENT_PUBLISHED:
        mqtt_manager_lock();
        s_snapshot.transmitted_messages++;
        mqtt_manager_set_status_locked("Test message delivered");
        mqtt_manager_unlock();
        break;

    case MQTT_EVENT_DATA: {
        if (event == NULL) {
            break;
        }
        bool message_complete = false;
        mqtt_manager_message_callback_t callback = NULL;
        void *callback_context = NULL;
        char completed_topic[MQTT_MANAGER_TOPIC_MAX_LEN + 1U];
        char completed_payload[MQTT_MANAGER_PAYLOAD_MAX_LEN + 1U];
        mqtt_manager_lock();
        if (event->current_data_offset == 0) {
            mqtt_manager_copy_event_text(
                s_snapshot.last_topic,
                sizeof(s_snapshot.last_topic),
                event->topic,
                event->topic_len);
            s_snapshot.last_payload[0] = '\0';
        }
        if (event->data != NULL && event->data_len > 0 &&
            event->current_data_offset <
                (int)sizeof(s_snapshot.last_payload) - 1) {
            size_t offset = (size_t)event->current_data_offset;
            size_t length = (size_t)event->data_len;
            const size_t available =
                sizeof(s_snapshot.last_payload) - 1U - offset;
            if (length > available) {
                length = available;
            }
            memcpy(
                s_snapshot.last_payload + offset,
                event->data,
                length);
            s_snapshot.last_payload[offset + length] = '\0';
        }
        if (event->current_data_offset + event->data_len >=
            event->total_data_len) {
            s_snapshot.received_messages++;
            mqtt_manager_set_status_locked("Message received");
            strlcpy(
                completed_topic,
                s_snapshot.last_topic,
                sizeof(completed_topic));
            strlcpy(
                completed_payload,
                s_snapshot.last_payload,
                sizeof(completed_payload));
            callback = s_message_callback;
            callback_context = s_message_callback_context;
            message_complete = true;
        }
        mqtt_manager_unlock();
        if (message_complete && callback != NULL) {
            callback(
                completed_topic,
                completed_payload,
                callback_context);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        mqtt_manager_lock();
        s_snapshot.connected = false;
        if (event != NULL && event->error_handle != NULL) {
            snprintf(
                s_snapshot.status,
                sizeof(s_snapshot.status),
                "MQTT error type %d",
                (int)event->error_handle->error_type);
        } else {
            strlcpy(
                s_snapshot.status,
                "MQTT connection error",
                sizeof(s_snapshot.status));
        }
        s_snapshot.revision++;
        mqtt_manager_unlock();
        break;

    default:
        break;
    }
}

static void mqtt_manager_stop_client(void)
{
    mqtt_manager_lock();
    esp_mqtt_client_handle_t client = s_client;
    s_client = NULL;
    mqtt_manager_unlock();

    if (client != NULL) {
        (void)esp_mqtt_client_stop(client);
        (void)esp_mqtt_client_destroy(client);
    }
}

static void mqtt_manager_worker(void *argument)
{
    (void)argument;
    mqtt_manager_command_t command;

    while (true) {
        if (xQueueReceive(
                s_command_queue,
                &command,
                portMAX_DELAY) != pdTRUE) {
            continue;
        }

        mqtt_manager_stop_client();

        if (command.type == MQTT_MANAGER_COMMAND_DISCONNECT) {
            mqtt_manager_lock();
            s_snapshot.connecting = false;
            s_snapshot.connected = false;
            mqtt_manager_set_status_locked("Disconnected");
            mqtt_manager_unlock();
            continue;
        }

        strlcpy(s_active_uri, command.broker_uri, sizeof(s_active_uri));
        const esp_mqtt_client_config_t configuration = {
            .broker.address.uri = s_active_uri,
            .credentials.client_id = "esp32s3-motor-hmi",
            .session.keepalive = 30,
            .network.reconnect_timeout_ms = 3000,
        };
        esp_mqtt_client_handle_t client =
            esp_mqtt_client_init(&configuration);
        if (client == NULL) {
            mqtt_manager_lock();
            s_snapshot.connecting = false;
            mqtt_manager_set_status_locked("MQTT client init failed");
            mqtt_manager_unlock();
            continue;
        }

        esp_err_t result = esp_mqtt_client_register_event(
            client,
            ESP_EVENT_ANY_ID,
            mqtt_manager_event_handler,
            NULL);
        if (result == ESP_OK) {
            mqtt_manager_lock();
            s_client = client;
            mqtt_manager_unlock();
            result = esp_mqtt_client_start(client);
        }
        if (result != ESP_OK) {
            mqtt_manager_lock();
            if (s_client == client) {
                s_client = NULL;
            }
            s_snapshot.connecting = false;
            snprintf(
                s_snapshot.status,
                sizeof(s_snapshot.status),
                "MQTT start failed: %s",
                esp_err_to_name(result));
            s_snapshot.revision++;
            mqtt_manager_unlock();
            (void)esp_mqtt_client_destroy(client);
        }
    }
}

esp_err_t mqtt_manager_init(void)
{
    if (s_lock != NULL) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    s_command_queue = xQueueCreate(
        MQTT_MANAGER_COMMAND_QUEUE_LENGTH,
        sizeof(mqtt_manager_command_t));
    if (s_lock == NULL || s_command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.initialized = true;
    strlcpy(
        s_snapshot.status,
        "Ready - enter broker URI",
        sizeof(s_snapshot.status));

    if (xTaskCreate(
            mqtt_manager_worker,
            "mqtt_manager",
            MQTT_MANAGER_WORKER_STACK_SIZE,
            NULL,
            MQTT_MANAGER_WORKER_PRIORITY,
            NULL) != pdPASS) {
        s_snapshot.initialized = false;
        strlcpy(
            s_snapshot.status,
            "MQTT worker allocation failed",
            sizeof(s_snapshot.status));
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t mqtt_manager_connect_async(const char *broker_uri)
{
    if (broker_uri == NULL ||
        (strncmp(broker_uri, "mqtt://", 7U) != 0 &&
         strncmp(broker_uri, "mqtts://", 8U) != 0) ||
        strlen(broker_uri) > MQTT_MANAGER_URI_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_command_t command = {
        .type = MQTT_MANAGER_COMMAND_CONNECT,
    };
    strlcpy(command.broker_uri, broker_uri, sizeof(command.broker_uri));

    mqtt_manager_lock();
    if (!s_snapshot.initialized) {
        mqtt_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_snapshot.connecting = true;
    s_snapshot.connected = false;
    strlcpy(
        s_snapshot.broker_uri,
        broker_uri,
        sizeof(s_snapshot.broker_uri));
    mqtt_manager_set_status_locked("Connection queued");
    mqtt_manager_unlock();

    if (xQueueSend(s_command_queue, &command, 0U) != pdTRUE) {
        mqtt_manager_lock();
        s_snapshot.connecting = false;
        mqtt_manager_set_status_locked("MQTT command queue busy");
        mqtt_manager_unlock();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t mqtt_manager_disconnect_async(void)
{
    const mqtt_manager_command_t command = {
        .type = MQTT_MANAGER_COMMAND_DISCONNECT,
    };
    if (s_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueSend(s_command_queue, &command, 0U) == pdTRUE
        ? ESP_OK
        : ESP_ERR_TIMEOUT;
}

esp_err_t mqtt_manager_publish(
    const char *topic,
    const char *payload)
{
    if (topic == NULL || topic[0] == '\0' || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_lock();
    if (!s_snapshot.connected || s_client == NULL) {
        mqtt_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    const int message_id = esp_mqtt_client_enqueue(
        s_client,
        topic,
        payload,
        0,
        1,
        0,
        true);
    if (message_id >= 0) {
        mqtt_manager_set_status_locked("Test message queued");
    }
    mqtt_manager_unlock();
    return message_id >= 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_manager_publish_qos0(
    const char *topic,
    const char *payload)
{
    if (topic == NULL || topic[0] == '\0' || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_lock();
    if (!s_snapshot.connected || s_client == NULL) {
        mqtt_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    const int message_id = esp_mqtt_client_enqueue(
        s_client,
        topic,
        payload,
        0,
        0,
        0,
        true);
    mqtt_manager_unlock();
    return message_id >= 0 ? ESP_OK : ESP_FAIL;
}

void mqtt_manager_set_message_callback(
    mqtt_manager_message_callback_t callback,
    void *context)
{
    mqtt_manager_lock();
    s_message_callback = callback;
    s_message_callback_context = context;
    mqtt_manager_unlock();
}

void mqtt_manager_get_snapshot(mqtt_manager_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    mqtt_manager_lock();
    *snapshot = s_snapshot;
    mqtt_manager_unlock();
}
