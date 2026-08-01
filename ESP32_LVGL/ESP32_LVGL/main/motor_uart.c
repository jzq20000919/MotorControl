#include "motor_uart.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MOTOR_UART_PORT                 UART_NUM_1
/* Port2 is dedicated to the UART link. Port1 (GPIO5/6) is reserved for CAN. */
#define MOTOR_UART_TX_GPIO              GPIO_NUM_18
#define MOTOR_UART_RX_GPIO              GPIO_NUM_8
#define MOTOR_UART_RX_BUFFER_SIZE       (1024U)
#define MOTOR_UART_TX_BUFFER_SIZE       (1024U)
#define MOTOR_UART_CONTROL_QUEUE_SIZE   (8U)
#define MOTOR_UART_LINK_TIMEOUT_MS      (300U)
#define MOTOR_UART_TX_TASK_PERIOD_MS    (2U)
#define MOTOR_UART_PING_PERIOD_MS       (100U)

typedef struct
{
    MotorUart_Command_t command;
    int32_t value;
} motor_uart_request_t;

typedef struct
{
    uint8_t buffer[MOTOR_UART_MAX_FRAME_SIZE];
    uint8_t length;
} motor_uart_parser_t;

static const char *TAG = "MOTOR_UART";
static QueueHandle_t s_control_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static motor_uart_snapshot_t s_snapshot;
static motor_uart_parser_t s_parser;
static uint8_t s_command_sequence;
static uint32_t s_last_telemetry_ms;
static int16_t s_latest_speed_rpm;
static uint16_t s_latest_position_cdeg;
static bool s_speed_dirty;
static bool s_position_dirty;
static bool s_control_enabled;
static uint32_t s_requested_baud_rate;
static bool s_reconnect_requested;
static bool s_force_ping;
static TaskHandle_t s_rx_task;
static TaskHandle_t s_tx_task;
static bool s_initialized;

static uint32_t motor_uart_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint16_t motor_uart_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static void motor_uart_apply_reconnect(void)
{
    uint32_t baud_rate;

    portENTER_CRITICAL(&s_lock);
    if (!s_reconnect_requested) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    baud_rate = s_requested_baud_rate;
    s_reconnect_requested = false;
    portEXIT_CRITICAL(&s_lock);

    /*
     * The RX task owns the parser, so resetting it here cannot race with byte
     * parsing. The TX task is held by snapshot.reconnecting while the UART
     * divider and buffers are changed.
     */
    (void)uart_wait_tx_done(MOTOR_UART_PORT, pdMS_TO_TICKS(20));
    const esp_err_t result =
        uart_set_baudrate(MOTOR_UART_PORT, baud_rate);
    if (result == ESP_OK) {
        (void)uart_flush_input(MOTOR_UART_PORT);
        memset(&s_parser, 0, sizeof(s_parser));
        xQueueReset(s_control_queue);

        portENTER_CRITICAL(&s_lock);
        s_snapshot.link_active = false;
        s_snapshot.reconnecting = false;
        s_snapshot.baud_rate = baud_rate;
        s_snapshot.reconnect_count++;
        s_last_telemetry_ms = 0U;
        s_speed_dirty = false;
        s_position_dirty = false;
        s_force_ping = true;
        portEXIT_CRITICAL(&s_lock);

        ESP_LOGI(
            TAG,
            "UART reconnected at %lu baud",
            (unsigned long)baud_rate);
    } else {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.link_active = false;
        s_snapshot.reconnecting = false;
        s_snapshot.reconnect_errors++;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGE(
            TAG,
            "UART reconnect at %lu baud failed: %s",
            (unsigned long)baud_rate,
            esp_err_to_name(result));
    }
}

static esp_err_t motor_uart_transmit(
    MotorUart_Command_t command,
    int32_t value)
{
    if (command != MOTOR_UART_CMD_NOP &&
        command != MOTOR_UART_CMD_PING) {
        bool enabled;
        portENTER_CRITICAL(&s_lock);
        enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);
        if (!enabled) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    uint8_t frame[
        MOTOR_UART_FRAME_OVERHEAD +
        MOTOR_UART_COMMAND_PAYLOAD_SIZE] = {0};
    const uint8_t sequence = s_command_sequence++;
    uint16_t crc;
    int written;

    frame[0] = MOTOR_UART_SOF0;
    frame[1] = MOTOR_UART_SOF1;
    frame[2] = MOTOR_UART_PROTOCOL_VERSION;
    frame[3] = MOTOR_UART_FRAME_COMMAND;
    frame[4] = sequence;
    frame[5] = MOTOR_UART_COMMAND_PAYLOAD_SIZE;
    frame[6] = (uint8_t)command;
    MotorUart_WriteS32(&frame[7], value);
    crc = motor_uart_crc16(
        &frame[2], 4U + MOTOR_UART_COMMAND_PAYLOAD_SIZE);
    MotorUart_WriteU16(
        &frame[6U + MOTOR_UART_COMMAND_PAYLOAD_SIZE], crc);

    written = uart_write_bytes(
        MOTOR_UART_PORT, frame, sizeof(frame));
    if (written != (int)sizeof(frame)) {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.transmit_errors++;
        portEXIT_CRITICAL(&s_lock);
        return ESP_FAIL;
    }
    portENTER_CRITICAL(&s_lock);
    s_snapshot.transmitted_frames++;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

static void motor_uart_queue_control(
    MotorUart_Command_t command,
    int32_t value)
{
    const motor_uart_request_t request = {
        .command = command,
        .value = value,
    };

    if (xQueueSend(s_control_queue, &request, 0) != pdTRUE) {
        motor_uart_request_t discarded;
        (void)xQueueReceive(s_control_queue, &discarded, 0);
        (void)xQueueSend(s_control_queue, &request, 0);
    }
}

static void motor_uart_parse_telemetry(
    const uint8_t *payload,
    uint8_t length)
{
    if (length != MOTOR_UART_TELEMETRY_PAYLOAD_SIZE) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_snapshot.motor_running =
        (payload[0] & MOTOR_UART_STATUS_MOTOR_RUNNING) != 0U;
    s_snapshot.motor_fault =
        (payload[0] & MOTOR_UART_STATUS_MOTOR_FAULT) != 0U;
    s_snapshot.command_rejected =
        (payload[0] & MOTOR_UART_STATUS_COMMAND_REJECTED) != 0U;
    s_snapshot.link_active =
        (payload[0] & MOTOR_UART_STATUS_LINK_ACTIVE) != 0U;
    s_snapshot.mode =
        payload[1] == MOTOR_UART_MODE_POSITION
            ? MOTOR_UART_MODE_POSITION
            : MOTOR_UART_MODE_SPEED;
    s_snapshot.faults = MotorUart_ReadU16(&payload[2]);
    s_snapshot.measured_speed_rpm =
        MotorUart_ReadS16(&payload[4]);
    s_snapshot.reference_speed_rpm =
        MotorUart_ReadS16(&payload[6]);
    s_snapshot.current_position_cdeg =
        MotorUart_ReadU16(&payload[8]);
    s_snapshot.target_position_cdeg =
        MotorUart_ReadU16(&payload[10]);
    s_snapshot.position_error_cdeg =
        MotorUart_ReadS16(&payload[12]);
    s_snapshot.iq_ma = MotorUart_ReadS16(&payload[14]);
    s_snapshot.id_ma = MotorUart_ReadS16(&payload[16]);
    s_snapshot.iq_reference_ma =
        MotorUart_ReadS16(&payload[18]);
    s_snapshot.uq_mv = MotorUart_ReadS16(&payload[20]);
    s_snapshot.ud_mv = MotorUart_ReadS16(&payload[22]);
    s_snapshot.received_frames++;
    s_last_telemetry_ms = motor_uart_now_ms();
    portEXIT_CRITICAL(&s_lock);
}

static void motor_uart_parse_byte(uint8_t byte)
{
    uint8_t payload_length;
    uint16_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (s_parser.length == 0U) {
        if (byte == MOTOR_UART_SOF0) {
            s_parser.buffer[s_parser.length++] = byte;
        }
        return;
    }

    if (s_parser.length == 1U) {
        if (byte == MOTOR_UART_SOF1) {
            s_parser.buffer[s_parser.length++] = byte;
        } else {
            s_parser.length =
                byte == MOTOR_UART_SOF0 ? 1U : 0U;
        }
        return;
    }

    if (s_parser.length >= MOTOR_UART_MAX_FRAME_SIZE) {
        s_parser.length = 0U;
        return;
    }
    s_parser.buffer[s_parser.length++] = byte;

    if (s_parser.length < 6U) {
        return;
    }

    payload_length = s_parser.buffer[5];
    if (payload_length > MOTOR_UART_MAX_PAYLOAD) {
        s_parser.length = 0U;
        return;
    }

    expected_length = (uint16_t)(6U + payload_length + 2U);
    if (s_parser.length < expected_length) {
        return;
    }

    received_crc = MotorUart_ReadU16(
        &s_parser.buffer[6U + payload_length]);
    calculated_crc = motor_uart_crc16(
        &s_parser.buffer[2], (uint16_t)(4U + payload_length));

    if (received_crc != calculated_crc) {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.crc_errors++;
        portEXIT_CRITICAL(&s_lock);
    } else if (
        s_parser.buffer[2] == MOTOR_UART_PROTOCOL_VERSION &&
        s_parser.buffer[3] == MOTOR_UART_FRAME_TELEMETRY) {
        motor_uart_parse_telemetry(
            &s_parser.buffer[6], payload_length);
    } else {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.protocol_errors++;
        portEXIT_CRITICAL(&s_lock);
    }
    s_parser.length = 0U;
}

static void motor_uart_rx_task(void *argument)
{
    (void)argument;
    uint8_t received[96];

    while (true) {
        motor_uart_apply_reconnect();

        const int count = uart_read_bytes(
            MOTOR_UART_PORT,
            received,
            sizeof(received),
            pdMS_TO_TICKS(10));
        if (count > 0) {
            portENTER_CRITICAL(&s_lock);
            s_snapshot.received_bytes += (uint32_t)count;
            portEXIT_CRITICAL(&s_lock);
        }
        for (int i = 0; i < count; i++) {
            motor_uart_parse_byte(received[i]);
        }

        portENTER_CRITICAL(&s_lock);
        if (s_snapshot.link_active &&
            (motor_uart_now_ms() - s_last_telemetry_ms >
             MOTOR_UART_LINK_TIMEOUT_MS)) {
            s_snapshot.link_active = false;
        }
        portEXIT_CRITICAL(&s_lock);
    }
}

static void motor_uart_tx_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t next_ping_ms = motor_uart_now_ms();

    while (true) {
        motor_uart_request_t request;
        uint8_t budget = 4U;
        bool reconnecting;
        bool control_enabled;

        portENTER_CRITICAL(&s_lock);
        reconnecting = s_snapshot.reconnecting;
        control_enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);
        if (reconnecting) {
            vTaskDelayUntil(
                &last_wake,
                pdMS_TO_TICKS(MOTOR_UART_TX_TASK_PERIOD_MS));
            continue;
        }

        while (control_enabled && budget-- > 0U &&
               xQueueReceive(
                   s_control_queue, &request, 0) == pdTRUE) {
            (void)motor_uart_transmit(
                request.command, request.value);
        }

        bool send_position;
        bool send_speed;
        uint16_t position;
        int16_t speed;
        portENTER_CRITICAL(&s_lock);
        send_position = control_enabled && s_position_dirty;
        send_speed = control_enabled && s_speed_dirty;
        position = s_latest_position_cdeg;
        speed = s_latest_speed_rpm;
        s_position_dirty = false;
        s_speed_dirty = false;
        portEXIT_CRITICAL(&s_lock);

        if (send_position) {
            (void)motor_uart_transmit(
                MOTOR_UART_CMD_SET_POSITION_CDEG, position);
        }
        if (send_speed) {
            (void)motor_uart_transmit(
                MOTOR_UART_CMD_SET_SPEED_RPM, speed);
        }

        const uint32_t now = motor_uart_now_ms();
        bool force_ping;
        portENTER_CRITICAL(&s_lock);
        force_ping = s_force_ping;
        s_force_ping = false;
        portEXIT_CRITICAL(&s_lock);
        if (force_ping || (int32_t)(now - next_ping_ms) >= 0) {
            next_ping_ms = now + MOTOR_UART_PING_PERIOD_MS;
            (void)motor_uart_transmit(MOTOR_UART_CMD_PING, 0);
        }

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(MOTOR_UART_TX_TASK_PERIOD_MS));
    }
}

esp_err_t motor_uart_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    const uart_config_t config = {
        .baud_rate = MOTOR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_control_enabled = false;
    s_snapshot.mode = MOTOR_UART_MODE_SPEED;
    s_snapshot.baud_rate = MOTOR_UART_BAUD_RATE;
    s_control_queue = xQueueCreate(
        MOTOR_UART_CONTROL_QUEUE_SIZE,
        sizeof(motor_uart_request_t));
    if (s_control_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(
        uart_driver_install(
            MOTOR_UART_PORT,
            MOTOR_UART_RX_BUFFER_SIZE,
            MOTOR_UART_TX_BUFFER_SIZE,
            0,
            NULL,
            0),
        TAG,
        "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(
        uart_param_config(MOTOR_UART_PORT, &config),
        TAG,
        "uart_param_config failed");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(
            MOTOR_UART_PORT,
            MOTOR_UART_TX_GPIO,
            MOTOR_UART_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE),
        TAG,
        "uart_set_pin failed");
    ESP_RETURN_ON_ERROR(
        uart_flush_input(MOTOR_UART_PORT),
        TAG,
        "uart_flush_input failed");

    if (xTaskCreate(
            motor_uart_rx_task,
            "motor_uart_rx",
            3072,
            NULL,
            8,
            &s_rx_task) != pdPASS ||
        xTaskCreate(
            motor_uart_tx_task,
            "motor_uart_tx",
            3072,
            NULL,
            8,
            &s_tx_task) != pdPASS) {
        if (s_rx_task != NULL) {
            vTaskDelete(s_rx_task);
            s_rx_task = NULL;
        }
        if (s_tx_task != NULL) {
            vTaskDelete(s_tx_task);
            s_tx_task = NULL;
        }
        (void)uart_driver_delete(MOTOR_UART_PORT);
        vQueueDelete(s_control_queue);
        s_control_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;

    ESP_LOGI(
        TAG,
        "USART link ready: TX GPIO%d, RX GPIO%d, %lu baud",
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO,
        (unsigned long)MOTOR_UART_BAUD_RATE);
    return ESP_OK;
}

void motor_uart_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    if (s_rx_task != NULL) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_tx_task != NULL) {
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }
    (void)uart_driver_delete(MOTOR_UART_PORT);
    if (s_control_queue != NULL) {
        vQueueDelete(s_control_queue);
        s_control_queue = NULL;
    }
    gpio_reset_pin(MOTOR_UART_TX_GPIO);
    gpio_reset_pin(MOTOR_UART_RX_GPIO);
    s_initialized = false;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

bool motor_uart_is_initialized(void)
{
    return s_initialized;
}

esp_err_t motor_uart_request_reconnect(uint32_t baud_rate)
{
    if (baud_rate == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    s_requested_baud_rate = baud_rate;
    s_reconnect_requested = true;
    s_snapshot.link_active = false;
    s_snapshot.reconnecting = true;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void motor_uart_get_snapshot(motor_uart_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_lock);
}

void motor_uart_set_control_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_lock);
    s_control_enabled = enabled;
    if (!enabled) {
        s_speed_dirty = false;
        s_position_dirty = false;
    }
    portEXIT_CRITICAL(&s_lock);
    if (!enabled && s_control_queue != NULL) {
        xQueueReset(s_control_queue);
    }
}

void motor_uart_set_mode(MotorUart_Mode_t mode)
{
    motor_uart_queue_control(MOTOR_UART_CMD_SET_MODE, mode);
}

void motor_uart_set_speed_rpm(int16_t speed_rpm)
{
    portENTER_CRITICAL(&s_lock);
    s_latest_speed_rpm = speed_rpm;
    s_speed_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

void motor_uart_set_position_cdeg(uint16_t position_cdeg)
{
    portENTER_CRITICAL(&s_lock);
    s_latest_position_cdeg = position_cdeg;
    s_position_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

void motor_uart_start_motor(void)
{
    motor_uart_queue_control(MOTOR_UART_CMD_START, 0);
}

void motor_uart_stop_motor(void)
{
    motor_uart_queue_control(MOTOR_UART_CMD_STOP, 0);
}

void motor_uart_acknowledge_fault(void)
{
    motor_uart_queue_control(MOTOR_UART_CMD_ACK_FAULT, 0);
}

void motor_uart_zero_position(void)
{
    motor_uart_queue_control(MOTOR_UART_CMD_ZERO_POSITION, 0);
}
