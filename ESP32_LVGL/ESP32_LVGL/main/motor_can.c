#include "motor_can.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MOTOR_CAN_TX_GPIO                  GPIO_NUM_5
#define MOTOR_CAN_RX_GPIO                  GPIO_NUM_6
#define MOTOR_CAN_CONTROL_QUEUE_LENGTH     12U
#define MOTOR_CAN_RX_QUEUE_LENGTH          24U
#define MOTOR_CAN_TX_QUEUE_DEPTH           4U
#define MOTOR_CAN_TX_TIMEOUT_MS            5
#define MOTOR_CAN_TX_TASK_PERIOD_MS        5U
#define MOTOR_CAN_HEARTBEAT_PERIOD_MS      100U
#define MOTOR_CAN_LINK_TIMEOUT_MS          250U

typedef struct
{
    MotorCan_Command_t command;
    int32_t value;
} motor_can_request_t;

/*
 * twai_frame_t 只保存指向负载的指针，因此离开 ISR 前需要把接收帧复制到
 * 这个自包含对象中。
 */
typedef struct
{
    uint32_t identifier;
    uint8_t data_length;
    bool extended;
    bool remote;
    uint8_t data[MOTOR_CAN_FRAME_SIZE];
} motor_can_rx_frame_t;

static const char *TAG = "MOTOR_CAN";
static twai_node_handle_t s_twai_node;
static QueueHandle_t s_control_queue;
static QueueHandle_t s_rx_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static motor_can_snapshot_t s_snapshot;
static int16_t s_pending_speed_rpm;
static uint16_t s_pending_position_cdeg;
static bool s_speed_dirty;
static bool s_position_dirty;
static bool s_control_enabled;
static uint8_t s_tx_sequence;
static int64_t s_last_status_us;
static bool s_recovery_requested;
static bool s_transceiver_test_passed;
static uint32_t s_pending_error_flags;
static int64_t s_last_error_log_us;
static uint8_t s_tx_data[MOTOR_CAN_FRAME_SIZE];
static twai_frame_t s_tx_frame = {
    .header = {
        .id = MOTOR_CAN_ID_COMMAND,
        .dlc = MOTOR_CAN_FRAME_SIZE,
    },
    .buffer = s_tx_data,
    .buffer_len = sizeof(s_tx_data),
};
static bool s_tx_pending;
static TaskHandle_t s_rx_task;
static TaskHandle_t s_tx_task;
static bool s_initialized;

/**
 * @brief 对外部 CAN 收发器执行 GPIO 电平连通性自检。
 *
 * 在 TWAI 接管引脚前，分别驱动 TXD 为隐性和显性电平并观察 RXD。此结果仅
 * 用作诊断提示；本地测试失败不会阻止 CAN 启动，因为有效状态帧更能证明链路正常。
 * @return RXD 跟随预期本地电平变化时返回 true，否则返回 false。
 */
static bool motor_can_transceiver_self_test(void)
{
    const gpio_config_t rx_config = {
        .pin_bit_mask = 1ULL << MOTOR_CAN_RX_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const gpio_config_t tx_config = {
        .pin_bit_mask = 1ULL << MOTOR_CAN_TX_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&rx_config));
    ESP_ERROR_CHECK(gpio_config(&tx_config));

    gpio_set_level(MOTOR_CAN_TX_GPIO, 1);
    esp_rom_delay_us(10);
    const int recessive_level = gpio_get_level(MOTOR_CAN_RX_GPIO);

    gpio_set_level(MOTOR_CAN_TX_GPIO, 0);
    esp_rom_delay_us(10);
    const int dominant_level = gpio_get_level(MOTOR_CAN_RX_GPIO);

    gpio_set_level(MOTOR_CAN_TX_GPIO, 1);
    esp_rom_delay_us(10);
    const int released_level = gpio_get_level(MOTOR_CAN_RX_GPIO);
    const bool passed =
        (recessive_level == 1) &&
        (dominant_level == 0) &&
        (released_level == 1);

    ESP_LOGI(TAG, "Transceiver self-test RXD: idle=%d dominant=%d release=%d -> %s", recessive_level, dominant_level, released_level, passed ? "PASS" : "FAIL");
    if (!passed) {
        ESP_LOGW(TAG, "Local CAN path check failed; live TWAI diagnostics remain enabled. " "Verify Port1 GPIO5->TXD, GPIO6<-RXD, VCC/VIO/EN and S=LOW");
    }
    return passed;
}

/** @brief 在模块锁保护下递增共享发送错误计数。 */
static void motor_can_record_tx_error(void)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot.transmit_errors++;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 编码并发送一帧 CAN 命令帧。
 *
 * ESP-IDF 6 将帧负载指针放入队列，因此复用持久缓冲区之前，本函数会等待前一帧
 * 和当前帧发送完成。只有 CAN 持有控制权时才允许发送会影响电机的命令。
 *
 * @param command 协议命令操作码。
 * @param value 按照 @p command 规定格式编码的命令值。
 * @return 发送完成时返回 ESP_OK，否则返回 TWAI 错误码。
 */
static esp_err_t motor_can_transmit(MotorCan_Command_t command, int32_t value)
{
    if (command != MOTOR_CAN_CMD_NOP && command != MOTOR_CAN_CMD_PING) {
        bool enabled;
        portENTER_CRITICAL(&s_lock);
        enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);
        if (!enabled) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    /*
     * ESP-IDF 6 将帧指针入队，而不是复制完整对象。前一帧仍可能由驱动持有时，
     * 不得覆盖持久发送缓冲区。
     */
    if (s_tx_pending) {
        const esp_err_t pending_result =
            twai_node_transmit_wait_all_done(s_twai_node, MOTOR_CAN_TX_TIMEOUT_MS);
        if (pending_result != ESP_OK) {
            motor_can_record_tx_error();
            return pending_result;
        }
        s_tx_pending = false;
    }

    memset(s_tx_data, 0, sizeof(s_tx_data));
    s_tx_data[0] = MOTOR_CAN_PROTOCOL_VERSION;
    s_tx_data[1] = ++s_tx_sequence;
    s_tx_data[2] = (uint8_t)command;

    switch (command) {
    case MOTOR_CAN_CMD_SET_MODE:
        s_tx_data[3] = (uint8_t)value;
        break;
    case MOTOR_CAN_CMD_SET_SPEED_RPM:
        MotorCan_WriteS16(&s_tx_data[3], (int16_t)value);
        break;
    case MOTOR_CAN_CMD_SET_POSITION_CDEG:
        MotorCan_WriteS32(&s_tx_data[3], value);
        break;
    default:
        break;
    }

    esp_err_t result = twai_node_transmit(s_twai_node, &s_tx_frame, MOTOR_CAN_TX_TIMEOUT_MS);
    if (result == ESP_OK) {
        s_tx_pending = true;
        result = twai_node_transmit_wait_all_done(s_twai_node, MOTOR_CAN_TX_TIMEOUT_MS);
        if (result == ESP_OK) {
            s_tx_pending = false;
        }
    }
    if (result != ESP_OK) {
        motor_can_record_tx_error();
    } else {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.transmitted_frames++;
        portEXIT_CRITICAL(&s_lock);
    }
    return result;
}

/**
 * @brief 为 CAN TX 工作任务排队一个离散控制动作。
 *
 * 有界队列已满时丢弃最旧条目，使最新用户动作优先于过时的界面操作。
 */
static void motor_can_queue_control(MotorCan_Command_t command, int32_t value)
{
    if (s_control_queue == NULL) {
        return;
    }

    const motor_can_request_t request = {
        .command = command,
        .value = value,
    };

    if (xQueueSend(s_control_queue, &request, 0) != pdTRUE) {
        motor_can_request_t discarded;
        (void)xQueueReceive(s_control_queue, &discarded, 0);
        (void)xQueueSend(s_control_queue, &request, 0);
    }
}

/**
 * @brief 解码 CAN 状态帧 0x180 并刷新电机/链路状态。
 * @param frame 包含 8 字节负载的完整 CAN 接收帧。
 */
static void motor_can_parse_status(const motor_can_rx_frame_t *frame)
{
    if (frame->data[0] != MOTOR_CAN_PROTOCOL_VERSION) {
        return;
    }

    const uint8_t flags = frame->data[3];
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);
    s_snapshot.mode =
        (flags & MOTOR_CAN_STATUS_POSITION_MODE) != 0U
            ? MOTOR_CAN_MODE_POSITION
            : MOTOR_CAN_MODE_SPEED;
    s_snapshot.motor_running =
        (flags & MOTOR_CAN_STATUS_MOTOR_RUNNING) != 0U;
    s_snapshot.motor_fault =
        (flags & MOTOR_CAN_STATUS_MOTOR_FAULT) != 0U;
    s_snapshot.command_rejected =
        (flags & MOTOR_CAN_STATUS_COMMAND_REJECTED) != 0U;
    s_snapshot.measured_speed_rpm =
        MotorCan_ReadS16(&frame->data[4]);
    s_snapshot.faults = MotorCan_ReadU16(&frame->data[6]);
    s_snapshot.received_frames++;
    /* 有效总线帧比可选的 GPIO 自检结果更能证明收发器链路正常。 */
    s_snapshot.transceiver_fault = false;
    s_transceiver_test_passed = true;
    s_last_status_us = now_us;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将 CAN 参考值/位置帧 0x181 解码到遥测快照中。
 * @param frame 完整的 CAN 接收帧。
 */
static void motor_can_parse_references(const motor_can_rx_frame_t *frame)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot.reference_speed_rpm =
        MotorCan_ReadS16(&frame->data[0]);
    s_snapshot.current_position_cdeg =
        MotorCan_ReadU16(&frame->data[2]);
    s_snapshot.target_position_cdeg =
        MotorCan_ReadU16(&frame->data[4]);
    s_snapshot.position_error_cdeg =
        MotorCan_ReadS16(&frame->data[6]);
    s_snapshot.received_frames++;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将 CAN 电气电流帧 0x182 解码到遥测快照中。
 * @param frame 完整的 CAN 接收帧。
 */
static void motor_can_parse_electrical(const motor_can_rx_frame_t *frame)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot.iq_ma = MotorCan_ReadS16(&frame->data[0]);
    s_snapshot.id_ma = MotorCan_ReadS16(&frame->data[2]);
    s_snapshot.iq_reference_ma = MotorCan_ReadS16(&frame->data[4]);
    s_snapshot.id_reference_ma = MotorCan_ReadS16(&frame->data[6]);
    s_snapshot.received_frames++;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief ISR 上下文的 TWAI 接收回调：将帧复制到 RTOS 队列。
 *
 * 此处禁止执行帧解析、日志记录或界面操作；中断返回后由 RX 任务完成所有
 * 非简单处理。
 */
static bool IRAM_ATTR motor_can_rx_callback(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *event_data,
    void *user_context)
{
    (void)event_data;

    motor_can_rx_frame_t received = {0};
    twai_frame_t frame = {
        .buffer = received.data,
        .buffer_len = sizeof(received.data),
    };
    BaseType_t task_woken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        received.identifier = frame.header.id;
        received.data_length = (uint8_t)frame.header.dlc;
        received.extended = frame.header.ide;
        received.remote = frame.header.rtr;

        (void)xQueueSendFromISR((QueueHandle_t)user_context, &received, &task_woken);
    }

    return task_woken == pdTRUE;
}

/**
 * @brief ISR 上下文的 TWAI 错误回调：记录标志供后续任务处理。
 * @return 此回调不会唤醒更高优先级任务，因此始终返回 false。
 */
static bool IRAM_ATTR motor_can_error_callback(
    twai_node_handle_t handle,
    const twai_error_event_data_t *event_data,
    void *user_context)
{
    (void)handle;
    (void)user_context;

    portENTER_CRITICAL_ISR(&s_lock);
    s_pending_error_flags |= event_data->err_flags.val;
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

/**
 * @brief 校验队列中 CAN 帧并按协议 ID 分发的接收任务。
 * @param argument 未使用的任务参数。
 * @note 该任务运行在 ISR 上下文之外，因此可以进入模块临界区。
 */
static void motor_can_rx_task(void *argument)
{
    (void)argument;
    motor_can_rx_frame_t frame;

    for (;;) {
        if (xQueueReceive(s_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (frame.extended || frame.remote ||
            frame.data_length != MOTOR_CAN_FRAME_SIZE) {
            continue;
        }

        switch (frame.identifier) {
        case MOTOR_CAN_ID_STATUS:
            motor_can_parse_status(&frame);
            break;
        case MOTOR_CAN_ID_REFERENCES:
            motor_can_parse_references(&frame);
            break;
        case MOTOR_CAN_ID_ELECTRICAL:
            motor_can_parse_electrical(&frame);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief 更新 CAN 诊断信息，并启动/跟踪 Bus-Off 恢复流程。
 * @return 仅当控制器能够安全发送正常报文时返回 true。
 */
static bool motor_can_service_bus_state(void)
{
    twai_node_status_t status;
    if (twai_node_get_info(s_twai_node, &status, NULL) != ESP_OK) {
        return false;
    }

    uint32_t error_flags;
    bool transceiver_fault;
    portENTER_CRITICAL(&s_lock);
    error_flags = s_pending_error_flags;
    s_pending_error_flags = 0U;
    transceiver_fault = s_snapshot.transceiver_fault;
    portEXIT_CRITICAL(&s_lock);
    const int64_t now_us = esp_timer_get_time();
    if ((error_flags != 0U) &&
        ((now_us - s_last_error_log_us) >= 1000000LL)) {
        const twai_error_flags_t decoded = {.val = error_flags};
        s_last_error_log_us = now_us;
        ESP_LOGW(TAG, "CAN error flags=0x%02lx ACK=%u BIT=%u FORM=%u STUFF=%u ARB=%u " "RXD=%d LOCAL=%s", (unsigned long)error_flags, (unsigned)decoded.ack_err, (unsigned)decoded.bit_err, (unsigned)decoded.form_err, (unsigned)decoded.stuff_err, (unsigned)decoded.arb_lost, gpio_get_level(MOTOR_CAN_RX_GPIO), transceiver_fault ? "WARN" : "PASS");
    }

    const bool bus_off = status.state == TWAI_ERROR_BUS_OFF;
    portENTER_CRITICAL(&s_lock);
    s_snapshot.bus_off = bus_off;
    portEXIT_CRITICAL(&s_lock);

    if (bus_off) {
        if (!s_recovery_requested) {
            ESP_LOGW(TAG, "Bus-off (TEC=%u REC=%u RXD=%d LOCAL=%s); starting recovery", (unsigned)status.tx_error_count, (unsigned)status.rx_error_count, gpio_get_level(MOTOR_CAN_RX_GPIO), transceiver_fault ? "WARN" : "PASS");
            if (twai_node_recover(s_twai_node) == ESP_OK) {
                s_recovery_requested = true;
            }
        }
        return false;
    }

    if ((status.state == TWAI_ERROR_ACTIVE) && s_recovery_requested) {
        ESP_LOGI(TAG, "CAN bus recovered");
        s_recovery_requested = false;
    }
    return true;
}

/**
 * @brief 仅当没有更新目标覆盖时，重新标记发送失败的位置目标。
 * @param position_cdeg 发送失败的位置目标值，单位为 0.01°。
 */
static void motor_can_restore_position_if_latest(uint16_t position_cdeg)
{
    portENTER_CRITICAL(&s_lock);
    if (s_pending_position_cdeg == position_cdeg) {
        s_position_dirty = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 仅当没有更新目标覆盖时，重新标记发送失败的速度目标。
 * @param speed_rpm 发送失败的速度目标值，单位为 rpm。
 */
static void motor_can_restore_speed_if_latest(int16_t speed_rpm)
{
    portENTER_CRITICAL(&s_lock);
    if (s_pending_speed_rpm == speed_rpm) {
        s_speed_dirty = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 周期处理排队动作、最新目标和心跳的 CAN TX 工作任务。
 *
 * 总线关闭期间任务不会取出请求。短暂发送失败后，离散命令会重新放回队首；
 * 连续速度和位置目标仅在仍是最新请求值时才重试。
 * @param argument 未使用的任务参数。
 */
static void motor_can_tx_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t next_heartbeat_us = 0;

    for (;;) {
        /*
         * 控制器处于总线关闭状态时，不得取出命令或调用驱动的发送等待接口。
         * 待发送帧保持有效，并在恢复后由驱动重试。
         */
        if (!motor_can_service_bus_state()) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CAN_TX_TASK_PERIOD_MS));
            continue;
        }

        bool control_enabled;
        portENTER_CRITICAL(&s_lock);
        control_enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);

        motor_can_request_t request;
        while (control_enabled &&
               xQueueReceive(s_control_queue, &request, 0) == pdTRUE) {
            if (motor_can_transmit(request.command, request.value) != ESP_OK) {
                /*
                 * START、STOP、MODE 和 ACK 均为边沿触发的界面动作。短暂发送
                 * 超时时将命令保留在队首，避免静默丢失按键操作。
                 */
                (void)xQueueSendToFront(s_control_queue, &request, 0);
                break;
            }
        }

        bool send_speed;
        bool send_position;
        int16_t speed_rpm;
        uint16_t position_cdeg;

        portENTER_CRITICAL(&s_lock);
        send_speed = control_enabled && s_speed_dirty;
        speed_rpm = s_pending_speed_rpm;
        s_speed_dirty = false;
        send_position = control_enabled && s_position_dirty;
        position_cdeg = s_pending_position_cdeg;
        s_position_dirty = false;
        portEXIT_CRITICAL(&s_lock);

        if (send_position &&
            motor_can_transmit(MOTOR_CAN_CMD_SET_POSITION_CDEG, position_cdeg) != ESP_OK) {
            motor_can_restore_position_if_latest(position_cdeg);
        }
        if (send_speed &&
            motor_can_transmit(MOTOR_CAN_CMD_SET_SPEED_RPM, speed_rpm) != ESP_OK) {
            motor_can_restore_speed_if_latest(speed_rpm);
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_heartbeat_us) {
            (void)motor_can_transmit(MOTOR_CAN_CMD_PING, 0);
            next_heartbeat_us =
                now_us + (MOTOR_CAN_HEARTBEAT_PERIOD_MS * 1000LL);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CAN_TX_TASK_PERIOD_MS));
    }
}

/** @brief 删除已经创建的 CAN 专属 FreeRTOS 队列。 */
static void motor_can_delete_queues(void)
{
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    if (s_control_queue != NULL) {
        vQueueDelete(s_control_queue);
        s_control_queue = NULL;
    }
}

/**
 * @brief 初始化 TWAI、过滤器、回调以及绑定核心的 CAN 工作任务。
 *
 * 硬件过滤器允许 0x180～0x183 反馈帧通过。两个工作任务均绑定到 Core 0，
 * 从而与 Core 1 上的界面工作隔离。
 * @return 传输通道就绪时返回 ESP_OK，否则返回内存分配或 TWAI 错误码。
 */
esp_err_t motor_can_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_control_enabled = false;
    s_snapshot.mode = MOTOR_CAN_MODE_SPEED;
    s_pending_speed_rpm = 0;
    s_pending_position_cdeg = 0;
    s_speed_dirty = false;
    s_position_dirty = false;
    s_tx_sequence = 0;
    s_last_status_us = 0;
    s_recovery_requested = false;
    s_pending_error_flags = 0U;
    s_last_error_log_us = 0;
    s_tx_pending = false;

    s_transceiver_test_passed = motor_can_transceiver_self_test();
    s_snapshot.transceiver_fault = !s_transceiver_test_passed;

    s_control_queue =
        xQueueCreate(MOTOR_CAN_CONTROL_QUEUE_LENGTH, sizeof(motor_can_request_t));
    s_rx_queue =
        xQueueCreate(MOTOR_CAN_RX_QUEUE_LENGTH, sizeof(motor_can_rx_frame_t));
    if ((s_control_queue == NULL) || (s_rx_queue == NULL)) {
        motor_can_delete_queues();
        return ESP_ERR_NO_MEM;
    }

    const twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = MOTOR_CAN_TX_GPIO,
            .rx = MOTOR_CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = MOTOR_CAN_BITRATE,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = MOTOR_CAN_TX_QUEUE_DEPTH,
    };

    esp_err_t result =
        twai_new_node_onchip(&node_config, &s_twai_node);
    if (result != ESP_OK) {
        motor_can_delete_queues();
        return result;
    }

    /*
     * 匹配 0x180～0x183。应用实际使用 0x180～0x182，第四个值用于保证该掩码
     * 能由经典 CAN 控制器表示。
     */
    const twai_mask_filter_config_t filter_config = {
        .id = MOTOR_CAN_ID_STATUS,
        .mask = 0x7FCU,
        .is_ext = false,
        .no_classic = false,
        .no_fd = true,
    };
    result = twai_node_config_mask_filter(s_twai_node, 0, &filter_config);
    if (result != ESP_OK) {
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        motor_can_delete_queues();
        return result;
    }

    const twai_event_callbacks_t callbacks = {
        .on_rx_done = motor_can_rx_callback,
        .on_error = motor_can_error_callback,
    };
    result = twai_node_register_event_callbacks(s_twai_node, &callbacks, s_rx_queue);
    if (result != ESP_OK) {
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        motor_can_delete_queues();
        return result;
    }

    result = twai_node_enable(s_twai_node);
    if (result != ESP_OK) {
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        motor_can_delete_queues();
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    const int rxd_idle_level = gpio_get_level(MOTOR_CAN_RX_GPIO);
    ESP_LOGI(TAG, "Port1 RXD idle level=%d (expected 1); TXD is peripheral output", rxd_idle_level);
    if (rxd_idle_level == 0) {
        ESP_LOGE(TAG, "RXD is stuck low: check transceiver VCC/VIO, S pin, CAN short and wiring");
    }

    if (xTaskCreatePinnedToCore(motor_can_rx_task, "motor_can_rx", 3072, NULL, 12, &s_rx_task, 0) != pdPASS ||
        xTaskCreatePinnedToCore(motor_can_tx_task, "motor_can_tx", 3072, NULL, 11, &s_tx_task, 0) != pdPASS) {
        if (s_rx_task != NULL) {
            vTaskDelete(s_rx_task);
            s_rx_task = NULL;
        }
        if (s_tx_task != NULL) {
            vTaskDelete(s_tx_task);
            s_tx_task = NULL;
        }
        (void)twai_node_disable(s_twai_node);
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        motor_can_delete_queues();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "TWAI ready: TX=GPIO%d RX=GPIO%d bitrate=%u", MOTOR_CAN_TX_GPIO, MOTOR_CAN_RX_GPIO, MOTOR_CAN_BITRATE);
    s_initialized = true;
    return ESP_OK;
}

/**
 * @brief 在销毁队列和 TWAI 节点前停止 CAN 工作任务。
 * @note 此释放顺序可防止任务或回调访问已经释放的资源。
 */
void motor_can_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    /* 删除队列或 TWAI 节点之前先停止工作任务。 */
    if (s_rx_task != NULL) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_tx_task != NULL) {
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }
    if (s_twai_node != NULL) {
        (void)twai_node_disable(s_twai_node);
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
    }
    motor_can_delete_queues();
    gpio_reset_pin(MOTOR_CAN_TX_GPIO);
    gpio_reset_pin(MOTOR_CAN_RX_GPIO);
    s_initialized = false;
    s_transceiver_test_passed = false;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

/** @brief 返回 CAN 传输通道是否已经完成初始化。 */
bool motor_can_is_initialized(void)
{
    return s_initialized;
}

/**
 * @brief 复制最新 CAN 遥测，并根据时间戳计算链路在线状态。
 * @param[out] snapshot 接收状态的目标快照；传入 NULL 时忽略。
 */
void motor_can_get_snapshot(motor_can_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    *snapshot = s_snapshot;
    const int64_t last_status_us = s_last_status_us;
    portEXIT_CRITICAL(&s_lock);

    snapshot->link_active =
        (last_status_us > 0) &&
        ((esp_timer_get_time() - last_status_us) <=
         (MOTOR_CAN_LINK_TIMEOUT_MS * 1000LL));
}

/**
 * @brief 授予或撤销 CAN 命令控制权。
 * @param enabled 选择 CAN 作为电机控制通道时传入 true，否则传入 false。
 */
void motor_can_set_control_enabled(bool enabled)
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

/** @brief 排队一个 CAN 模式切换命令。 */
void motor_can_set_mode(MotorCan_Mode_t mode)
{
    motor_can_queue_control(MOTOR_CAN_CMD_SET_MODE, mode);
}

/** @brief 替换最新 CAN 速度目标，单位 rpm。 */
void motor_can_set_speed_rpm(int16_t speed_rpm)
{
    portENTER_CRITICAL(&s_lock);
    s_pending_speed_rpm = speed_rpm;
    s_speed_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将角度环绕到 0..35999 后替换最新 CAN 位置目标。
 * @param position_cdeg 请求角度，单位为 0.01°。
 */
void motor_can_set_position_cdeg(uint16_t position_cdeg)
{
    portENTER_CRITICAL(&s_lock);
    s_pending_position_cdeg = position_cdeg % 36000U;
    s_position_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

/** @brief 排队一个 CAN 电机启动命令。 */
void motor_can_start_motor(void)
{
    motor_can_queue_control(MOTOR_CAN_CMD_START, 0);
}

/** @brief 排队一个 CAN 电机停止命令。 */
void motor_can_stop_motor(void)
{
    motor_can_queue_control(MOTOR_CAN_CMD_STOP, 0);
}

/** @brief 排队一个 CAN 故障确认命令。 */
void motor_can_acknowledge_fault(void)
{
    motor_can_queue_control(MOTOR_CAN_CMD_ACK_FAULT, 0);
}

/** @brief 排队一个 CAN 位置清零命令。 */
void motor_can_zero_position(void)
{
    motor_can_queue_control(MOTOR_CAN_CMD_ZERO_POSITION, 0);
}
