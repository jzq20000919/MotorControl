#ifndef MOTOR_UART_H
#define MOTOR_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_uart_protocol.h"

typedef struct
{
    bool link_active;
    bool reconnecting;
    bool motor_running;
    bool motor_fault;
    bool command_rejected;
    MotorUart_Mode_t mode;
    uint16_t faults;
    int16_t measured_speed_rpm;
    int16_t reference_speed_rpm;
    uint16_t current_position_cdeg;
    uint16_t target_position_cdeg;
    int16_t position_error_cdeg;
    int16_t iq_ma;
    int16_t id_ma;
    int16_t iq_reference_ma;
    int16_t uq_mv;
    int16_t ud_mv;
    uint32_t received_bytes;
    uint32_t received_frames;
    uint32_t transmitted_frames;
    uint32_t transmit_errors;
    uint32_t crc_errors;
    uint32_t protocol_errors;
    uint32_t baud_rate;
    uint32_t reconnect_count;
    uint32_t reconnect_errors;
} motor_uart_snapshot_t;

/** @brief 安装 UART1、创建 RX/TX 任务并初始化协议状态。 */
esp_err_t motor_uart_init(void);
/** @brief 停止 UART 任务、删除队列并释放 UART 驱动。 */
void motor_uart_deinit(void);
/** @brief 返回 UART 传输通道是否已完整初始化。 */
bool motor_uart_is_initialized(void);
/** @brief 请求以 @p baud_rate 异步重新配置 UART。 */
esp_err_t motor_uart_request_reconnect(uint32_t baud_rate);
/** @brief 不等待新帧，直接复制最新已解析 UART 遥测。 */
void motor_uart_get_snapshot(motor_uart_snapshot_t *snapshot);
/** @brief 授予或撤销该通道发送电机命令的控制权。 */
void motor_uart_set_control_enabled(bool enabled);

/** @brief 排队一个 UART 控制模式命令。 */
void motor_uart_set_mode(MotorUart_Mode_t mode);
/** @brief 保存最新速度目标，供 UART 周期发送，单位 rpm。 */
void motor_uart_set_speed_rpm(int16_t speed_rpm);
/** @brief 保存最新位置目标，供 UART 周期发送，单位 0.01°。 */
void motor_uart_set_position_cdeg(uint16_t position_cdeg);
/** @brief 排队一个 UART 电机启动命令。 */
void motor_uart_start_motor(void);
/** @brief 排队一个 UART 电机停止命令。 */
void motor_uart_stop_motor(void);
/** @brief 排队一个 UART 故障确认命令。 */
void motor_uart_acknowledge_fault(void);
/** @brief 排队一个 UART 位置清零命令。 */
void motor_uart_zero_position(void);

#endif /* MOTOR_UART_H */
