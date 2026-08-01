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

esp_err_t motor_uart_init(void);
void motor_uart_deinit(void);
bool motor_uart_is_initialized(void);
esp_err_t motor_uart_request_reconnect(uint32_t baud_rate);
void motor_uart_get_snapshot(motor_uart_snapshot_t *snapshot);
void motor_uart_set_control_enabled(bool enabled);

void motor_uart_set_mode(MotorUart_Mode_t mode);
void motor_uart_set_speed_rpm(int16_t speed_rpm);
void motor_uart_set_position_cdeg(uint16_t position_cdeg);
void motor_uart_start_motor(void);
void motor_uart_stop_motor(void);
void motor_uart_acknowledge_fault(void);
void motor_uart_zero_position(void);

#endif /* MOTOR_UART_H */
