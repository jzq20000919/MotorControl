#ifndef MOTOR_LINK_H
#define MOTOR_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    MOTOR_LINK_NONE = 0,
    MOTOR_LINK_UART,
    MOTOR_LINK_CAN
} motor_link_transport_t;

typedef enum
{
    MOTOR_LINK_MODE_SPEED = 0,
    MOTOR_LINK_MODE_POSITION = 1
} motor_link_mode_t;

typedef struct
{
    motor_link_transport_t transport;
    bool uart_connected;
    bool uart_link_active;
    bool can_connected;
    bool can_link_active;
    bool link_active;
    bool reconnecting;
    bool bus_off;
    bool transceiver_fault;
    bool motor_running;
    bool motor_fault;
    bool command_rejected;
    motor_link_mode_t mode;
    uint16_t faults;
    int16_t measured_speed_rpm;
    int16_t reference_speed_rpm;
    uint16_t current_position_cdeg;
    uint16_t target_position_cdeg;
    int16_t position_error_cdeg;
    int16_t iq_ma;
    int16_t id_ma;
    int16_t iq_reference_ma;
    int16_t id_reference_ma;
    int16_t uq_mv;
    int16_t ud_mv;
    uint32_t received_frames;
    uint32_t transmitted_frames;
    uint32_t transmit_errors;
    uint32_t baud_rate;
} motor_link_snapshot_t;

void motor_link_init(void);
esp_err_t motor_link_connect_uart(uint32_t baud_rate);
esp_err_t motor_link_connect_can(void);
void motor_link_disconnect_uart(void);
void motor_link_disconnect_can(void);
void motor_link_get_snapshot(motor_link_snapshot_t *snapshot);
void motor_link_set_mode(motor_link_mode_t mode);
void motor_link_set_speed_rpm(int16_t speed_rpm);
void motor_link_set_position_cdeg(uint16_t position_cdeg);
void motor_link_start_motor(void);
void motor_link_stop_motor(void);
void motor_link_acknowledge_fault(void);
void motor_link_zero_position(void);

#endif /* MOTOR_LINK_H */
