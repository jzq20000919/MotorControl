#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_can_protocol.h"

typedef struct
{
    bool link_active;
    bool bus_off;
    bool transceiver_fault;
    bool motor_running;
    bool motor_fault;
    bool command_rejected;
    MotorCan_Mode_t mode;
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
    uint32_t received_frames;
    uint32_t transmitted_frames;
    uint32_t transmit_errors;
} motor_can_snapshot_t;

esp_err_t motor_can_init(void);
void motor_can_deinit(void);
bool motor_can_is_initialized(void);
void motor_can_get_snapshot(motor_can_snapshot_t *snapshot);
void motor_can_set_control_enabled(bool enabled);

void motor_can_set_mode(MotorCan_Mode_t mode);
void motor_can_set_speed_rpm(int16_t speed_rpm);
void motor_can_set_position_cdeg(uint16_t position_cdeg);
void motor_can_start_motor(void);
void motor_can_stop_motor(void);
void motor_can_acknowledge_fault(void);
void motor_can_zero_position(void);

#endif /* MOTOR_CAN_H */
