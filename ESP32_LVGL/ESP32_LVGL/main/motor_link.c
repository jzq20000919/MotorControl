#include "motor_link.h"

#include <string.h>

#include "motor_can.h"
#include "motor_uart.h"

static motor_link_transport_t s_transport;

void motor_link_init(void)
{
    s_transport = MOTOR_LINK_NONE;
}

esp_err_t motor_link_connect_uart(uint32_t baud_rate)
{
    const motor_link_transport_t previous = s_transport;
    if (!motor_uart_is_initialized()) {
        const esp_err_t result = motor_uart_init();
        if (result != ESP_OK) {
            return result;
        }
    }
    const esp_err_t result = motor_uart_request_reconnect(baud_rate);
    s_transport = result == ESP_OK ? MOTOR_LINK_UART : previous;
    if (result == ESP_OK) {
        motor_uart_set_control_enabled(true);
        motor_can_set_control_enabled(false);
    }
    return result;
}

esp_err_t motor_link_connect_can(void)
{
    const motor_link_transport_t previous = s_transport;
    if (!motor_can_is_initialized()) {
        const esp_err_t result = motor_can_init();
        if (result != ESP_OK) {
            s_transport = previous;
            return result;
        }
    }
    s_transport = MOTOR_LINK_CAN;
    motor_can_set_control_enabled(true);
    motor_uart_set_control_enabled(false);
    return ESP_OK;
}

void motor_link_disconnect_uart(void)
{
    motor_uart_set_control_enabled(false);
    if (motor_uart_is_initialized()) {
        motor_uart_deinit();
    }
    if (s_transport == MOTOR_LINK_UART) {
        s_transport = MOTOR_LINK_NONE;
    }
}

void motor_link_disconnect_can(void)
{
    motor_can_set_control_enabled(false);
    if (motor_can_is_initialized()) {
        motor_can_deinit();
    }
    if (s_transport == MOTOR_LINK_CAN) {
        s_transport = MOTOR_LINK_NONE;
    }
}

void motor_link_get_snapshot(motor_link_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->transport = s_transport;
    snapshot->uart_connected = motor_uart_is_initialized();
    snapshot->can_connected = motor_can_is_initialized();

    motor_uart_snapshot_t uart;
    memset(&uart, 0, sizeof(uart));
    if (snapshot->uart_connected) {
        motor_uart_get_snapshot(&uart);
        snapshot->uart_link_active = uart.link_active;
    }

    motor_can_snapshot_t can;
    memset(&can, 0, sizeof(can));
    if (snapshot->can_connected) {
        motor_can_get_snapshot(&can);
        snapshot->can_link_active = can.link_active;
    }

    if (s_transport == MOTOR_LINK_UART && snapshot->uart_connected) {
        snapshot->link_active = uart.link_active;
        snapshot->reconnecting = uart.reconnecting;
        snapshot->motor_running = uart.motor_running;
        snapshot->motor_fault = uart.motor_fault;
        snapshot->command_rejected = uart.command_rejected;
        snapshot->mode = (motor_link_mode_t)uart.mode;
        snapshot->faults = uart.faults;
        snapshot->measured_speed_rpm = uart.measured_speed_rpm;
        snapshot->reference_speed_rpm = uart.reference_speed_rpm;
        snapshot->current_position_cdeg = uart.current_position_cdeg;
        snapshot->target_position_cdeg = uart.target_position_cdeg;
        snapshot->position_error_cdeg = uart.position_error_cdeg;
        snapshot->iq_ma = uart.iq_ma;
        snapshot->id_ma = uart.id_ma;
        snapshot->iq_reference_ma = uart.iq_reference_ma;
        /* The UART v1 telemetry has no Id reference field. */
        snapshot->id_reference_ma = 0;
        snapshot->received_frames = uart.received_frames;
        snapshot->transmitted_frames = uart.transmitted_frames;
        snapshot->transmit_errors = uart.transmit_errors;
        snapshot->baud_rate = uart.baud_rate;
    } else if (s_transport == MOTOR_LINK_CAN && snapshot->can_connected) {
        snapshot->link_active = can.link_active;
        snapshot->bus_off = can.bus_off;
        snapshot->transceiver_fault = can.transceiver_fault;
        snapshot->motor_running = can.motor_running;
        snapshot->motor_fault = can.motor_fault;
        snapshot->command_rejected = can.command_rejected;
        snapshot->mode = (motor_link_mode_t)can.mode;
        snapshot->faults = can.faults;
        snapshot->measured_speed_rpm = can.measured_speed_rpm;
        snapshot->reference_speed_rpm = can.reference_speed_rpm;
        snapshot->current_position_cdeg = can.current_position_cdeg;
        snapshot->target_position_cdeg = can.target_position_cdeg;
        snapshot->position_error_cdeg = can.position_error_cdeg;
        snapshot->iq_ma = can.iq_ma;
        snapshot->id_ma = can.id_ma;
        snapshot->iq_reference_ma = can.iq_reference_ma;
        snapshot->id_reference_ma = can.id_reference_ma;
        snapshot->received_frames = can.received_frames;
        snapshot->transmitted_frames = can.transmitted_frames;
        snapshot->transmit_errors = can.transmit_errors;
    }
}

void motor_link_set_mode(motor_link_mode_t mode)
{
    if (s_transport == MOTOR_LINK_UART) motor_uart_set_mode((MotorUart_Mode_t)mode);
    if (s_transport == MOTOR_LINK_CAN) motor_can_set_mode((MotorCan_Mode_t)mode);
}
void motor_link_set_speed_rpm(int16_t speed_rpm)
{
    if (s_transport == MOTOR_LINK_UART) motor_uart_set_speed_rpm(speed_rpm);
    if (s_transport == MOTOR_LINK_CAN) motor_can_set_speed_rpm(speed_rpm);
}
void motor_link_set_position_cdeg(uint16_t position_cdeg)
{
    if (s_transport == MOTOR_LINK_UART) motor_uart_set_position_cdeg(position_cdeg);
    if (s_transport == MOTOR_LINK_CAN) motor_can_set_position_cdeg(position_cdeg);
}
void motor_link_start_motor(void)
{
    if (s_transport == MOTOR_LINK_UART) motor_uart_start_motor();
    if (s_transport == MOTOR_LINK_CAN) motor_can_start_motor();
}
void motor_link_stop_motor(void)
{
    if (s_transport == MOTOR_LINK_UART) motor_uart_stop_motor();
    if (s_transport == MOTOR_LINK_CAN) motor_can_stop_motor();
}
void motor_link_acknowledge_fault(void)
{
    if (s_transport == MOTOR_LINK_UART) motor_uart_acknowledge_fault();
    if (s_transport == MOTOR_LINK_CAN) motor_can_acknowledge_fault();
}
