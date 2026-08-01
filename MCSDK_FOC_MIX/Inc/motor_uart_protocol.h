#ifndef MOTOR_UART_PROTOCOL_H
#define MOTOR_UART_PROTOCOL_H

#include <stdint.h>

#define MOTOR_UART_PROTOCOL_VERSION       (1U)
#define MOTOR_UART_BAUD_RATE              (115200U)
#define MOTOR_UART_SOF0                   (0xA5U)
#define MOTOR_UART_SOF1                   (0x5AU)
#define MOTOR_UART_MAX_PAYLOAD            (32U)
#define MOTOR_UART_MAX_FRAME_SIZE         (40U)
#define MOTOR_UART_COMMAND_PAYLOAD_SIZE   (5U)
#define MOTOR_UART_TELEMETRY_PAYLOAD_SIZE (24U)

typedef enum { MOTOR_UART_FRAME_COMMAND = 1, MOTOR_UART_FRAME_TELEMETRY = 2 } MotorUart_FrameType_t;
typedef enum {
  MOTOR_UART_CMD_NOP = 0, MOTOR_UART_CMD_SET_MODE, MOTOR_UART_CMD_SET_SPEED_RPM,
  MOTOR_UART_CMD_SET_POSITION_CDEG, MOTOR_UART_CMD_START, MOTOR_UART_CMD_STOP,
  MOTOR_UART_CMD_ACK_FAULT, MOTOR_UART_CMD_ZERO_POSITION, MOTOR_UART_CMD_PING
} MotorUart_Command_t;
typedef enum { MOTOR_UART_MODE_SPEED = 0, MOTOR_UART_MODE_POSITION = 1 } MotorUart_Mode_t;

#define MOTOR_UART_STATUS_MOTOR_RUNNING    (1U << 0)
#define MOTOR_UART_STATUS_MOTOR_FAULT      (1U << 1)
#define MOTOR_UART_STATUS_COMMAND_REJECTED (1U << 2)
#define MOTOR_UART_STATUS_LINK_ACTIVE      (1U << 3)

static inline uint16_t MotorUart_ReadU16(const uint8_t *data)
{ return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U)); }
static inline int16_t MotorUart_ReadS16(const uint8_t *data)
{ return (int16_t)MotorUart_ReadU16(data); }
static inline int32_t MotorUart_ReadS32(const uint8_t *data)
{ return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                   ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U)); }
static inline void MotorUart_WriteU16(uint8_t *data, uint16_t value)
{ data[0] = (uint8_t)value; data[1] = (uint8_t)(value >> 8U); }
static inline void MotorUart_WriteS16(uint8_t *data, int16_t value)
{ MotorUart_WriteU16(data, (uint16_t)value); }

#endif /* MOTOR_UART_PROTOCOL_H */
