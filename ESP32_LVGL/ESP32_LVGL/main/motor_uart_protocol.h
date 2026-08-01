/*
 * Shared USART3 protocol between the ESP32 HMI and STM32 motor board.
 * Keep this file byte-for-byte compatible with MCSDK_FOC_2804/Inc.
 */

#ifndef MOTOR_UART_PROTOCOL_H
#define MOTOR_UART_PROTOCOL_H

#include <stdint.h>

#define MOTOR_UART_PROTOCOL_VERSION       (1U)
#define MOTOR_UART_BAUD_RATE              (115200U)
#define MOTOR_UART_SOF0                   (0xA5U)
#define MOTOR_UART_SOF1                   (0x5AU)
#define MOTOR_UART_MAX_PAYLOAD            (32U)
#define MOTOR_UART_FRAME_OVERHEAD         (8U)
#define MOTOR_UART_MAX_FRAME_SIZE         \
  (MOTOR_UART_MAX_PAYLOAD + MOTOR_UART_FRAME_OVERHEAD)

typedef enum
{
  MOTOR_UART_FRAME_COMMAND = 1,
  MOTOR_UART_FRAME_TELEMETRY = 2
} MotorUart_FrameType_t;

typedef enum
{
  MOTOR_UART_CMD_NOP = 0,
  MOTOR_UART_CMD_SET_MODE = 1,
  MOTOR_UART_CMD_SET_SPEED_RPM = 2,
  MOTOR_UART_CMD_SET_POSITION_CDEG = 3,
  MOTOR_UART_CMD_START = 4,
  MOTOR_UART_CMD_STOP = 5,
  MOTOR_UART_CMD_ACK_FAULT = 6,
  MOTOR_UART_CMD_ZERO_POSITION = 7,
  MOTOR_UART_CMD_PING = 8
} MotorUart_Command_t;

typedef enum
{
  MOTOR_UART_MODE_SPEED = 0,
  MOTOR_UART_MODE_POSITION = 1
} MotorUart_Mode_t;

#define MOTOR_UART_STATUS_MOTOR_RUNNING       (1U << 0)
#define MOTOR_UART_STATUS_MOTOR_FAULT         (1U << 1)
#define MOTOR_UART_STATUS_COMMAND_REJECTED    (1U << 2)
#define MOTOR_UART_STATUS_LINK_ACTIVE         (1U << 3)

#define MOTOR_UART_COMMAND_PAYLOAD_SIZE        (5U)
#define MOTOR_UART_TELEMETRY_PAYLOAD_SIZE      (24U)

static inline uint16_t MotorUart_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0]) |
                    ((uint16_t)data[1] << 8U));
}

static inline int16_t MotorUart_ReadS16(const uint8_t *data)
{
  return (int16_t)MotorUart_ReadU16(data);
}

static inline uint32_t MotorUart_ReadU32(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static inline int32_t MotorUart_ReadS32(const uint8_t *data)
{
  return (int32_t)MotorUart_ReadU32(data);
}

static inline void MotorUart_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static inline void MotorUart_WriteS16(uint8_t *data, int16_t value)
{
  MotorUart_WriteU16(data, (uint16_t)value);
}

static inline void MotorUart_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static inline void MotorUart_WriteS32(uint8_t *data, int32_t value)
{
  MotorUart_WriteU32(data, (uint32_t)value);
}

#endif /* MOTOR_UART_PROTOCOL_H */
