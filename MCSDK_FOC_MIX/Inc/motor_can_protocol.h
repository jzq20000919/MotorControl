/*
 * Shared classic-CAN protocol between the ESP32 HMI and STM32 motor board.
 * Keep this file byte-for-byte compatible with ESP32_LVGL/main.
 */

#ifndef MOTOR_CAN_PROTOCOL_H
#define MOTOR_CAN_PROTOCOL_H

#include <stdint.h>

#define MOTOR_CAN_PROTOCOL_VERSION          (1U)
#define MOTOR_CAN_BITRATE                   (500000U)

#define MOTOR_CAN_ID_COMMAND                (0x100U)
#define MOTOR_CAN_ID_STATUS                 (0x180U)
#define MOTOR_CAN_ID_REFERENCES             (0x181U)
#define MOTOR_CAN_ID_ELECTRICAL             (0x182U)

#define MOTOR_CAN_FRAME_SIZE                (8U)

typedef enum
{
  MOTOR_CAN_CMD_NOP = 0,
  MOTOR_CAN_CMD_SET_MODE = 1,
  MOTOR_CAN_CMD_SET_SPEED_RPM = 2,
  MOTOR_CAN_CMD_SET_POSITION_CDEG = 3,
  MOTOR_CAN_CMD_START = 4,
  MOTOR_CAN_CMD_STOP = 5,
  MOTOR_CAN_CMD_ACK_FAULT = 6,
  MOTOR_CAN_CMD_ZERO_POSITION = 7,
  MOTOR_CAN_CMD_PING = 8
} MotorCan_Command_t;

typedef enum
{
  MOTOR_CAN_MODE_SPEED = 0,
  MOTOR_CAN_MODE_POSITION = 1
} MotorCan_Mode_t;

#define MOTOR_CAN_STATUS_POSITION_MODE      (1U << 0)
#define MOTOR_CAN_STATUS_MOTOR_RUNNING      (1U << 1)
#define MOTOR_CAN_STATUS_MOTOR_FAULT        (1U << 2)
#define MOTOR_CAN_STATUS_LINK_ACTIVE        (1U << 3)
#define MOTOR_CAN_STATUS_COMMAND_REJECTED   (1U << 4)

static inline uint16_t MotorCan_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0]) |
                    ((uint16_t)data[1] << 8U));
}

static inline int16_t MotorCan_ReadS16(const uint8_t *data)
{
  return (int16_t)MotorCan_ReadU16(data);
}

static inline uint32_t MotorCan_ReadU32(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static inline int32_t MotorCan_ReadS32(const uint8_t *data)
{
  return (int32_t)MotorCan_ReadU32(data);
}

static inline void MotorCan_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static inline void MotorCan_WriteS16(uint8_t *data, int16_t value)
{
  MotorCan_WriteU16(data, (uint16_t)value);
}

static inline void MotorCan_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static inline void MotorCan_WriteS32(uint8_t *data, int32_t value)
{
  MotorCan_WriteU32(data, (uint32_t)value);
}

#endif /* MOTOR_CAN_PROTOCOL_H */
