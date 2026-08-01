/**
  ******************************************************************************
  * @file    foc_app_protocol.h
  * @brief   MCP user-command bridge for the Qt speed/position control panel.
  ******************************************************************************
  */
#ifndef FOC_APP_PROTOCOL_H
#define FOC_APP_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define FOC_APP_PROTOCOL_VERSION  (3U)
#define FOC_APP_MCP_CALLBACK_ID   (0U)
#define FOC_APP_TELEMETRY_MAGIC   (0x31434F46UL) /* "FOC1", little endian */
#define FOC_APP_UART_DIAG_MAGIC   (0x31443355UL) /* "U3D1", little endian */
#define FOC_APP_TELEMETRY_SIZE    (88U)

typedef enum
{
  FOC_APP_CMD_GET_TELEMETRY = 0,
  FOC_APP_CMD_SET_MODE = 1,      /* uint8: 0 speed, 1 position */
  FOC_APP_CMD_SET_SPEED_RPM = 2, /* int16 rpm + uint32 duration_ms */
  FOC_APP_CMD_SET_POSITION = 3, /* int32 cdeg + uint32 duration_ms */
  FOC_APP_CMD_START_MOTOR = 4,
  FOC_APP_CMD_STOP_MOTOR = 5,
  FOC_APP_CMD_ACK_FAULT = 6,
  FOC_APP_CMD_ZERO_POSITION = 7,
  FOC_APP_CMD_HOLD_POSITION = 8
} FocApp_Command_t;

typedef enum
{
  FOC_APP_MODE_SPEED = 0,
  FOC_APP_MODE_POSITION = 1
} FocApp_Mode_t;

void FocAppProtocol_Init(void);
void FocAppProtocol_Tick(void);
FocApp_Mode_t FocApp_GetControlMode(void);
bool FocApp_SetControlMode(FocApp_Mode_t mode);
bool FocApp_SetSpeedCommand(int16_t targetRpm, uint32_t durationMs);
bool FocApp_SetPositionCommand(int32_t targetCdeg, uint32_t durationMs);
bool FocApp_HoldPosition(void);

#ifdef __cplusplus
}
#endif
#endif /* FOC_APP_PROTOCOL_H */
