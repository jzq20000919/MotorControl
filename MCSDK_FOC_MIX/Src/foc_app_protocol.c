/**
  ******************************************************************************
  * @file    foc_app_protocol.c
  * @brief   Qt speed/position panel command and telemetry bridge over MCP/ASPEP.
  ******************************************************************************
  */
#include "foc_app_protocol.h"

#include <stdbool.h>
#include <string.h>

#include "mc_api.h"
#include "mc_config.h"
#include "mcp.h"
#include "motor_uart.h"

#define FOC_APP_RAD_TO_CDEG      (5729.577951308232F)
#define FOC_APP_CDEG_TO_RAD      (0.000174532925199433F)
#define FOC_APP_MIN_DURATION_MS  (100UL)
#define FOC_APP_MAX_DURATION_MS  (120000UL)
#define FOC_APP_MAX_SPEED_DURATION_MS (60000UL)
#define FOC_APP_MAX_SPEED_RPM    (2600)

static MCI_State_t FocApp_PreviousMotorState = IDLE;
static FocApp_Mode_t FocApp_ControlMode = FOC_APP_MODE_POSITION;

static void FocApp_HoldCurrentPosition(void)
{
  float currentPosition = MC_GetCurrentPosition1();

  /*
   * The incremental encoder is re-zeroed when startup alignment completes.
   * Discard any trajectory created against the pre-alignment coordinate
   * system and enter RUN with a zero-error, zero-history torque command.
   */
  PID_HandleInit(&PID_PosParamsM1);
  PosCtrlM1.MovementDuration = 0.0F;
  PosCtrlM1.AngleStep = 0.0F;
  PosCtrlM1.Jerk = 0.0F;
  PosCtrlM1.CruiseSpeed = 0.0F;
  PosCtrlM1.Acceleration = 0.0F;
  PosCtrlM1.Omega = 0.0F;
  PosCtrlM1.OmegaPrev = 0.0F;
  PosCtrlM1.Theta = currentPosition;
  PosCtrlM1.ThetaPrev = currentPosition;
  PosCtrlM1.ReceivedTh = 0U;
  PosCtrlM1.TcTick = 0U;
  PosCtrlM1.ElapseTime = 0.0F;
  PosCtrlM1.PositionControlRegulation = true;
  PosCtrlM1.PositionCtrlStatus = TC_READY_FOR_COMMAND;
}

static void FocApp_EnterSpeedMode(void)
{
  PosCtrlM1.PositionControlRegulation = false;
  PosCtrlM1.PositionCtrlStatus = TC_READY_FOR_COMMAND;
  PosCtrlM1.MovementDuration = 0.0F;
  PID_HandleInit(&PID_PosParamsM1);

  /* A zero-speed ramp changes STC back to MCM_SPEED_MODE without a torque step. */
  if (MC_GetSTMStateMotor1() == RUN)
  {
    MC_ProgramSpeedRampMotor1_F(0.0F, 0U);
  }
}

bool FocApp_SetControlMode(FocApp_Mode_t mode)
{
  if ((mode != FOC_APP_MODE_SPEED) && (mode != FOC_APP_MODE_POSITION))
  {
    return false;
  }
  if (mode == FocApp_ControlMode)
  {
    return true;
  }

  FocApp_ControlMode = mode;
  if (mode == FOC_APP_MODE_POSITION)
  {
    if (MC_GetSTMStateMotor1() == RUN)
    {
      /* Lock to the measured angle before enabling the direct position PID. */
      FocApp_HoldCurrentPosition();
    }
  }
  else
  {
    FocApp_EnterSpeedMode();
  }
  return true;
}

FocApp_Mode_t FocApp_GetControlMode(void)
{
  return FocApp_ControlMode;
}

bool FocApp_HoldPosition(void)
{
  if ((MC_GetSTMStateMotor1() != RUN) ||
      (FocApp_ControlMode != FOC_APP_MODE_POSITION))
  {
    return false;
  }
  FocApp_HoldCurrentPosition();
  return true;
}

static void FocApp_WriteU16(uint8_t *buffer, uint16_t value)
{
  buffer[0] = (uint8_t)(value & 0xFFU);
  buffer[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void FocApp_WriteS16(uint8_t *buffer, int16_t value)
{
  FocApp_WriteU16(buffer, (uint16_t)value);
}

static void FocApp_WriteU32(uint8_t *buffer, uint32_t value)
{
  buffer[0] = (uint8_t)(value & 0xFFU);
  buffer[1] = (uint8_t)((value >> 8U) & 0xFFU);
  buffer[2] = (uint8_t)((value >> 16U) & 0xFFU);
  buffer[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void FocApp_WriteS32(uint8_t *buffer, int32_t value)
{
  FocApp_WriteU32(buffer, (uint32_t)value);
}

static int32_t FocApp_ReadS32(const uint8_t *buffer)
{
  uint32_t value = ((uint32_t)buffer[0]) |
                   ((uint32_t)buffer[1] << 8U) |
                   ((uint32_t)buffer[2] << 16U) |
                   ((uint32_t)buffer[3] << 24U);
  return (int32_t)value;
}

static int16_t FocApp_ReadS16(const uint8_t *buffer)
{
  uint16_t value = ((uint16_t)buffer[0]) |
                   ((uint16_t)buffer[1] << 8U);
  return (int16_t)value;
}

static uint32_t FocApp_ReadU32(const uint8_t *buffer)
{
  return ((uint32_t)buffer[0]) |
         ((uint32_t)buffer[1] << 8U) |
         ((uint32_t)buffer[2] << 16U) |
         ((uint32_t)buffer[3] << 24U);
}

bool FocApp_SetSpeedCommand(int16_t targetRpm, uint32_t durationMs)
{
  if ((MC_GetSTMStateMotor1() != RUN) ||
      (FocApp_ControlMode != FOC_APP_MODE_SPEED))
  {
    return false;
  }

  if (targetRpm > FOC_APP_MAX_SPEED_RPM)
  {
    targetRpm = FOC_APP_MAX_SPEED_RPM;
  }
  else if (targetRpm < -FOC_APP_MAX_SPEED_RPM)
  {
    targetRpm = -FOC_APP_MAX_SPEED_RPM;
  }
  if (durationMs > FOC_APP_MAX_SPEED_DURATION_MS)
  {
    durationMs = FOC_APP_MAX_SPEED_DURATION_MS;
  }
  MC_ProgramSpeedRampMotor1_F((float)targetRpm, (uint16_t)durationMs);
  return true;
}

bool FocApp_SetPositionCommand(int32_t targetCdeg, uint32_t durationMs)
{
  if ((MC_GetSTMStateMotor1() != RUN) ||
      (FocApp_ControlMode != FOC_APP_MODE_POSITION))
  {
    return false;
  }

  if (durationMs < FOC_APP_MIN_DURATION_MS)
  {
    durationMs = FOC_APP_MIN_DURATION_MS;
  }
  if (durationMs > FOC_APP_MAX_DURATION_MS)
  {
    durationMs = FOC_APP_MAX_DURATION_MS;
  }

  /* Start the replacement trajectory from the measured position. */
  PosCtrlM1.PositionCtrlStatus = TC_READY_FOR_COMMAND;
  PID_PosParamsM1.wPrevProcessVarError = 0;
  MC_ProgramPositionCommandMotor1(((float)targetCdeg) * FOC_APP_CDEG_TO_RAD,
                                  ((float)durationMs) / 1000.0F);
  return true;
}

static int32_t FocApp_FloatToS32(float value)
{
  if (value > 2147483000.0F)
  {
    return INT32_MAX;
  }
  if (value < -2147483000.0F)
  {
    return INT32_MIN;
  }
  return (int32_t)value;
}

static uint16_t FocApp_BuildTelemetry(uint8_t *buffer, int16_t capacity)
{
  qd_f_t current;
  qd_f_t currentReference;
  MotorUart_Diagnostics_t uartDiagnostics;

  if ((buffer == NULL) || (capacity < (int16_t)FOC_APP_TELEMETRY_SIZE))
  {
    return 0U;
  }

  current = MC_GetIqdMotor1_F();
  currentReference = MC_GetIqdrefMotor1_F();
  (void)memset(buffer, 0, FOC_APP_TELEMETRY_SIZE);
  FocApp_WriteU32(&buffer[0], FOC_APP_TELEMETRY_MAGIC);
  buffer[4] = FOC_APP_PROTOCOL_VERSION;
  buffer[5] = (uint8_t)FocApp_ControlMode;
  buffer[6] = (uint8_t)MC_GetSTMStateMotor1();
  buffer[7] = (uint8_t)MC_GetControlPositionStatusMotor1();
  FocApp_WriteU16(&buffer[8], MC_GetCurrentFaultsMotor1());
  FocApp_WriteU16(&buffer[10], MC_GetOccurredFaultsMotor1());
  FocApp_WriteS32(&buffer[12], FocApp_FloatToS32(current.q * 1000.0F));
  FocApp_WriteS32(&buffer[16], FocApp_FloatToS32(current.d * 1000.0F));
  FocApp_WriteS32(&buffer[20], FocApp_FloatToS32(currentReference.q * 1000.0F));
  FocApp_WriteS32(&buffer[24], FocApp_FloatToS32(currentReference.d * 1000.0F));
  FocApp_WriteS16(&buffer[36], (int16_t)MC_GetMecSpeedReferenceMotor1_F());
  FocApp_WriteS16(&buffer[38], (int16_t)MC_GetAverageMecSpeedMotor1_F());
  FocApp_WriteS32(&buffer[40], FocApp_FloatToS32(MC_GetTargetPosition1() * FOC_APP_RAD_TO_CDEG));
  FocApp_WriteS32(&buffer[44], FocApp_FloatToS32(MC_GetCurrentPosition1() * FOC_APP_RAD_TO_CDEG));
  MotorUart_GetDiagnostics(&uartDiagnostics);
  FocApp_WriteU32(&buffer[48], FOC_APP_UART_DIAG_MAGIC);
  buffer[52] = uartDiagnostics.initialized ? 1U : 0U;
  buffer[53] = uartDiagnostics.linkActive ? 1U : 0U;
  buffer[54] = uartDiagnostics.initStage;
  buffer[55] = uartDiagnostics.lastTxStatus;
  FocApp_WriteU32(&buffer[56], uartDiagnostics.uartError);
  FocApp_WriteU32(&buffer[60], uartDiagnostics.receivedBytes);
  FocApp_WriteU32(&buffer[64], uartDiagnostics.validCommandFrames);
  FocApp_WriteU32(&buffer[68], uartDiagnostics.crcErrors);
  FocApp_WriteU32(&buffer[72], uartDiagnostics.protocolErrors);
  FocApp_WriteU32(&buffer[76], uartDiagnostics.telemetryAttempts);
  FocApp_WriteU32(&buffer[80], uartDiagnostics.telemetrySent);
  FocApp_WriteU32(&buffer[84], uartDiagnostics.telemetryErrors);
  return FOC_APP_TELEMETRY_SIZE;
}

static uint8_t FocApp_McpCallback(uint16_t rxLength, uint8_t *rxBuffer,
                                  int16_t txSyncFreeSpace, uint16_t *txLength,
                                  uint8_t *txBuffer)
{
  uint8_t result = MCP_CMD_OK;

  if ((rxLength < 1U) || (rxBuffer == NULL) || (txLength == NULL) || (txBuffer == NULL))
  {
    return MCP_CMD_NOK;
  }

  switch ((FocApp_Command_t)rxBuffer[0])
  {
    case FOC_APP_CMD_GET_TELEMETRY:
      break;

    case FOC_APP_CMD_SET_MODE:
      if ((rxLength < 2U) || (rxBuffer[1] > (uint8_t)FOC_APP_MODE_POSITION))
      {
        result = MCP_CMD_NOK;
      }
      else
      {
        result = FocApp_SetControlMode((FocApp_Mode_t)rxBuffer[1])
          ? MCP_CMD_OK : MCP_CMD_NOK;
      }
      break;

    case FOC_APP_CMD_SET_SPEED_RPM:
      if (rxLength < 7U)
      {
        result = MCP_CMD_NOK;
      }
      else
      {
        int16_t targetRpm = FocApp_ReadS16(&rxBuffer[1]);
        uint32_t durationMs = FocApp_ReadU32(&rxBuffer[3]);
        result = FocApp_SetSpeedCommand(targetRpm, durationMs)
          ? MCP_CMD_OK : MCP_CMD_NOK;
      }
      break;

    case FOC_APP_CMD_SET_POSITION:
      if (rxLength < 9U)
      {
        result = MCP_CMD_NOK;
      }
      else
      {
        int32_t targetCdeg = FocApp_ReadS32(&rxBuffer[1]);
        uint32_t durationMs = FocApp_ReadU32(&rxBuffer[5]);
        result = FocApp_SetPositionCommand(targetCdeg, durationMs)
          ? MCP_CMD_OK : MCP_CMD_NOK;
      }
      break;

    case FOC_APP_CMD_START_MOTOR:
      result = MC_StartMotor1() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;

    case FOC_APP_CMD_STOP_MOTOR:
      (void)MC_StopMotor1();
      break;

    case FOC_APP_CMD_ACK_FAULT:
      result = MC_AcknowledgeFaultMotor1() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;

    case FOC_APP_CMD_ZERO_POSITION:
      /* Changing Theta to zero is not an encoder-zero operation and can cause
       * a sudden full-torque move. The MIX Qt uses a timed nearest-path 0 deg
       * position command instead, so reject this legacy command. */
      result = MCP_CMD_UNKNOWN;
      break;

    case FOC_APP_CMD_HOLD_POSITION:
      result = FocApp_HoldPosition() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;

    default:
      result = MCP_CMD_UNKNOWN;
      break;
  }

  if (result == MCP_CMD_OK)
  {
    *txLength = FocApp_BuildTelemetry(txBuffer, txSyncFreeSpace);
    if (*txLength == 0U)
    {
      result = MCP_ERROR_NO_TXSYNC_SPACE;
    }
  }
  else
  {
    *txLength = 0U;
  }
  return result;
}

void FocAppProtocol_Init(void)
{
  FocApp_ControlMode = FOC_APP_MODE_POSITION;
  FocApp_PreviousMotorState = MC_GetSTMStateMotor1();
  (void)MCP_RegisterCallBack(FOC_APP_MCP_CALLBACK_ID, FocApp_McpCallback);
}

void FocAppProtocol_Tick(void)
{
  MCI_State_t currentState = MC_GetSTMStateMotor1();

  if ((currentState == RUN) && (FocApp_PreviousMotorState != RUN))
  {
    if (FocApp_ControlMode == FOC_APP_MODE_POSITION)
    {
      FocApp_HoldCurrentPosition();
    }
    else
    {
      FocApp_EnterSpeedMode();
    }
  }
  FocApp_PreviousMotorState = currentState;
}
