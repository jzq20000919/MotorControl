/*
 * Dedicated USART3 bridge for the ESP32 HMI.
 * PC10 is TX and PC11 is RX.  USART2/ASPEP remains dedicated to Qt/Motor Pilot.
 */
#include "motor_uart.h"

#include <limits.h>
#include <string.h>

#include "foc_app_protocol.h"
#include "mc_api.h"
#include "motor_uart_protocol.h"
#include "stm32g4xx_hal.h"

#define MOTOR_UART_LINK_TIMEOUT_MS       (300U)
#define MOTOR_UART_TELEMETRY_PERIOD_MS   (20U)
#define MOTOR_UART_CDEG_PER_RAD           (5729.577951308232F)
#define MOTOR_UART_RX_DMA_SIZE             (128U)

typedef struct
{
  UART_HandleTypeDef handle;
  DMA_HandleTypeDef rxDma;
  uint8_t rxDmaBuffer[MOTOR_UART_RX_DMA_SIZE];
  uint16_t rxTail;
  uint8_t parser[MOTOR_UART_MAX_FRAME_SIZE];
  uint8_t parserLength;
  uint8_t telemetrySequence;
  uint8_t lastCommandSequence;
  uint32_t lastCommandTick;
  uint32_t nextTelemetryTick;
  bool initialized;
  bool commandRejected;
  uint8_t initStage;
  uint8_t lastTxStatus;
  uint32_t uartError;
  uint32_t receivedBytes;
  uint32_t validCommandFrames;
  uint32_t crcErrors;
  uint32_t protocolErrors;
  uint32_t telemetryAttempts;
  uint32_t telemetrySent;
  uint32_t telemetryErrors;
} MotorUart_State_t;

static MotorUart_State_t motorUart;

/** @brief 执行 MotorUart_Crc16 对应的模块功能。 */
static uint16_t MotorUart_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

/** @brief 计算 MotorUart_ClampS16 对应的电机控制量或数学结果。 */
static int16_t MotorUart_ClampS16(int32_t value)
{
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return (int16_t)value;
}

/** @brief 执行 MotorUart_FloatToS32 对应的模块功能。 */
static int32_t MotorUart_FloatToS32(float value)
{
  if (value > 2147483000.0F) return INT32_MAX;
  if (value < -2147483000.0F) return INT32_MIN;
  return (int32_t)value;
}

/** @brief 计算 MotorUart_NormalizeCdeg 对应的电机控制量或数学结果。 */
static int32_t MotorUart_NormalizeCdeg(int32_t cdeg)
{
  cdeg %= 36000;
  return cdeg < 0 ? cdeg + 36000 : cdeg;
}

/** @brief 执行 MotorUart_LinkActive 对应的模块功能。 */
static bool MotorUart_LinkActive(void)
{
  return (motorUart.lastCommandTick != 0U) &&
         ((HAL_GetTick() - motorUart.lastCommandTick) <= MOTOR_UART_LINK_TIMEOUT_MS);
}

/** @brief 设置 MotorUart_SetNearestSingleTurnPosition 对应的控制参数、目标值或外设配置。 */
static bool MotorUart_SetNearestSingleTurnPosition(int32_t targetCdeg)
{
  const int32_t current = MotorUart_FloatToS32(MC_GetCurrentPosition1() * MOTOR_UART_CDEG_PER_RAD);
  const int32_t currentSingle = MotorUart_NormalizeCdeg(current);
  int32_t delta = MotorUart_NormalizeCdeg(targetCdeg) - currentSingle;
  if (delta > 18000) delta -= 36000;
  return FocApp_SetPositionCommand(current + delta, 1000U);
}

/** @brief 执行 MotorUart_HandleCommand 对应的模块功能。 */
static bool MotorUart_HandleCommand(const uint8_t *payload)
{
  const int32_t value = MotorUart_ReadS32(&payload[1]);
  switch ((MotorUart_Command_t)payload[0])
  {
    case MOTOR_UART_CMD_NOP:
    case MOTOR_UART_CMD_PING:
      return true;
    case MOTOR_UART_CMD_SET_MODE:
      if ((value != MOTOR_UART_MODE_SPEED) && (value != MOTOR_UART_MODE_POSITION)) return false;
      return FocApp_SetControlMode((FocApp_Mode_t)value);
    case MOTOR_UART_CMD_SET_SPEED_RPM:
      return FocApp_SetSpeedCommand(MotorUart_ClampS16(value), 150U);
    case MOTOR_UART_CMD_SET_POSITION_CDEG:
      return MotorUart_SetNearestSingleTurnPosition(value);
    case MOTOR_UART_CMD_START:
      return MC_StartMotor1();
    case MOTOR_UART_CMD_STOP:
      return MC_StopMotor1();
    case MOTOR_UART_CMD_ACK_FAULT:
      return MC_AcknowledgeFaultMotor1();
    case MOTOR_UART_CMD_ZERO_POSITION:
    default:
      return false;
  }
}

/** @brief 接收并解析 MotorUart_ParseByte 对应的数据或通信帧。 */
static void MotorUart_ParseByte(uint8_t byte)
{
  if (motorUart.parserLength == 0U)
  {
    if (byte == MOTOR_UART_SOF0) motorUart.parser[motorUart.parserLength++] = byte;
    return;
  }
  if (motorUart.parserLength == 1U)
  {
    if (byte == MOTOR_UART_SOF1) motorUart.parser[motorUart.parserLength++] = byte;
    else motorUart.parserLength = byte == MOTOR_UART_SOF0 ? 1U : 0U;
    return;
  }
  if (motorUart.parserLength >= MOTOR_UART_MAX_FRAME_SIZE)
  {
    motorUart.parserLength = 0U;
    return;
  }
  motorUart.parser[motorUart.parserLength++] = byte;
  if (motorUart.parserLength < 6U) return;

  const uint8_t length = motorUart.parser[5];
  const uint16_t frameLength = (uint16_t)(8U + length);
  if (length > MOTOR_UART_MAX_PAYLOAD)
  {
    motorUart.protocolErrors++;
    motorUart.parserLength = 0U;
    return;
  }
  if (motorUart.parserLength < frameLength) return;

  if (MotorUart_ReadU16(&motorUart.parser[6U + length]) !=
      MotorUart_Crc16(&motorUart.parser[2], (uint16_t)(4U + length)))
  {
    motorUart.crcErrors++;
  }
  else if ((motorUart.parser[2] == MOTOR_UART_PROTOCOL_VERSION) &&
           (motorUart.parser[3] == MOTOR_UART_FRAME_COMMAND) &&
           (length == MOTOR_UART_COMMAND_PAYLOAD_SIZE))
  {
    motorUart.commandRejected = !MotorUart_HandleCommand(&motorUart.parser[6]);
    motorUart.lastCommandSequence = motorUart.parser[4];
    motorUart.lastCommandTick = HAL_GetTick();
    motorUart.validCommandFrames++;
  }
  else
  {
    motorUart.protocolErrors++;
  }
  motorUart.parserLength = 0U;
}

/** @brief 获取 MotorUart_ReadDma 对应的状态、配置或计算结果。 */
static void MotorUart_ReadDma(void)
{
  const uint16_t head = (uint16_t)(
    MOTOR_UART_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(&motorUart.rxDma));

  while (motorUart.rxTail != head)
  {
    motorUart.receivedBytes++;
    MotorUart_ParseByte(motorUart.rxDmaBuffer[motorUart.rxTail]);
    motorUart.rxTail++;
    if (motorUart.rxTail >= MOTOR_UART_RX_DMA_SIZE)
    {
      motorUart.rxTail = 0U;
    }
  }
}

/** @brief 编码并发送 MotorUart_SendTelemetry 对应的数据或通信帧。 */
static void MotorUart_SendTelemetry(void)
{
  uint8_t frame[32] = {0U};
  uint8_t *payload = &frame[6];
  const uint16_t faults = MC_GetCurrentFaultsMotor1();
  const qd_f_t current = MC_GetIqdMotor1_F();
  const qd_f_t reference = MC_GetIqdrefMotor1_F();
  const int32_t position = MotorUart_FloatToS32(MC_GetCurrentPosition1() * MOTOR_UART_CDEG_PER_RAD);
  const int32_t target = MotorUart_FloatToS32(MC_GetTargetPosition1() * MOTOR_UART_CDEG_PER_RAD);
  uint8_t flags = 0U;
  if (MC_GetSTMStateMotor1() == RUN) flags |= MOTOR_UART_STATUS_MOTOR_RUNNING;
  if (faults != 0U) flags |= MOTOR_UART_STATUS_MOTOR_FAULT;
  if (motorUart.commandRejected) flags |= MOTOR_UART_STATUS_COMMAND_REJECTED;
  if (MotorUart_LinkActive()) flags |= MOTOR_UART_STATUS_LINK_ACTIVE;
  payload[0] = flags;
  payload[1] = FocApp_GetControlMode() == FOC_APP_MODE_POSITION ? MOTOR_UART_MODE_POSITION : MOTOR_UART_MODE_SPEED;
  MotorUart_WriteU16(&payload[2], faults);
  MotorUart_WriteS16(&payload[4], MotorUart_ClampS16((int32_t)MC_GetAverageMecSpeedMotor1_F()));
  MotorUart_WriteS16(&payload[6], MotorUart_ClampS16((int32_t)MC_GetMecSpeedReferenceMotor1_F()));
  MotorUart_WriteU16(&payload[8], (uint16_t)MotorUart_NormalizeCdeg(position));
  MotorUart_WriteU16(&payload[10], (uint16_t)MotorUart_NormalizeCdeg(target));
  MotorUart_WriteS16(&payload[12], MotorUart_ClampS16(target - position));
  MotorUart_WriteS16(&payload[14], MotorUart_ClampS16((int32_t)(current.q * 1000.0F)));
  MotorUart_WriteS16(&payload[16], MotorUart_ClampS16((int32_t)(current.d * 1000.0F)));
  MotorUart_WriteS16(&payload[18], MotorUart_ClampS16((int32_t)(reference.q * 1000.0F)));
  frame[0] = MOTOR_UART_SOF0;
  frame[1] = MOTOR_UART_SOF1;
  frame[2] = MOTOR_UART_PROTOCOL_VERSION;
  frame[3] = MOTOR_UART_FRAME_TELEMETRY;
  frame[4] = motorUart.telemetrySequence++;
  frame[5] = MOTOR_UART_TELEMETRY_PAYLOAD_SIZE;
  MotorUart_WriteU16(&frame[30], MotorUart_Crc16(&frame[2], 28U));
  motorUart.telemetryAttempts++;
  const HAL_StatusTypeDef status =
    HAL_UART_Transmit(&motorUart.handle, frame, sizeof(frame), 2U);
  motorUart.lastTxStatus = (uint8_t)status;
  motorUart.uartError = HAL_UART_GetError(&motorUart.handle);
  if (status == HAL_OK)
  {
    motorUart.telemetrySent++;
  }
  else
  {
    motorUart.telemetryErrors++;
  }
}

/** @brief 初始化 MotorUart_Init 所属模块、外设或运行状态。 */
bool MotorUart_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  memset(&motorUart, 0, sizeof(motorUart));
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &gpio);
  motorUart.handle.Instance = USART3;
  motorUart.handle.Init.BaudRate = MOTOR_UART_BAUD_RATE;
  motorUart.handle.Init.WordLength = UART_WORDLENGTH_8B;
  motorUart.handle.Init.StopBits = UART_STOPBITS_1;
  motorUart.handle.Init.Parity = UART_PARITY_NONE;
  motorUart.handle.Init.Mode = UART_MODE_TX_RX;
  motorUart.handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  motorUart.handle.Init.OverSampling = UART_OVERSAMPLING_16;
  motorUart.handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  motorUart.handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  motorUart.handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  motorUart.initStage = 1U;
  motorUart.lastTxStatus = (uint8_t)HAL_UART_Init(&motorUart.handle);
  if (motorUart.lastTxStatus != (uint8_t)HAL_OK)
  {
    motorUart.uartError = HAL_UART_GetError(&motorUart.handle);
    return false;
  }
  motorUart.initStage = 2U;
  if (HAL_UARTEx_SetTxFifoThreshold(&motorUart.handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) return false;
  motorUart.initStage = 3U;
  if (HAL_UARTEx_SetRxFifoThreshold(&motorUart.handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) return false;
  motorUart.initStage = 4U;
  if (HAL_UARTEx_DisableFifoMode(&motorUart.handle) != HAL_OK) return false;
  motorUart.rxDma.Instance = DMA1_Channel3;
  motorUart.rxDma.Init.Request = DMA_REQUEST_USART3_RX;
  motorUart.rxDma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  motorUart.rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
  motorUart.rxDma.Init.MemInc = DMA_MINC_ENABLE;
  motorUart.rxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  motorUart.rxDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  motorUart.rxDma.Init.Mode = DMA_CIRCULAR;
  motorUart.rxDma.Init.Priority = DMA_PRIORITY_HIGH;
  motorUart.initStage = 5U;
  if (HAL_DMA_Init(&motorUart.rxDma) != HAL_OK) return false;
  __HAL_LINKDMA(&motorUart.handle, hdmarx, motorUart.rxDma);
  motorUart.initStage = 6U;
  if (HAL_UART_Receive_DMA(&motorUart.handle, motorUart.rxDmaBuffer,
                           MOTOR_UART_RX_DMA_SIZE) != HAL_OK) return false;
  motorUart.nextTelemetryTick = HAL_GetTick() + MOTOR_UART_TELEMETRY_PERIOD_MS;
  motorUart.initialized = true;
  motorUart.initStage = 7U;
  return true;
}

/** @brief 执行 MotorUart_Process 对应的周期任务或电机控制流程。 */
void MotorUart_Process(void)
{
  if (!motorUart.initialized) return;
  MotorUart_ReadDma();
  const uint32_t now = HAL_GetTick();
  /*
   * Emit the same startup telemetry as the proven 2804 implementation.  The
   * LINK_ACTIVE bit still remains clear until a valid ESP32 command arrives,
   * but ESP32 RX can now distinguish a missing return wire from a bad command.
   */
  if ((int32_t)(now - motorUart.nextTelemetryTick) >= 0)
  {
    motorUart.nextTelemetryTick = now + MOTOR_UART_TELEMETRY_PERIOD_MS;
    MotorUart_SendTelemetry();
  }
}

/** @brief 获取 MotorUart_GetDiagnostics 对应的状态、配置或计算结果。 */
void MotorUart_GetDiagnostics(MotorUart_Diagnostics_t *diagnostics)
{
  if (diagnostics == NULL) return;
  diagnostics->initialized = motorUart.initialized;
  diagnostics->linkActive = MotorUart_LinkActive();
  diagnostics->initStage = motorUart.initStage;
  diagnostics->lastTxStatus = motorUart.lastTxStatus;
  diagnostics->uartError = motorUart.uartError;
  diagnostics->receivedBytes = motorUart.receivedBytes;
  diagnostics->validCommandFrames = motorUart.validCommandFrames;
  diagnostics->crcErrors = motorUart.crcErrors;
  diagnostics->protocolErrors = motorUart.protocolErrors;
  diagnostics->telemetryAttempts = motorUart.telemetryAttempts;
  diagnostics->telemetrySent = motorUart.telemetrySent;
  diagnostics->telemetryErrors = motorUart.telemetryErrors;
}
