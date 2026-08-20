/**
  ******************************************************************************
  * @file    motor_can.c
  * @brief   ESP32 HMI command and telemetry bridge over FDCAN1.
  ******************************************************************************
  */
#include "motor_can.h"

#include <stdint.h>
#include <string.h>

#include "foc_app_protocol.h"
#include "main.h"
#include "mc_api.h"
#include "motor_can_protocol.h"

#define MOTOR_CAN_TELEMETRY_PERIOD_MS       (20UL)
#define MOTOR_CAN_LINK_TIMEOUT_MS           (300UL)
#define MOTOR_CAN_INIT_RETRY_MS              (500UL)
#define MOTOR_CAN_BUS_OFF_RECOVERY_MS        (100UL)
#define MOTOR_CAN_SPEED_RAMP_MS             (150UL)
#define MOTOR_CAN_POSITION_MIN_DURATION_MS  (200UL)
#define MOTOR_CAN_POSITION_CDEG_PER_SECOND  (18000UL)

static FDCAN_HandleTypeDef MotorCan_Handle;       /**< FDCAN1 外设配置及运行状态句柄。 */
static bool MotorCan_Ready;                      /**< 为 true 时 FDCAN 已初始化、过滤器已配置且节点已启动。 */
static uint32_t MotorCan_LastCommandTick;        /**< 最近收到有效 ESP32 命令的 HAL 毫秒节拍。 */
static uint32_t MotorCan_LastTelemetryTick;      /**< 最近一次发送三组遥测帧的 HAL 毫秒节拍。 */
static uint32_t MotorCan_LastInitAttemptTick;    /**< 最近尝试初始化 FDCAN 的 HAL 毫秒节拍。 */
static uint32_t MotorCan_LastRecoveryTick;       /**< 最近尝试从 Bus-Off 恢复的 HAL 毫秒节拍。 */
static uint8_t MotorCan_LastSequence;            /**< 最近接收命令帧中的序列号，用于状态回显与诊断。 */
static uint8_t MotorCan_LastCommand;             /**< 最近接收命令帧中的命令码，用于状态回显。 */
static bool MotorCan_CommandRejected;            /**< 为 true 时表示最近一条非 PING 命令未被应用层接受。 */

/** @brief 计算 MotorCan_ClampS16 对应的电机控制量或数学结果。 */
static int16_t MotorCan_ClampS16(int32_t value)
{
  if (value > INT16_MAX)
  {
    return INT16_MAX;
  }
  if (value < INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)value;
}

/** @brief 执行 MotorCan_FloatToS32 对应的模块功能。 */
static int32_t MotorCan_FloatToS32(float value)
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

/** @brief 计算 MotorCan_NormalizeCdeg 对应的电机控制量或数学结果。 */
static int32_t MotorCan_NormalizeCdeg(int32_t positionCdeg)
{
  /* 位置对一圈 36000 个 0.01° 单位取模后的临时结果。 */
  int32_t normalized = positionCdeg % 36000;
  if (normalized < 0)
  {
    normalized += 36000;
  }
  return normalized;
}

/** @brief 执行 MotorCan_LinkActive 对应的模块功能。 */
static bool MotorCan_LinkActive(uint32_t now)
{
  return (MotorCan_LastCommandTick != 0UL) &&
         ((now - MotorCan_LastCommandTick) <= MOTOR_CAN_LINK_TIMEOUT_MS);
}

/** @brief 编码并发送 MotorCan_Send 对应的数据或通信帧。 */
static bool MotorCan_Send(uint32_t identifier, const uint8_t data[8])
{
  /* 当前待发送经典 CAN 数据帧的 FDCAN 帧头配置。 */
  FDCAN_TxHeaderTypeDef header = {0};

  if (HAL_FDCAN_GetTxFifoFreeLevel(&MotorCan_Handle) == 0UL)
  {
    return false;
  }

  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  return HAL_FDCAN_AddMessageToTxFifoQ(
           &MotorCan_Handle, &header, data) == HAL_OK;
}

/** @brief 设置 MotorCan_SetNearestSingleTurnPosition 对应的控制参数、目标值或外设配置。 */
static bool MotorCan_SetNearestSingleTurnPosition(int32_t targetSingleTurnCdeg)
{
  /* MCSDK 多圈机械位置换算后的当前位置，单位为 0.01°。 */
  const int32_t currentCdeg = MotorCan_FloatToS32(
    MC_GetCurrentPosition1() * 5729.577951308232F);
  /* 归一化到 0～35999 范围的当前单圈位置，单位为 0.01°。 */
  const int32_t currentSingleTurn = MotorCan_NormalizeCdeg(currentCdeg);
  /* 目标与当前位置之间的角差，随后修正为不超过半圈的最短路径。 */
  int32_t delta = MotorCan_NormalizeCdeg(targetSingleTurnCdeg) - currentSingleTurn;
  uint32_t distance;    /* 最短路径的绝对角距离，单位为 0.01°。 */
  uint32_t durationMs;  /* 按目标运动速度计算出的轨迹持续时间，单位为 ms。 */

  if (delta > 18000)
  {
    delta -= 36000;
  }
  else if (delta < -18000)
  {
    delta += 36000;
  }

  distance = (uint32_t)((delta < 0) ? -delta : delta);
  durationMs = (distance * 1000UL) / MOTOR_CAN_POSITION_CDEG_PER_SECOND;
  if (durationMs < MOTOR_CAN_POSITION_MIN_DURATION_MS)
  {
    durationMs = MOTOR_CAN_POSITION_MIN_DURATION_MS;
  }
  return FocApp_SetPositionCommand(currentCdeg + delta, durationMs);
}

/** @brief 执行 MotorCan_ExecuteCommand 对应的周期任务或电机控制流程。 */
static bool MotorCan_ExecuteCommand(const uint8_t data[8])
{
  /* 命令帧 byte2 中携带的协议命令码。 */
  const MotorCan_Command_t command = (MotorCan_Command_t)data[2];

  switch (command)
  {
    case MOTOR_CAN_CMD_NOP:
    case MOTOR_CAN_CMD_PING:
      return true;

    case MOTOR_CAN_CMD_SET_MODE:
      return FocApp_SetControlMode((FocApp_Mode_t)data[3]);

    case MOTOR_CAN_CMD_SET_SPEED_RPM:
      return FocApp_SetSpeedCommand(
        MotorCan_ReadS16(&data[3]), MOTOR_CAN_SPEED_RAMP_MS);

    case MOTOR_CAN_CMD_SET_POSITION_CDEG:
      return MotorCan_SetNearestSingleTurnPosition(MotorCan_ReadS32(&data[3]));

    case MOTOR_CAN_CMD_START:
      return (MC_GetSTMStateMotor1() == RUN) ? true : MC_StartMotor1();

    case MOTOR_CAN_CMD_STOP:
      (void)MC_StopMotor1();
      return true;

    case MOTOR_CAN_CMD_ACK_FAULT:
      return MC_AcknowledgeFaultMotor1();

    case MOTOR_CAN_CMD_ZERO_POSITION:
    default:
      /* Changing the estimator angle is unsafe; reject the legacy command. */
      return false;
  }
}

/** @brief 执行 MotorCan_ProcessRx 对应的周期任务或电机控制流程。 */
static void MotorCan_ProcessRx(void)
{
  while (HAL_FDCAN_GetRxFifoFillLevel(
           &MotorCan_Handle, FDCAN_RX_FIFO0) > 0UL)
  {
    FDCAN_RxHeaderTypeDef header; /* 从 RX FIFO0 读出的帧格式、ID 和长度信息。 */
    uint8_t data[8];              /* 从 RX FIFO0 读出的 8 字节命令负载。 */

    if (HAL_FDCAN_GetRxMessage(
          &MotorCan_Handle, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      break;
    }
    if ((header.Identifier != MOTOR_CAN_ID_COMMAND) ||
        (header.IdType != FDCAN_STANDARD_ID) ||
        (header.RxFrameType != FDCAN_DATA_FRAME) ||
        (header.FDFormat != FDCAN_CLASSIC_CAN) ||
        (header.DataLength != FDCAN_DLC_BYTES_8) ||
        (data[0] != MOTOR_CAN_PROTOCOL_VERSION))
    {
      continue;
    }

    MotorCan_LastCommandTick = HAL_GetTick();
    MotorCan_LastSequence = data[1];
    MotorCan_LastCommand = data[2];
    if ((MotorCan_Command_t)data[2] != MOTOR_CAN_CMD_PING)
    {
      MotorCan_CommandRejected = !MotorCan_ExecuteCommand(data);
    }
    else
    {
      (void)MotorCan_ExecuteCommand(data);
    }
  }
}

/** @brief 执行 MotorCan_ServiceBus 对应的模块功能。 */
static bool MotorCan_ServiceBus(uint32_t now)
{
  /* FDCAN 控制器当前 Bus-Off、错误被动等协议状态。 */
  FDCAN_ProtocolStatusTypeDef status = {0};

  if (HAL_FDCAN_GetProtocolStatus(&MotorCan_Handle, &status) != HAL_OK)
  {
    return false;
  }
  if (status.BusOff == 0U)
  {
    return true;
  }

  /*
   * M_CAN enters INIT when the transmit error counter reaches Bus-Off.
   * The HAL handle does not recover its state automatically, so explicitly
   * stop/start the node.  Clear the link first: telemetry must wait until a
   * fresh ESP32 command proves that another active node can acknowledge it.
   */
  MotorCan_LastCommandTick = 0UL;
  if ((now - MotorCan_LastRecoveryTick) < MOTOR_CAN_BUS_OFF_RECOVERY_MS)
  {
    return false;
  }
  MotorCan_LastRecoveryTick = now;

  if ((HAL_FDCAN_Stop(&MotorCan_Handle) != HAL_OK) ||
      (HAL_FDCAN_Start(&MotorCan_Handle) != HAL_OK))
  {
    MotorCan_Ready = false;
    return false;
  }

  MotorCan_LastTelemetryTick = now;
  return false;
}

/** @brief 编码并发送 MotorCan_SendStatus 对应的数据或通信帧。 */
static void MotorCan_SendStatus(uint32_t now)
{
  uint8_t data[8] = {0}; /* ID 0x180 状态遥测帧的 8 字节负载。 */
  uint8_t flags = 0U;    /* 控制模式、运行、故障、链路和拒绝状态的位集合。 */
  /* MCSDK 当前故障位集合，将写入状态帧 byte6～7。 */
  const uint16_t currentFaults = MC_GetCurrentFaultsMotor1();
  /* MCSDK 当前电机状态机状态，用于判断是否处于 RUN。 */
  const MCI_State_t motorState = MC_GetSTMStateMotor1();

  if (FocApp_GetControlMode() == FOC_APP_MODE_POSITION)
  {
    flags |= MOTOR_CAN_STATUS_POSITION_MODE;
  }
  if (motorState == RUN)
  {
    flags |= MOTOR_CAN_STATUS_MOTOR_RUNNING;
  }
  if (currentFaults != 0U)
  {
    flags |= MOTOR_CAN_STATUS_MOTOR_FAULT;
  }
  if (MotorCan_LinkActive(now))
  {
    flags |= MOTOR_CAN_STATUS_LINK_ACTIVE;
  }
  if (MotorCan_CommandRejected)
  {
    flags |= MOTOR_CAN_STATUS_COMMAND_REJECTED;
  }

  data[0] = MOTOR_CAN_PROTOCOL_VERSION;
  data[1] = MotorCan_LastSequence;
  data[2] = MotorCan_LastCommand;
  data[3] = flags;
  MotorCan_WriteS16(
    &data[4], MotorCan_ClampS16((int32_t)MC_GetAverageMecSpeedMotor1_F()));
  MotorCan_WriteU16(&data[6], currentFaults);
  (void)MotorCan_Send(MOTOR_CAN_ID_STATUS, data);
}

/** @brief 编码并发送 MotorCan_SendReferences 对应的数据或通信帧。 */
static void MotorCan_SendReferences(void)
{
  uint8_t data[8] = {0}; /* ID 0x181 速度与位置参考遥测帧的负载。 */
  /* MCSDK 多圈机械当前位置换算值，单位为 0.01°。 */
  const int32_t currentCdeg = MotorCan_FloatToS32(
    MC_GetCurrentPosition1() * 5729.577951308232F);
  /* MCSDK 多圈位置轨迹目标换算值，单位为 0.01°。 */
  const int32_t targetCdeg = MotorCan_FloatToS32(
    MC_GetTargetPosition1() * 5729.577951308232F);

  MotorCan_WriteS16(
    &data[0], MotorCan_ClampS16((int32_t)MC_GetMecSpeedReferenceMotor1_F()));
  MotorCan_WriteU16(&data[2], (uint16_t)MotorCan_NormalizeCdeg(currentCdeg));
  MotorCan_WriteU16(&data[4], (uint16_t)MotorCan_NormalizeCdeg(targetCdeg));
  MotorCan_WriteS16(&data[6], MotorCan_ClampS16(targetCdeg - currentCdeg));
  (void)MotorCan_Send(MOTOR_CAN_ID_REFERENCES, data);
}

/** @brief 编码并发送 MotorCan_SendElectrical 对应的数据或通信帧。 */
static void MotorCan_SendElectrical(void)
{
  uint8_t data[8] = {0}; /* ID 0x182 d/q 轴电流遥测帧的负载。 */
  /* MCSDK 测得的 q/d 轴实际电流，结构成员单位为 A。 */
  const qd_f_t current = MC_GetIqdMotor1_F();
  /* MCSDK 当前 q/d 轴电流参考，结构成员单位为 A。 */
  const qd_f_t reference = MC_GetIqdrefMotor1_F();

  MotorCan_WriteS16(
    &data[0], MotorCan_ClampS16(MotorCan_FloatToS32(current.q * 1000.0F)));
  MotorCan_WriteS16(
    &data[2], MotorCan_ClampS16(MotorCan_FloatToS32(current.d * 1000.0F)));
  MotorCan_WriteS16(
    &data[4], MotorCan_ClampS16(MotorCan_FloatToS32(reference.q * 1000.0F)));
  MotorCan_WriteS16(
    &data[6], MotorCan_ClampS16(MotorCan_FloatToS32(reference.d * 1000.0F)));
  (void)MotorCan_Send(MOTOR_CAN_ID_ELECTRICAL, data);
}

/** @brief 初始化 MotorCan_Init 所属模块、外设或运行状态。 */
bool MotorCan_Init(void)
{
  GPIO_InitTypeDef gpio = {0};               /* PA11/PB9 FDCAN 复用功能的 GPIO 配置。 */
  RCC_PeriphCLKInitTypeDef clock = {0};      /* FDCAN 外设时钟源选择配置。 */
  FDCAN_FilterTypeDef filter = {0};          /* 仅允许命令 ID 0x100 进入 FIFO0 的过滤器。 */

  MotorCan_Ready = false;
  MotorCan_LastInitAttemptTick = HAL_GetTick();
  MotorCan_LastCommandTick = 0UL;
  MotorCan_LastTelemetryTick = 0UL;
  MotorCan_LastRecoveryTick = 0UL;
  MotorCan_LastSequence = 0U;
  MotorCan_LastCommand = (uint8_t)MOTOR_CAN_CMD_NOP;
  MotorCan_CommandRejected = false;

  clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
  {
    return false;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN1;
  gpio.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOA, &gpio);              /* PA11: FDCAN1_RX */
  gpio.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &gpio);              /* PB9: FDCAN1_TX */

  MotorCan_Handle.Instance = FDCAN1;
  MotorCan_Handle.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  MotorCan_Handle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  MotorCan_Handle.Init.Mode = FDCAN_MODE_NORMAL;
  MotorCan_Handle.Init.AutoRetransmission = ENABLE;
  MotorCan_Handle.Init.TransmitPause = DISABLE;
  MotorCan_Handle.Init.ProtocolException = DISABLE;
  /* 170 MHz / 17 / (1 + 15 + 4) = 500 kbit/s, sample point 80%. */
  MotorCan_Handle.Init.NominalPrescaler = 17U;
  MotorCan_Handle.Init.NominalSyncJumpWidth = 4U;
  MotorCan_Handle.Init.NominalTimeSeg1 = 15U;
  MotorCan_Handle.Init.NominalTimeSeg2 = 4U;
  MotorCan_Handle.Init.DataPrescaler = 17U;
  MotorCan_Handle.Init.DataSyncJumpWidth = 4U;
  MotorCan_Handle.Init.DataTimeSeg1 = 15U;
  MotorCan_Handle.Init.DataTimeSeg2 = 4U;
  MotorCan_Handle.Init.StdFiltersNbr = 1U;
  MotorCan_Handle.Init.ExtFiltersNbr = 0U;
  MotorCan_Handle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&MotorCan_Handle) != HAL_OK)
  {
    return false;
  }

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤器就放入FIFO
  filter.FilterID1 = MOTOR_CAN_ID_COMMAND;
  filter.FilterID2 = 0x7FFU;
  if ((HAL_FDCAN_ConfigFilter(&MotorCan_Handle, &filter) != HAL_OK) ||
      (HAL_FDCAN_ConfigGlobalFilter(
         &MotorCan_Handle, FDCAN_REJECT, FDCAN_REJECT,
         FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
      (HAL_FDCAN_Start(&MotorCan_Handle) != HAL_OK))
  {
    return false;
  }

  MotorCan_Ready = true;
  return true;
}

/** @brief 执行 MotorCan_Tick 对应的周期任务或电机控制流程。 */
void MotorCan_Tick(void)
{
  /* 本轮周期处理开始时读取的 HAL 毫秒节拍。 */
  uint32_t now = HAL_GetTick();

  if (!MotorCan_Ready)
  {
    if ((now - MotorCan_LastInitAttemptTick) >= MOTOR_CAN_INIT_RETRY_MS)
    {
      (void)MotorCan_Init();
    }
    return;
  }

  if (!MotorCan_ServiceBus(now))
  {
    return;
  }
  MotorCan_ProcessRx();
  now = HAL_GetTick();
  if (MotorCan_LinkActive(now) &&
      ((now - MotorCan_LastTelemetryTick) >= MOTOR_CAN_TELEMETRY_PERIOD_MS))
  {
    MotorCan_LastTelemetryTick = now;
    MotorCan_SendStatus(now);
    MotorCan_SendReferences();
    MotorCan_SendElectrical();
  }
}
