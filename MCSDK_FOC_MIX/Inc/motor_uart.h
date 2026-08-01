#ifndef MOTOR_UART_H
#define MOTOR_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool initialized;
  bool linkActive;
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
} MotorUart_Diagnostics_t;

bool MotorUart_Init(void);
void MotorUart_Process(void);
void MotorUart_GetDiagnostics(MotorUart_Diagnostics_t *diagnostics);

#endif /* MOTOR_UART_H */
