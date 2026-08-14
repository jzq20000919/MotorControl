#ifndef SGUAN_SENSORLESS_PID_STM32G431_PORT_H
#define SGUAN_SENSORLESS_PID_STM32G431_PORT_H

#include <stdint.h>

/*
 * X-STAR-S FOC/BLDC board, STM32G431RBT6.
 * TIM1 CH1/2/3: PA8/PA9/PA10 (high side),
 * TIM1 CH1N/2N/3N: PB13/PA12/PB15 (low side).
 * ADC2 JDR1: phase V, PC4 / ADC2_IN5.
 * ADC1 JDR1: phase W, PB14 / ADC1_IN5.
 */
#define SGUAN_SENSORLESS_PID_PWM_FREQUENCY_HZ  16000U
#define SGUAN_SENSORLESS_PID_TIM1_ARR          5312U
#define SGUAN_SENSORLESS_PID_PHASE_V_DIR       1
#define SGUAN_SENSORLESS_PID_PHASE_W_DIR       1

/* Conservative initial limits; tune on a current-limited supply. */
#define SGUAN_SENSORLESS_PID_MAX_SPEED_RAD_S   155.0f
#define SGUAN_SENSORLESS_PID_MAX_START_IQ_A    0.80f

void SguanSensorlessPID_Port_InitialInit(void);
void SguanSensorlessPID_Port_StartMotor(void);
void SguanSensorlessPID_Port_DelayMs(uint32_t ms);
int32_t SguanSensorlessPID_Port_ReadPhaseCurrentRaw(uint8_t channel);
void SguanSensorlessPID_Port_SetPwmDuty(uint32_t duty_u,
                                        uint32_t duty_v,
                                        uint32_t duty_w);

/* Call from ADC1_2 IRQ, a 1 ms tick, and the foreground loop respectively. */
void SguanSensorlessPID_Port_AdcInjectedIrq(void);
void SguanSensorlessPID_Port_1msTask(void);
void SguanSensorlessPID_Port_MainTask(void);

void SguanSensorlessPID_Port_RequestStart(void);
void SguanSensorlessPID_Port_RequestStop(void);
void SguanSensorlessPID_Port_SetSpeedTarget(float rad_s);
void SguanSensorlessPID_Port_SetIfStartCurrent(float ampere);
void SguanSensorlessPID_Port_SetSpeedPid(float kp, float ki, float kd);

#endif
