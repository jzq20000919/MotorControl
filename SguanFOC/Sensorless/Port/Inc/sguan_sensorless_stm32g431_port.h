#ifndef SGUAN_SENSORLESS_STM32G431_PORT_H
#define SGUAN_SENSORLESS_STM32G431_PORT_H

#include <stdint.h>

/*
 * X-STAR-S FOC/BLDC board, STM32G431RBT6.
 * TIM1 CH1/2/3: PA8/PA9/PA10 (high side),
 * TIM1 CH1N/2N/3N: PB13/PA12/PB15 (low side).
 * ADC2 JDR1: phase V, PC4 / ADC2_IN5.
 * ADC1 JDR1: phase W, PB14 / ADC1_IN5.
 */
#define SGUAN_SENSORLESS_PWM_FREQUENCY_HZ  16000U
#define SGUAN_SENSORLESS_TIM1_ARR          5312U
#define SGUAN_SENSORLESS_PHASE_V_DIR       1
#define SGUAN_SENSORLESS_PHASE_W_DIR       1

/* Conservative initial limits; tune on a current-limited supply. */
#define SGUAN_SENSORLESS_MAX_SPEED_RAD_S   155.0f
#define SGUAN_SENSORLESS_MAX_START_IQ_A    0.80f

void SguanSensorless_Port_InitialInit(void);
void SguanSensorless_Port_StartMotor(void);
void SguanSensorless_Port_DelayMs(uint32_t ms);
int32_t SguanSensorless_Port_ReadPhaseCurrentRaw(uint8_t channel);
void SguanSensorless_Port_SetPwmDuty(uint32_t duty_u,
                                     uint32_t duty_v,
                                     uint32_t duty_w);

/* Call from ADC1_2 IRQ, a 1 ms tick, and the foreground loop respectively. */
void SguanSensorless_Port_AdcInjectedIrq(void);
void SguanSensorless_Port_1msTask(void);
void SguanSensorless_Port_MainTask(void);

void SguanSensorless_Port_RequestStart(void);
void SguanSensorless_Port_RequestStop(void);
void SguanSensorless_Port_SetSpeedTarget(float rad_s);
void SguanSensorless_Port_SetIfStartCurrent(float ampere);

#endif
