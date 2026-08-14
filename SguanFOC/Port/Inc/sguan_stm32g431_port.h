#ifndef SGUAN_STM32G431_PORT_H
#define SGUAN_STM32G431_PORT_H

#include <stdint.h>

/*
 * X-STAR-S FOC/BLDC board (STM32G431RBT6)
 * TIM1: PA8/PA9/PA10 and PB13/PA12/PB15, complementary three-phase PWM.
 * TIM3: PC6/PC7, MT6701 ABI A/B outputs in encoder mode.
 * ADC2 JDR1: phase V (PC4, ADC2_IN5); ADC1 JDR1: phase W (PB14, ADC1_IN5).
 */

#define SGUANFOC_PWM_FREQUENCY_HZ       16000U
#define SGUANFOC_TIM1_ARR               5312U
#define SGUANFOC_MT6701_ABI_PPR         1024U
#define SGUANFOC_MT6701_ABI_CPR         (4U * SGUANFOC_MT6701_ABI_PPR)

/* Change after the first alignment test if measured direction or current sign is reversed. */
#define SGUANFOC_ENCODER_DIR            (-1)
#define SGUANFOC_PHASE_V_DIR            (1)
#define SGUANFOC_PHASE_W_DIR            (1)

void SguanFOC_Port_Init(void);
void SguanFOC_Port_DelayMs(uint32_t ms);
int32_t SguanFOC_Port_ReadPhaseCurrentRaw(uint8_t channel);
float SguanFOC_Port_ReadEncoderRad(void);
void SguanFOC_Port_SetPwmDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w);

/* Call these thin wrappers from the project ISR/main loop. */
void SguanFOC_Port_AdcInjectedIrq(void);
void SguanFOC_Port_1msTask(void);
void SguanFOC_Port_MainTask(void);
void SguanFOC_Port_RequestStart(void);
void SguanFOC_Port_RequestStop(void);
uint8_t SguanFOC_Port_StartRequested(void);

#endif
