#include "stm32g4xx_hal.h"
#include "SguanFOC.h"
#include "sguan_stm32g431_port.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

static volatile uint8_t start_requested;

static uint16_t clamp_duty(uint16_t duty)
{
    return (duty > SGUANFOC_TIM1_ARR) ? SGUANFOC_TIM1_ARR : duty;
}

void SguanFOC_Port_Init(void)
{
    /* ADC calibration must precede injected conversions. ADC data alignment is right. */
    (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_InjectedStart(&hadc1);
    (void)HAL_ADCEx_InjectedStart(&hadc2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void SguanFOC_Port_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

int32_t SguanFOC_Port_ReadPhaseCurrentRaw(uint8_t channel)
{
    /* These are the most recently completed synchronized injected conversions. */
    if (channel == 0U) {
        return (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    }
    return (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

float SguanFOC_Port_ReadEncoderRad(void)
{
    const float scale = 6.2831853071795864769f / (float)SGUANFOC_MT6701_ABI_CPR;
    uint32_t count = __HAL_TIM_GET_COUNTER(&htim3);
    return (float)(count % SGUANFOC_MT6701_ABI_CPR) * scale;
}

void SguanFOC_Port_SetPwmDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clamp_duty(duty_u));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, clamp_duty(duty_v));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clamp_duty(duty_w));
}

void SguanFOC_Port_AdcInjectedIrq(void)
{
    SguanFOC_High_Loop();
}

void SguanFOC_Port_1msTask(void)
{
    SguanFOC_Low_Loop();
}

void SguanFOC_Port_MainTask(void)
{
    SguanFOC_main_Loop();
}

void SguanFOC_Port_RequestStart(void)
{
    start_requested = 1U;
    if (Sguan.status == MOTOR_STATUS_STANDBY) {
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }
}

void SguanFOC_Port_RequestStop(void)
{
    start_requested = 0U;
    Sguan.status = MOTOR_STATUS_DISABLED;
    SguanFOC_Port_SetPwmDuty(0U, 0U, 0U);
}

uint8_t SguanFOC_Port_StartRequested(void)
{
    return start_requested;
}
