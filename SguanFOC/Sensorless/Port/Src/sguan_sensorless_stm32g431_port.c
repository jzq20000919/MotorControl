#include "stm32g4xx_hal.h"
#include "SguanFOC.h"
#include "sguan_sensorless_stm32g431_port.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;

static uint8_t motor_peripherals_started;
static float requested_speed_rad_s;
static float requested_if_start_current_a;

static uint32_t clamp_duty(uint32_t duty)
{
    return (duty > SGUAN_SENSORLESS_TIM1_ARR) ? SGUAN_SENSORLESS_TIM1_ARR : duty;
}

void SguanSensorless_Port_InitialInit(void)
{
    /* Keep bridge commands inactive until SguanFOC enters its start sequence. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
}

void SguanSensorless_Port_StartMotor(void)
{
    if (motor_peripherals_started != 0U) {
        return;
    }

    /* CubeMX must configure both ADCs for right-aligned injected rank 1. */
    (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_InjectedStart(&hadc1);
    (void)HAL_ADCEx_InjectedStart(&hadc2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    motor_peripherals_started = 1U;
}

void SguanSensorless_Port_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

int32_t SguanSensorless_Port_ReadPhaseCurrentRaw(uint8_t channel)
{
    /* Channel 0 is phase V (ADC2); channel 1 is phase W (ADC1). */
    if (channel == 0U) {
        return (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    }
    return (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

void SguanSensorless_Port_SetPwmDuty(uint32_t duty_u,
                                     uint32_t duty_v,
                                     uint32_t duty_w)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clamp_duty(duty_u));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, clamp_duty(duty_v));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clamp_duty(duty_w));
}

void SguanSensorless_Port_AdcInjectedIrq(void)
{
    SguanFOC_High_Loop();
}

void SguanSensorless_Port_1msTask(void)
{
    SguanFOC_Low_Loop();
    /* Transfer_Init resets user targets. Restore requests only after it completes. */
    if (Sguan.status >= MOTOR_STATUS_IDLE) {
        Sguan.Func_Set_Velocity(requested_speed_rad_s);
        Sguan.foc.Target_IF_Iq = requested_if_start_current_a;
    }
}

void SguanSensorless_Port_MainTask(void)
{
    SguanFOC_main_Loop();
}

void SguanSensorless_Port_RequestStart(void)
{
    Sguan.Func_Start();
}

void SguanSensorless_Port_RequestStop(void)
{
    Sguan.Func_Stop();
    SguanSensorless_Port_SetPwmDuty(0U, 0U, 0U);
}

void SguanSensorless_Port_SetSpeedTarget(float rad_s)
{
    if (rad_s > SGUAN_SENSORLESS_MAX_SPEED_RAD_S) {
        rad_s = SGUAN_SENSORLESS_MAX_SPEED_RAD_S;
    } else if (rad_s < -SGUAN_SENSORLESS_MAX_SPEED_RAD_S) {
        rad_s = -SGUAN_SENSORLESS_MAX_SPEED_RAD_S;
    }
    requested_speed_rad_s = rad_s;
    Sguan.Func_Set_Velocity(rad_s);
}

void SguanSensorless_Port_SetIfStartCurrent(float ampere)
{
    if (ampere > SGUAN_SENSORLESS_MAX_START_IQ_A) {
        ampere = SGUAN_SENSORLESS_MAX_START_IQ_A;
    } else if (ampere < -SGUAN_SENSORLESS_MAX_START_IQ_A) {
        ampere = -SGUAN_SENSORLESS_MAX_START_IQ_A;
    }
    requested_if_start_current_a = ampere;
    Sguan.foc.Target_IF_Iq = ampere;
}
