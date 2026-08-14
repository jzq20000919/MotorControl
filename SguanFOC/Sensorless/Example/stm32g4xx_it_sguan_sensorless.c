#include "stm32g4xx_hal.h"
#include "sguan_sensorless_stm32g431_port.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* Merge this JEOS handling into the CubeMX-generated ADC1_2_IRQHandler. */
void ADC1_2_IRQHandler(void)
{
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS) != RESET) {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS);
        SguanSensorless_Port_AdcInjectedIrq();
    }
}
