/* Merge this handler into the CubeMX-generated stm32g4xx_it.c. */
#include "main.h"
#include "sguan_stm32g431_port.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void ADC1_2_IRQHandler(void)
{
    /* ADC1 is phase W and is the synchronization owner. Clear both JEOS flags
       before entering the 16 kHz FOC loop. */
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS) != RESET) {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS);
        SguanFOC_Port_AdcInjectedIrq();
    }
}
