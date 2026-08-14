#include "sguan_sensorless_stm32g431_port.h"

/* Call after MX_GPIO_Init(), MX_ADC1_Init(), MX_ADC2_Init(), and MX_TIM1_Init(). */
void App_SguanSensorless_Init(void)
{
    /* Run once so SguanFOC performs its power-on initialization. */
    SguanSensorless_Port_MainTask();

    /* Conservative first-run command. Values are restored after Transfer_Init. */
    SguanSensorless_Port_SetIfStartCurrent(0.50f);
    SguanSensorless_Port_SetSpeedTarget(140.0f);
    SguanSensorless_Port_RequestStart();
}

/* Call from a 1 kHz timer callback or a 1 ms RTOS task. */
void App_SguanSensorless_1msTask(void)
{
    SguanSensorless_Port_1msTask();
}

/* Call continuously from the foreground loop. */
void App_SguanSensorless_MainLoop(void)
{
    SguanSensorless_Port_MainTask();
}

void App_SguanSensorless_Stop(void)
{
    SguanSensorless_Port_RequestStop();
}
