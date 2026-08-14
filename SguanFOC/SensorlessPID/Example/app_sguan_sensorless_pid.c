#include "sguan_sensorless_pid_stm32g431_port.h"

/* Call after MX_GPIO_Init(), MX_ADC1_Init(), MX_ADC2_Init(), and MX_TIM1_Init(). */
void App_SguanSensorlessPID_Init(void)
{
    /* Run once so SguanFOC performs its power-on initialization. */
    SguanSensorlessPID_Port_MainTask();

    /* Conservative first-run command. Values are restored after Transfer_Init. */
    SguanSensorlessPID_Port_SetSpeedPid(1.046875f,
                                        0.00030517578125f,
                                        0.0f);
    SguanSensorlessPID_Port_SetIfStartCurrent(0.50f);
    SguanSensorlessPID_Port_SetSpeedTarget(140.0f);
    SguanSensorlessPID_Port_RequestStart();
}

/* Call from a 1 kHz timer callback or a 1 ms RTOS task. */
void App_SguanSensorlessPID_1msTask(void)
{
    SguanSensorlessPID_Port_1msTask();
}

/* Call continuously from the foreground loop. */
void App_SguanSensorlessPID_MainLoop(void)
{
    SguanSensorlessPID_Port_MainTask();
}

void App_SguanSensorlessPID_Stop(void)
{
    SguanSensorlessPID_Port_RequestStop();
}
