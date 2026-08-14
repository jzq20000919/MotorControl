/* Add this file to a CubeMX-generated STM32G431 project. */
#include "main.h"
#include "SguanFOC.h"
#include "sguan_stm32g431_port.h"

void App_SguanFOC_Init(void)
{
    /* Peripherals must already be initialized by CubeMX. This only requests a
       controlled offset/alignment sequence; it intentionally does not run at boot. */
    SguanFOC_Port_RequestStart();
}

void App_SguanFOC_Process(void)
{
    static uint32_t last_low_task_ms;

    SguanFOC_Port_MainTask();
    if ((HAL_GetTick() - last_low_task_ms) >= 1U) {
        last_low_task_ms++;
        SguanFOC_Port_1msTask();
    }
}

/* Start the motor only after the DC bus is present and the rotor is free to align.
   Call App_SguanFOC_Init() from a button/command handler, then call
   App_SguanFOC_Process() from while(1). */
