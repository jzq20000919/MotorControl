# STM32G431RBT6 CubeMX configuration for sensorless PID SguanFOC

Generate the STM32 HAL project before importing this port. Add all
`SensorlessPID/Library/*.c` files and `SensorlessPID/Port/Src/*.c`, then add
`SensorlessPID/Library` and `SensorlessPID/Port/Inc` to the include paths.

| Function | MCU resource | Board pins | Required setup |
|---|---|---|---|
| Three-phase PWM | TIM1 CH1/2/3 and CH1N/2N/3N | PA8/PA9/PA10 and PB13/PA12/PB15 | centre-aligned 16 kHz, complementary, 1 us dead time |
| ADC trigger | TIM1 CH4 | no board output | configure CC4 as the ADC injected trigger near the centre of the low-side sample window |
| Current V/W | ADC2 JDR1 / ADC1 JDR1 | PC4 ADC2_IN5 / PB14 ADC1_IN5 | injected rank 1, TIM1 CH4 trigger, 12-bit right aligned |
| Fault brake | TIM1 BKIN | PC13 | active-low external break, enabled |
| FOC interrupt | ADC1_2_IRQn | — | enable IRQ; service ADC1 JEOS before calling the high loop |

Use BC sampling: the port reads V from ADC2 as channel 0 and W from ADC1 as
channel 1, then SguanFOC derives U as `-(V + W)`. Do not left-align the ADC
data. Do not start regular ADC conversions on these channels; the port starts
and reads the injected conversions.

No encoder peripheral is configured in this version: leave TIM3, PC6, and PC7
available for other uses. To make 16 kHz centre-aligned PWM from a 170 MHz TIM1
clock, use a prescaler of 0 and an ARR matching
`SGUAN_SENSORLESS_PID_TIM1_ARR` (5312) in the port header. Set CH4 compare to
the measured safe current-sampling instant, then verify it with an oscilloscope
before enabling the motor bridge.

Merge the JEOS body from
`Example/stm32g4xx_it_sguan_sensorless_pid.c` with generated interrupts;
do not create two definitions of `ADC1_2_IRQHandler`.
