# SguanFOC on X-STAR-S STM32G431RBT6

This directory ports upstream **SguanFOC v3.0.0 floating-point sensor FOC**
(MIT license retained in `Library`) to the X-STAR-S FOC/BLDC board.

For the separate encoder-free SMO implementation, see
[Sensorless/README.md](Sensorless/README.md).
For the standalone SMO + speed PID profile, see
[SensorlessPID/README.md](SensorlessPID/README.md).

The port is intentionally safe at power-up: PWM is not enabled until the host
application calls `SguanFOC_Port_RequestStart()`. Start-up then samples current
offsets, applies the upstream alignment vector, records the MT6701 electrical
offset, and enters current-loop mode with zero torque request.

## Hardware mapping

| Signal | Mapping |
|---|---|
| PWM U/V/W | TIM1 PA8/PA9/PA10; complementary PB13/PA12/PB15 |
| Current V/W | ADC2_IN5 PC4; ADC1_IN5 PB14 |
| Encoder | MT6701 ABI A/B to TIM3 PC6/PC7 |
| Break input | TIM1 BKIN PC13, active-low |

## Add to the CubeMX project

1. Apply the peripheral configuration in [STM32G431RBT6_SguanFOC.md](CubeMX/STM32G431RBT6_SguanFOC.md).
2. Add every `.c` file in `Library` and `Port/Src/sguan_stm32g431_port.c`; add
   `Library` and `Port/Inc` to the compiler include paths.
3. Merge [app_sguan_foc.c](Example/app_sguan_foc.c) into the generated
   application and merge the ADC interrupt body from
   [stm32g4xx_it_sguan_foc.c](Example/stm32g4xx_it_sguan_foc.c).
4. Before calling `SguanFOC_Port_RequestStart()`, confirm the DC bus is within
   range, hardware break is released, and the motor can turn freely.

## First commissioning

Start at zero torque. During alignment check that the shaft holds smoothly; if
not, stop immediately. Correct encoder direction and V/W current signs only
through the three macros in `Port/Inc/sguan_stm32g431_port.h`. The default
motor values are copied from the existing `MCSDK_FOC_MIX` project (2804,
7 pole pairs, 20 V, 2 A, 3 mΩ shunts) and must be retuned if the attached
motor differs.

`User_VBUS_DataGet()` and `User_Temperature_DataGet()` deliberately return the
upstream sentinel until the board-divider and NTC transfer functions are
calibrated. TIM1 BKIN remains the immediate hardware fault path; implement
those two functions before enabling software voltage/temperature protection.
