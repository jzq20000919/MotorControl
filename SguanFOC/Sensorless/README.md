# Sensorless SguanFOC on X-STAR-S STM32G431RBT6

This is a separate port of upstream SguanFOC v3.1.0 floating-point sensorless
FOC. It uses mode 8: an IF forced ramp starts the rotor and a sliding-mode
observer (SMO) takes over above the configured transition speed. MT6701 and
TIM3 are deliberately not used by this version.

The board profile is 16 kHz PWM, two-shunt BC current sensing, a 20 V bus,
seven pole pairs, Rs=2.55 ohm, Ld=Lq=0.86 mH, and flux linkage 0.02246532 Wb.
The source code retains the upstream MIT license in `Library/LICENSE`.

## Integration

1. Follow [the CubeMX setup](CubeMX/STM32G431RBT6_SguanSensorless.md).
2. Add every C file in `Library`, plus
   `Port/Src/sguan_sensorless_stm32g431_port.c`, to the G431 project. Add
   `Library` and `Port/Inc` to its include paths.
3. Merge [app_sguan_sensorless.c](Example/app_sguan_sensorless.c) and the
   JEOS section from
   [stm32g4xx_it_sguan_sensorless.c](Example/stm32g4xx_it_sguan_sensorless.c).

`App_SguanSensorless_Init()` requests 0.50 A IF start current and 140 rad/s
(about 1337 rpm) mechanical speed. The speed must rise above the 130 rad/s
SMO threshold before closed-loop sensorless takeover can occur. The port
clamps commands to 0.80 A IF start current and 155 rad/s while commissioning.

## Tuning and safety

Run first with a current-limited supply and the shaft unloaded. The sign of
V/W current samples is set by `SGUAN_SENSORLESS_PHASE_V_DIR` and
`SGUAN_SENSORLESS_PHASE_W_DIR` in `Port/Inc/sguan_sensorless_stm32g431_port.h`.
If the ramp stalls, first verify ADC offsets and phase order, then adjust
`Target_IF_Iq`, the LTD ramp, SMO parameters, and the three sensorless
transition values in `Library/UserData_Parameter.h`.

The library’s bus-voltage and temperature user callbacks still return the
upstream sentinel. TIM1 BKIN is therefore the immediate protection path until
the board voltage divider and NTC conversion functions are implemented.
