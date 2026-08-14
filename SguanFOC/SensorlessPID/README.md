# Sensorless PID SguanFOC on X-STAR-S STM32G431RBT6

This standalone profile uses the upstream SguanFOC v3.1.0 mode 8 signal path:
IF forced start, SMO rotor-state estimation, D/Q current PI loops, and an
outer mechanical-speed PID loop. It does not use MT6701 or TIM3.

The configured gains reproduce the values requested from the MCSDK screen:

| Loop | Kp | Ki | Kd | Period |
|---|---:|---:|---:|---:|
| Id current | 28.3828125 (`3633/128`) | 5.259765625 (`2693/512`) | 0 | 62.5 us |
| Iq current | 28.3828125 (`3633/128`) | 5.259765625 (`2693/512`) | 0 | 62.5 us |
| Speed | 1.046875 (`2144/2048`) | 0.00030517578125 (`5/16384`) | 0 | 1 ms |

The speed-loop output remains limited to +/-0.80 A. The 16 kHz board profile
uses BC two-shunt sampling, a 20 V bus, seven pole pairs, Rs=2.55 ohm,
Ld=Lq=0.86 mH, and flux linkage 0.02246532 Wb.

## Integration

1. Follow [the CubeMX setup](CubeMX/STM32G431RBT6_SguanSensorlessPID.md).
2. Add every C file in `Library` and
   `Port/Src/sguan_sensorless_pid_stm32g431_port.c`. Add `Library` and
   `Port/Inc` to the compiler include paths.
3. Merge [the application example](Example/app_sguan_sensorless_pid.c) and
   the JEOS section from
   [the interrupt example](Example/stm32g4xx_it_sguan_sensorless_pid.c).

Compile this folder as an alternative to `Sensorless` or the encoder version;
do not link two SguanFOC library copies into one firmware image.

The application example requests a 0.50 A IF start current and 140 rad/s
mechanical speed. The requested speed must cross the 130 rad/s SMO start
threshold before observer takeover. PID gains can be changed before or after
startup with:

```c
SguanSensorlessPID_Port_SetSpeedPid(1.046875f,
                                    0.00030517578125f,
                                    0.0f);
```

The port preserves requested gains across the library's `Transfer_Init()`,
which otherwise reloads compile-time defaults. Applying gains while the motor
is initialized also reinitializes the speed PID state, including its integral.

## SMO and PLL

The SMO estimates alpha/beta back-EMF from commanded voltage and measured
current. The PLL then uses that back-EMF to track rotor electrical phase and
speed. This profile does use `PLL_encoder`: `Transfer_SMO_Loop()` calls
`PLL_Loop()`, and the resulting `OutRe`/`OutWe` values become the sensorless
angle and speed feedback. Default PLL gains are Kp=650 and Ki=210000.

## First commissioning

Use a current-limited supply and an unloaded shaft. Verify current offsets,
phase order, TIM1 CH4 sampling position, and the IF-to-SMO transition before
increasing speed or PID gains. Current signs are controlled by
`SGUAN_SENSORLESS_PID_PHASE_V_DIR` and
`SGUAN_SENSORLESS_PID_PHASE_W_DIR` in the port header.

Bus-voltage and temperature callbacks still return the upstream sentinel, so
TIM1 BKIN remains the immediate protection path until those board conversion
functions are calibrated and implemented.
