# STM32G431RBT6 CubeMX configuration

This port targets the X-STAR-S FOC/BLDC board documented in this directory.
Generate the HAL project first, then add `Library/*.c`, `Port/Src/*.c` and the
two include directories `Library` and `Port/Inc`.

| Function | MCU resource | Board pins | Required setting |
|---|---|---|---|
| Three-phase PWM | TIM1 CH1/2/3 + CH1N/2N/3N | PA8/PA9/PA10 + PB13/PA12/PB15 | center-aligned, 16 kHz, complementary outputs, 1 us dead time |
| Current sampling | ADC2 JDR1 / ADC1 JDR1 | PC4 phase-V / PB14 phase-W | injected, external trigger TIM1 CH4, 12-bit **right aligned** |
| Sampling trigger | TIM1 CH4 | no output | trigger near the centre of the PWM low-side window |
| MT6701 ABI | TIM3 encoder TI12 | PC6=A, PC7=B | period `SGUANFOC_MT6701_ABI_CPR - 1`, both input filters enabled |
| Hardware brake | TIM1 BKIN | PC13 | active-low external break, keep enabled |
| Optional telemetry | USART2 | PB3=TX, PB4=RX | use the upstream JustFloat protocol if required |

ADC mapping is deliberately BC: `ADC2_IN5` is V phase and `ADC1_IN5` is W
phase. `SguanFOC` derives U from `-(V + W)`. Do not enable left data alignment:
the upstream current scaling expects samples in the 0–4095 range. The board
configuration uses gain 10 and 3 mΩ shunts; both constants are in
`Library/UserData_Motor.h`.

## MT6701 setup

The default macros assume MT6701 is configured for 1024 PPR ABI. With TIM3
x4 decoding that is 4096 counts per mechanical revolution. If the encoder has
been programmed to another PPR, change only `SGUANFOC_MT6701_ABI_PPR` in
`Port/Inc/sguan_stm32g431_port.h`, and set the TIM3 period to `CPR - 1`.

The port aligns the encoder electrical zero during start-up, so an ABI-only
MT6701 does not require a Z index. If the shaft direction is reversed, change
`SGUANFOC_ENCODER_DIR`; if the measured phase current signs are reversed,
change the two `SGUANFOC_PHASE_*_DIR` macros. Make one change at a time and
keep the rotor unloaded for the first alignment.
