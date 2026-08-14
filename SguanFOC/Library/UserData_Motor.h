#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
#include "sguan_stm32g431_port.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码) */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 1.mode选择电机的运行模式
    Sguan.mode = Current_SINGLE_MODE;
    // 如果你要在电机启动后主动切换模式，这个地方请不要使用
    // 它会在每次启动时，刷新你的更改值
    // 2.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10; // (uint8_t)PWM错误限幅
    // 3.identify电机参数辨识结果(根据实际电机参数填写，或者通过辨识算法得到)
    /* 2804 motor values from the existing board project; verify on the target motor. */
    Sguan.identify.Ld = 0.00086f;
    Sguan.identify.Lq = 0.00086f;
    Sguan.identify.Ls = 0.00086f;
    Sguan.identify.Rs = 2.55f;
    Sguan.identify.Flux = 0.02246532f;
    // 4.motor电机参数辨识
    Sguan.motor.Poles = 7;              // (uint8_t)极对极数
    Sguan.motor.VBUS = 20.0f;           // (float)母线电压

    Sguan.motor.Motor_Dir = 1;          // (int8_t)电机方向1->正向，负1->负向
    Sguan.motor.PWM_Dir = -1;           // (int8_t)PWM占空比高低对应1->正向，负1->负向
    Sguan.motor.Duty = SGUANFOC_TIM1_ARR; // TIM1 ARR, generated for 16 kHz center-aligned PWM

    Sguan.motor.Encoder_Dir = SGUANFOC_ENCODER_DIR;

    Sguan.motor.Current_Dir0 = SGUANFOC_PHASE_V_DIR;
    Sguan.motor.Current_Dir1 = SGUANFOC_PHASE_W_DIR;
    Sguan.motor.Current_Num = 2;        // BC sampling: CH0=V, CH1=W; library derives U
    Sguan.motor.ADC_Precision = 4096;   // (uint32_t)ADC采样精度
    Sguan.motor.Amplifier = 10.0f;      // (float)运算放大器增益
    Sguan.motor.MCU_Voltage = 3.3f;     // (float)DSP/单片机的ADC电压基准
    Sguan.motor.Sampling_Rs = 0.003f;   // Board shunt value from MCSDK_FOC_MIX
    // 5.电机安全设计
    Sguan.safe.VBUS_MAX = 28.0f;        // Enable only after implementing User_VBUS_DataGet
    Sguan.safe.VBUS_MIM = 8.0f;
    Sguan.safe.VBUS_watchdog_limit = 1000;

    Sguan.safe.Temp_MAX = 60.0f;        // (float)驱动器允许最大温度
    Sguan.safe.Temp_MIN = -20.0f;       // (float)驱动器允许最小温度
    Sguan.safe.Temp_watchdog_limit = 1000;

    Sguan.safe.Dcur_MAX = 2.0f;         // Board project rated current is 2 A
    Sguan.safe.Qcur_MAX = 2.0f;
    Sguan.safe.DQcur_watchdog_limit = 1000;

    Sguan.safe.DISABLED_watchdog_limit = 1000;
    // 6.系统定时中断周期设计
    Sguan.PMSM_RUN_T = (1.0f / 16000.0f); // TIM1/ADC injected-control rate
}   


#endif // USERDATA_MOTOR_H
