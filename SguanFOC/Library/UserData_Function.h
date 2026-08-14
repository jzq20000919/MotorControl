#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口 */
/* X-STAR-S / STM32G431RBT6 hardware adaptation. */
#include "sguan_stm32g431_port.h"

static inline void User_InitialInit(void){
    SguanFOC_Port_Init();
}

static inline void User_Delay(unsigned int ms){
    SguanFOC_Port_DelayMs(ms);
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    return SguanFOC_Port_ReadPhaseCurrentRaw(Current_CH);
}

static inline float User_Encoder_ReadRad(void){
    return SguanFOC_Port_ReadEncoderRad();
}

static inline void User_PwmDuty_Set(unsigned short int Duty_u,
                                unsigned short int Duty_v,
                                unsigned short int Duty_w){
    SguanFOC_Port_SetPwmDuty(Duty_u, Duty_v, Duty_w);
}

static inline float User_VBUS_DataGet(void){
    // float VBUS_num = 0.0f;
    /* Your code for motor VBUS_Voltage Data return if you use it */
    
    // 如果不使用电压功能，返回-9999.0f（正常电压不会是负数）
    return -9999.0f;
}

static inline float User_Temperature_DataGet(void){
    // float Temp_num = 0.0f;
    /* Your code for motor Temperature Data return if you use it */
    
    // 如果不使用温度功能，返回-9999.0f（正常温度不会是这么大的负数）
    return -9999.0f;
}


#endif // USERDATA_FUNCTION_H
