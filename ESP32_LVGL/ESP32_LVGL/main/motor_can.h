#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_can_protocol.h"

typedef struct
{
    bool link_active;
    bool bus_off;
    bool transceiver_fault;
    bool motor_running;
    bool motor_fault;
    bool command_rejected;
    MotorCan_Mode_t mode;
    uint16_t faults;
    int16_t measured_speed_rpm;
    int16_t reference_speed_rpm;
    uint16_t current_position_cdeg;
    uint16_t target_position_cdeg;
    int16_t position_error_cdeg;
    int16_t iq_ma;
    int16_t id_ma;
    int16_t iq_reference_ma;
    int16_t id_reference_ma;
    uint32_t received_frames;
    uint32_t transmitted_frames;
    uint32_t transmit_errors;
} motor_can_snapshot_t;

/** @brief 安装并启动 ESP32 TWAI 控制器及 CAN RX/TX 任务。 */
esp_err_t motor_can_init(void);
/** @brief 停止 CAN 任务、卸载 TWAI 并释放队列。 */
void motor_can_deinit(void);
/** @brief 返回 CAN 传输通道是否已完整初始化。 */
bool motor_can_is_initialized(void);
/** @brief 不等待新帧，直接复制最新已解码 CAN 遥测。 */
void motor_can_get_snapshot(motor_can_snapshot_t *snapshot);
/** @brief 授予或撤销该传输通道发送电机命令的控制权。 */
void motor_can_set_control_enabled(bool enabled);

/** @brief 排队一个 CAN 控制模式命令。 */
void motor_can_set_mode(MotorCan_Mode_t mode);
/** @brief 保存最新速度目标，供 CAN 周期发送，单位 rpm。 */
void motor_can_set_speed_rpm(int16_t speed_rpm);
/** @brief 保存最新位置目标，供 CAN 周期发送，单位 0.01°。 */
void motor_can_set_position_cdeg(uint16_t position_cdeg);
/** @brief 排队一个 CAN 电机启动命令。 */
void motor_can_start_motor(void);
/** @brief 排队一个 CAN 电机停止命令。 */
void motor_can_stop_motor(void);
/** @brief 排队一个 CAN 故障确认命令。 */
void motor_can_acknowledge_fault(void);
/** @brief 排队一个 CAN 位置清零命令。 */
void motor_can_zero_position(void);

#endif /* MOTOR_CAN_H */
