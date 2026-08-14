#ifndef MOTOR_LINK_H
#define MOTOR_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** @brief 当前拥有电机控制权的物理传输通道。 */
typedef enum
{
    MOTOR_LINK_NONE = 0, /**< 尚未选择控制通道。 */
    MOTOR_LINK_UART,     /**< 使用 UART 控制通道。 */
    MOTOR_LINK_CAN       /**< 使用 CAN 控制通道。 */
} motor_link_transport_t;

/** @brief 与底层传输协议无关的电机控制模式。 */
typedef enum
{
    MOTOR_LINK_MODE_SPEED = 0,   /**< 速度闭环控制模式。 */
    MOTOR_LINK_MODE_POSITION = 1 /**< 位置闭环控制模式。 */
} motor_link_mode_t;

/** @brief 汇总 UART/CAN 状态的统一电机遥测快照。 */
typedef struct
{
    motor_link_transport_t transport; /**< 当前拥有控制权的传输通道。 */
    bool uart_connected;              /**< UART 驱动已初始化。 */
    bool uart_link_active;            /**< UART 最近收到有效遥测。 */
    bool can_connected;               /**< CAN 驱动已初始化。 */
    bool can_link_active;             /**< CAN 最近收到有效遥测。 */
    bool link_active;                 /**< 当前活动通道在线。 */
    bool reconnecting;                /**< 当前活动 UART 正在重连。 */
    bool bus_off;                     /**< 当前 CAN 控制器处于 Bus-Off。 */
    bool transceiver_fault;           /**< CAN 收发器自检失败。 */
    bool motor_running;               /**< 电机当前处于运行状态。 */
    bool motor_fault;                 /**< 电机当前存在故障。 */
    bool command_rejected;            /**< 电机板拒绝了最近一条命令。 */
    motor_link_mode_t mode;           /**< 当前速度或位置控制模式。 */
    uint16_t faults;                  /**< 电机故障位集合。 */
    int16_t measured_speed_rpm;       /**< 实测机械转速，单位 rpm。 */
    int16_t reference_speed_rpm;      /**< 机械转速参考值，单位 rpm。 */
    uint16_t current_position_cdeg;   /**< 当前单圈位置，单位 0.01°。 */
    uint16_t target_position_cdeg;    /**< 目标单圈位置，单位 0.01°。 */
    int16_t position_error_cdeg;      /**< 位置误差，单位 0.01°。 */
    int16_t iq_ma;                    /**< q 轴实测电流，单位 mA。 */
    int16_t id_ma;                    /**< d 轴实测电流，单位 mA。 */
    int16_t iq_reference_ma;          /**< q 轴电流参考值，单位 mA。 */
    int16_t id_reference_ma;          /**< d 轴电流参考值，单位 mA。 */
    int16_t uq_mv;                    /**< q 轴电压指令，单位 mV。 */
    int16_t ud_mv;                    /**< d 轴电压指令，单位 mV。 */
    uint32_t received_frames;         /**< 活动通道累计有效接收帧数。 */
    uint32_t transmitted_frames;      /**< 活动通道累计成功发送帧数。 */
    uint32_t transmit_errors;         /**< 活动通道累计发送错误次数。 */
    uint32_t baud_rate;               /**< UART 当前波特率；CAN 模式下为零。 */
} motor_link_snapshot_t;

/** @brief 清除当前传输通道选择；使用本模块前调用一次。 */
void motor_link_init(void);
/** @brief 启用 UART 控制，同时关闭 CAN 控制权。 @param baud_rate UART 波特率。 @return 成功返回 ESP_OK。 */
esp_err_t motor_link_connect_uart(uint32_t baud_rate);
/** @brief 启用 CAN 控制，同时关闭 UART 控制权。 @return 成功返回 ESP_OK。 */
esp_err_t motor_link_connect_can(void);
/** @brief 停止 UART 命令输出并释放 UART 驱动资源。 */
void motor_link_disconnect_uart(void);
/** @brief 停止 CAN 命令输出并释放 CAN 驱动资源。 */
void motor_link_disconnect_can(void);
/** @brief 复制当前活动通道的最新遥测数据。 @param[out] snapshot 接收统一快照的对象。 */
void motor_link_get_snapshot(motor_link_snapshot_t *snapshot);
/** @brief 请求活动通道切换至速度或位置控制模式。 @param mode 目标控制模式。 */
void motor_link_set_mode(motor_link_mode_t mode);
/** @brief 向活动通道请求转速目标。 @param speed_rpm 目标转速，单位 rpm。 */
void motor_link_set_speed_rpm(int16_t speed_rpm);
/** @brief 向活动通道请求位置目标。 @param position_cdeg 目标位置，单位 0.01°。 */
void motor_link_set_position_cdeg(uint16_t position_cdeg);
/** @brief 请求活动通道启动电机。 */
void motor_link_start_motor(void);
/** @brief 请求活动通道停止电机。 */
void motor_link_stop_motor(void);
/** @brief 请求活动通道确认/复位电机故障。 */
void motor_link_acknowledge_fault(void);
/** @brief 请求活动通道将当前位置设为零点。 */
void motor_link_zero_position(void);

#endif /* MOTOR_LINK_H */
