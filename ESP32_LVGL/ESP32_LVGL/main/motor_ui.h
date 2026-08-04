#ifndef MOTOR_UI_H
#define MOTOR_UI_H

#include "lvgl.h"

/**
 * @brief 创建全部应用页面、定时器及与显示器绑定的 LVGL 对象。
 * @note 调用者必须持有 LVGL port 锁。
 */
void motor_ui_create(lv_display_t *display);
/** @brief 在 UI 创建完成后绑定指针或编码器输入设备。 */
void motor_ui_attach_input(lv_indev_t *indev);

#endif /* MOTOR_UI_H */
