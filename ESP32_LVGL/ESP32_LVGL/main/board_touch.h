#ifndef BOARD_TOUCH_H
#define BOARD_TOUCH_H

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "lvgl.h"

/* CHSC5432 correction after the vendor horizontal conversion. */
#define BOARD_TOUCH_SWAP_XY       0
#define BOARD_TOUCH_MIRROR_X      0
#define BOARD_TOUCH_MIRROR_Y      0

typedef esp_err_t (*board_touch_reset_cb_t)(bool asserted);

/**
 * @brief 复位、探测并将板载触摸控制器注册为 LVGL 输入设备。
 * @note 必须在持有 LVGL port 锁时调用。
 */
esp_err_t board_touch_init(
    i2c_master_bus_handle_t i2c_bus,
    lv_display_t *display,
    board_touch_reset_cb_t reset_callback);

/** @brief 获取 board_touch_init() 创建的指针输入设备。 */
lv_indev_t *board_touch_get_indev(void);

#endif /* BOARD_TOUCH_H */
