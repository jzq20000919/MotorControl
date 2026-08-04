#include "board_touch.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOUCH_I2C_ADDRESS        0x2EU
#define TOUCH_ID_ADDRESS         0x20000080UL
#define TOUCH_EVENT_ADDRESS      0x2000002CUL
#define TOUCH_EVENT_DATA_SIZE    28U
#define TOUCH_I2C_FREQUENCY_HZ   400000U
#define TOUCH_DISPLAY_H_RES      320U
#define TOUCH_DISPLAY_V_RES      240U

static const char *TAG = "BOARD_TOUCH";
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_touch_device;
static board_touch_reset_cb_t s_reset_callback;
static bool s_address_little_endian = true;
static lv_point_t s_last_point;
static lv_indev_t *s_touch_indev;

/**
 * @brief 为 I2C 事务编码 32 位 CHSC5432 寄存器地址。
 *
 * Different controller revisions expose the same register map with different
 * address byte order.  The order detected during initialisation is retained
 * and used by all following reads.
 *
 * @param address Register address to encode.
 * @param little_endian True for least-significant byte first.
 * @param[out] bytes Four-byte destination buffer.
 */
static void touch_make_address(
    uint32_t address,
    bool little_endian,
    uint8_t bytes[4])
{
    if (little_endian) {
        bytes[0] = (uint8_t)(address >> 0U);
        bytes[1] = (uint8_t)(address >> 8U);
        bytes[2] = (uint8_t)(address >> 16U);
        bytes[3] = (uint8_t)(address >> 24U);
    } else {
        bytes[0] = (uint8_t)(address >> 24U);
        bytes[1] = (uint8_t)(address >> 16U);
        bytes[2] = (uint8_t)(address >> 8U);
        bytes[3] = (uint8_t)(address >> 0U);
    }
}

/**
 * @brief 使用指定地址字节序读取触摸控制器寄存器数据。
 *
 * @param address CHSC5432 register address.
 * @param little_endian Register-address byte order to test/use.
 * @param[out] data Destination buffer for received bytes.
 * @param length Number of bytes to read.
 * @return ESP_OK when the combined I2C transaction succeeds.
 */
static esp_err_t touch_direct_read_order(
    uint32_t address,
    bool little_endian,
    uint8_t *data,
    size_t length)
{
    uint8_t address_bytes[4];
    touch_make_address(address, little_endian, address_bytes);
    return i2c_master_transmit_receive(
        s_touch_device,
        address_bytes,
        sizeof(address_bytes),
        data,
        length,
        20);
}

/**
 * @brief 使用启动时选定的字节序读取 CHSC5432 寄存器。
 * @param address Register address.
 * @param[out] data Destination buffer.
 * @param length Number of requested bytes.
 * @return Result returned by the I2C master driver.
 */
static esp_err_t touch_direct_read(
    uint32_t address,
    uint8_t *data,
    size_t length)
{
    return touch_direct_read_order(
        address, s_address_little_endian, data, length);
}

/**
 * @brief 检查候选控制器 ID 是否为无效总线返回值。
 *
 * An all-zero or all-0xFF response usually indicates an unresponsive device
 * or an invalid register-address order, rather than a useful chip ID.
 *
 * @param id Bytes read from the ID register.
 * @param length Number of bytes in @p id.
 * @return True when the value is plausible.
 */
static bool touch_id_is_valid(const uint8_t *id, size_t length)
{
    bool all_zero = true;
    bool all_ff = true;

    for (size_t i = 0; i < length; i++) {
        all_zero = all_zero && (id[i] == 0x00U);
        all_ff = all_ff && (id[i] == 0xFFU);
    }
    return !all_zero && !all_ff;
}

/**
 * @brief 将原始触摸坐标转换到 LVGL 显示坐标系。
 *
 * The compile-time swap/mirror options must match the LCD orientation set in
 * main.c.  The final clamp protects LVGL from malformed points outside the
 * 320 x 240 display.
 *
 * @param[in,out] x Horizontal coordinate.
 * @param[in,out] y Vertical coordinate.
 */
static void touch_transform(uint16_t *x, uint16_t *y)
{
    uint16_t transformed_x = *x;
    uint16_t transformed_y = *y;

#if BOARD_TOUCH_SWAP_XY
    const uint16_t temporary = transformed_x;
    transformed_x = transformed_y;
    transformed_y = temporary;
#endif

#if BOARD_TOUCH_MIRROR_X
    transformed_x =
        (uint16_t)(TOUCH_DISPLAY_H_RES - 1U - transformed_x);
#endif

#if BOARD_TOUCH_MIRROR_Y
    transformed_y =
        (uint16_t)(TOUCH_DISPLAY_V_RES - 1U - transformed_y);
#endif

    if (transformed_x >= TOUCH_DISPLAY_H_RES) {
        transformed_x = TOUCH_DISPLAY_H_RES - 1U;
    }
    if (transformed_y >= TOUCH_DISPLAY_V_RES) {
        transformed_y = TOUCH_DISPLAY_V_RES - 1U;
    }

    *x = transformed_x;
    *y = transformed_y;
}

/**
 * @brief 读取并解码第一个有效的 CHSC5432 触摸点。
 *
 * The controller event report may contain multiple contacts; this UI uses the
 * first valid contact because LVGL is registered as a single-pointer device.
 * A failed I2C transaction, no contact, or an out-of-range point is reported
 * as "not pressed" to the caller.
 *
 * @param[out] x Decoded, transformed X coordinate.
 * @param[out] y Decoded, transformed Y coordinate.
 * @return True when a valid pressed point was obtained.
 */
static bool touch_read_point(uint16_t *x, uint16_t *y)
{
    uint8_t buffer[TOUCH_EVENT_DATA_SIZE] = {0U};
    if (touch_direct_read(
            TOUCH_EVENT_ADDRESS,
            buffer,
            sizeof(buffer)) != ESP_OK) {
        return false;
    }

    const uint8_t touch_count = buffer[1] & 0x0FU;
    if (touch_count == 0U || touch_count > 5U) {
        return false;
    }

    /*
     * CHSC5432 vendor conversion for the DNESP32S3B horizontal
     * 320x240 display.
     */
    uint16_t point_x =
        ((uint16_t)(buffer[5] >> 4U) << 8U) | buffer[3];
    const uint16_t raw_y =
        ((uint16_t)(buffer[5] & 0x0FU) << 8U) | buffer[2];
    uint16_t point_y =
        raw_y >= TOUCH_DISPLAY_V_RES
            ? 0U
            : (uint16_t)(TOUCH_DISPLAY_V_RES - raw_y);

    if (point_x >= TOUCH_DISPLAY_H_RES ||
        point_y >= TOUCH_DISPLAY_V_RES) {
        return false;
    }

    touch_transform(&point_x, &point_y);
    *x = point_x;
    *y = point_y;
    return true;
}

/**
 * @brief 向 LVGL 输入子系统提供最新触摸状态。
 *
 * This callback is invoked by LVGL's input-read timer, not by application
 * code.  Keeping the last valid point on release follows LVGL's pointer-input
 * convention and lets widgets receive a consistent press/release sequence.
 *
 * @param indev LVGL input device requesting data; unused by this driver.
 * @param[out] data LVGL structure to receive coordinate and pressed state.
 */
static void touch_read_callback(
    lv_indev_t *indev,
    lv_indev_data_t *data)
{
    uint16_t x;
    uint16_t y;
    (void)indev;

    if (touch_read_point(&x, &y)) {
        s_last_point.x = (lv_coord_t)x;
        s_last_point.y = (lv_coord_t)y;
        data->point = s_last_point;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->point = s_last_point;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/**
 * @brief 初始化 CHSC5432，并将其注册为 LVGL 指针输入设备。
 *
 * The caller must already have created the shared I2C master bus and hold the
 * LVGL port lock.  The routine resets and probes the controller, detects its
 * register-address byte order, then creates an LVGL input device polled at
 * 100 Hz.
 *
 * @param i2c_bus Existing I2C master bus used by the board peripherals.
 * @param display LVGL display associated with the touch coordinates.
 * @param reset_callback Board-specific callback that drives touch reset.
 * @return ESP_OK when the controller and LVGL input device are ready.
 */
esp_err_t board_touch_init(
    i2c_master_bus_handle_t i2c_bus,
    lv_display_t *display,
    board_touch_reset_cb_t reset_callback)
{
    uint8_t id[4] = {0U};
    esp_err_t result;

    if (i2c_bus == NULL ||
        display == NULL ||
        reset_callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_bus = i2c_bus;
    s_reset_callback = reset_callback;

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_I2C_ADDRESS,
        .scl_speed_hz = TOUCH_I2C_FREQUENCY_HZ,
    };
    result = i2c_master_bus_add_device(
        s_i2c_bus, &device_config, &s_touch_device);
    if (result != ESP_OK) {
        return result;
    }

    result = s_reset_callback(true);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    result = s_reset_callback(false);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(80));

    result = i2c_master_probe(
        s_i2c_bus, TOUCH_I2C_ADDRESS, 1000);
    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "CHSC5432 did not respond at I2C address 0x%02X",
            TOUCH_I2C_ADDRESS);
        return result;
    }

    result = touch_direct_read_order(
        TOUCH_ID_ADDRESS, true, id, sizeof(id));
    if (result == ESP_OK && touch_id_is_valid(id, sizeof(id))) {
        s_address_little_endian = true;
    } else {
        (void)memset(id, 0, sizeof(id));
        result = touch_direct_read_order(
            TOUCH_ID_ADDRESS, false, id, sizeof(id));
        if (result != ESP_OK) {
            return result;
        }
        s_address_little_endian = false;
    }

    ESP_LOGI(
        TAG,
        "CHSC5432 ID %02X %02X %02X %02X, %s-endian address",
        id[0],
        id[1],
        id[2],
        id[3],
        s_address_little_endian ? "little" : "big");

    s_touch_indev = lv_indev_create();
    if (s_touch_indev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_touch_indev, touch_read_callback);
    lv_indev_set_display(s_touch_indev, display);

    lv_timer_t *read_timer =
        lv_indev_get_read_timer(s_touch_indev);
    if (read_timer != NULL) {
        lv_timer_set_period(read_timer, 10U);
    }

    ESP_LOGI(TAG, "CHSC5432 LVGL input registered at 100 Hz");
    return ESP_OK;
}

/**
 * @brief 返回 board_touch_init() 创建的 LVGL 输入设备。
 * @return Pointer input device, or NULL before successful initialisation.
 */
lv_indev_t *board_touch_get_indev(void)
{
    return s_touch_indev;
}
