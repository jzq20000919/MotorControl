#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "board_keys.h"
#include "board_touch.h"
#include "motor_link.h"
#include "motor_ui.h"
#include "mqtt_manager.h"
#include "mqtt_motor_gateway.h"
#include "wifi_manager.h"

/* ============================================================
 * 基本配置
 * ============================================================ */

static const char *TAG = "MOTOR_HMI";

/*
 * 横屏分辨率。
 * ST7789 物理面板为 240 × 320，旋转后作为 320 × 240 使用。
 */
#define LCD_H_RES              320
#define LCD_V_RES              240

/*
 * LVGL 使用单个整屏 RGB565 绘图缓冲区：
 * 320 × 240 × 2 = 153.6 KB。
 * 整屏刷新用于避免页面切换时的残影。
 */
#define LCD_DRAW_BUF_LINES     LCD_V_RES

#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)

/* ============================================================
 * LCD GPIO
 * ============================================================ */

#define LCD_GPIO_CS            GPIO_NUM_1
#define LCD_GPIO_DC            GPIO_NUM_2
#define LCD_GPIO_RD            GPIO_NUM_41
#define LCD_GPIO_WR            GPIO_NUM_42

#define LCD_GPIO_D0            GPIO_NUM_40
#define LCD_GPIO_D1            GPIO_NUM_39
#define LCD_GPIO_D2            GPIO_NUM_38
#define LCD_GPIO_D3            GPIO_NUM_12
#define LCD_GPIO_D4            GPIO_NUM_11
#define LCD_GPIO_D5            GPIO_NUM_10
#define LCD_GPIO_D6            GPIO_NUM_9
#define LCD_GPIO_D7            GPIO_NUM_46

/*
 * LCD 复位信号与开发板复位连接在一起，
 * 没有独立 GPIO。
 */
#define LCD_GPIO_RST           GPIO_NUM_NC

/*
 * 横屏方向参数。
 * 若画面方向不正确，只调整这三个宏。
 */
#define LCD_SWAP_XY            true
#define LCD_MIRROR_X           true
#define LCD_MIRROR_Y           false

/* ============================================================
 * XL9555 / I2C
 * ============================================================ */

#define BOARD_I2C_PORT         I2C_NUM_0
#define BOARD_I2C_SDA          GPIO_NUM_48
#define BOARD_I2C_SCL          GPIO_NUM_45
#define BOARD_I2C_FREQ_HZ      400000

#define XL9555_I2C_ADDR        0x20

#define XL9555_INPUT_PORT0     0x00
#define XL9555_OUTPUT_PORT0    0x02
#define XL9555_CONFIG_PORT0    0x06

/* LCD 背光连接 XL9555 P0_7 */
#define XL9555_LCD_BL_MASK     (1U << 7)
#define XL9555_TOUCH_RST_MASK  (1U << 6)
#define XL9555_KEY0_MASK       (1U << 4) /* P0.4, active low */
#define XL9555_KEY1_MASK       (1U << 3) /* P0.3, active low */
#define BOARD_KEY0_GPIO        GPIO_NUM_0 /* BOOT key, active low */

/* ============================================================
 * 全局句柄
 * ============================================================ */

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_xl9555;
static esp_lcd_i80_bus_handle_t s_lcd_bus;
static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_lcd_panel;
static lv_display_t *s_lvgl_display;

/* ============================================================
 * XL9555 驱动
 * ============================================================ */

/**
 * @brief 从 XL9555 连续读取两个寄存器
 */
/**
 * @brief 在一次 I2C 事务中读取两个连续的 XL9555 寄存器。
 * @param start_register 第一个寄存器地址。
 * @param[out] values 接收两个寄存器值的缓冲区。
 * @return ESP-IDF I2C 事务的执行结果。
 */
static esp_err_t xl9555_read_pair(uint8_t start_register,
                                  uint8_t values[2])
{
    return i2c_master_transmit_receive(
        s_xl9555,
        &start_register,
        sizeof(start_register),
        values,
        2,
        10
    );
}

/**
 * @brief 向 XL9555 连续写入两个寄存器
 */
/**
 * @brief 在一次 I2C 事务中写入两个连续的 XL9555 寄存器。
 * @param start_register 第一个寄存器地址。
 * @param values 对应 @p start_register 及后继寄存器的值。
 * @return ESP-IDF I2C 事务的执行结果。
 */
static esp_err_t xl9555_write_pair(uint8_t start_register,
                                   const uint8_t values[2])
{
    const uint8_t tx_buffer[3] = {
        start_register,
        values[0],
        values[1],
    };

    return i2c_master_transmit(
        s_xl9555,
        tx_buffer,
        sizeof(tx_buffer),
        1000
    );
}

/**
 * @brief 初始化板载 I2C 和 XL9555
 */
/**
 * @brief 初始化共享 I2C 总线和 XL9555 板载 I/O 扩展器。
 *
 * 扩展器负责 LCD 背光、触摸复位及两个物理按键。在改变输出方向前会先关闭
 * 背光，避免上电过程中出现可见闪屏。
 */
static void board_xl9555_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus");

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BOARD_I2C_PORT,
        .scl_io_num = BOARD_I2C_SCL,
        .sda_io_num = BOARD_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &s_i2c_bus)
    );

    ESP_LOGI(TAG, "Adding XL9555 device");

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XL9555_I2C_ADDR,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            s_i2c_bus,
            &device_config,
            &s_xl9555
        )
    );

    /*
     * 检查 XL9555 是否真正应答。
     * 如果接线或地址错误，程序会在这里报错。
     */
    ESP_ERROR_CHECK(
        i2c_master_probe(
            s_i2c_bus,
            XL9555_I2C_ADDR,
            1000
        )
    );

    /*
     * 先把背光输出值设置为低，防止配置方向时闪屏。
     */
    uint8_t output_values[2];

    ESP_ERROR_CHECK(
        xl9555_read_pair(
            XL9555_OUTPUT_PORT0,
            output_values
        )
    );

    output_values[0] &= (uint8_t)~XL9555_LCD_BL_MASK;
    output_values[0] |= XL9555_TOUCH_RST_MASK;

    ESP_ERROR_CHECK(
        xl9555_write_pair(
            XL9555_OUTPUT_PORT0,
            output_values
        )
    );

    /*
     * XL9555 配置寄存器：
     * 1 = 输入
     * 0 = 输出
     *
     * 将 P0_7 设置为输出，不改变其他引脚配置。
     */
    uint8_t config_values[2];

    ESP_ERROR_CHECK(
        xl9555_read_pair(
            XL9555_CONFIG_PORT0,
            config_values
        )
    );

    config_values[0] &=
        (uint8_t)~(XL9555_LCD_BL_MASK | XL9555_TOUCH_RST_MASK);

    ESP_ERROR_CHECK(
        xl9555_write_pair(
            XL9555_CONFIG_PORT0,
            config_values
        )
    );

    ESP_LOGI(TAG, "XL9555 initialized");
}

/**
 * @brief 控制 LCD 背光
 */
/**
 * @brief 通过 XL9555 P0.7 打开或关闭 LCD 背光。
 * @param enabled 为 true 时点亮屏幕，为 false 时熄灭背光。
 * @note 采用读-改-写方式，避免影响 XL9555 上的其它输出引脚。
 */
static void board_lcd_backlight_set(bool enabled)
{
    uint8_t output_values[2];

    ESP_ERROR_CHECK(
        xl9555_read_pair(
            XL9555_OUTPUT_PORT0,
            output_values
        )
    );

    if (enabled) {
        output_values[0] |= XL9555_LCD_BL_MASK;
    } else {
        output_values[0] &= (uint8_t)~XL9555_LCD_BL_MASK;
    }

    ESP_ERROR_CHECK(
        xl9555_write_pair(
            XL9555_OUTPUT_PORT0,
            output_values
        )
    );
}

/**
 * @brief 通过 XL9555 P0.6 驱动低有效的 CHSC5432 复位线。
 * @param asserted true 表示保持触摸芯片复位，false 表示释放复位。
 * @return I2C 读写结果，供 board_touch.c 上报初始化失败。
 */
static esp_err_t board_touch_reset_set(bool asserted)
{
    uint8_t output_values[2];
    esp_err_t result = xl9555_read_pair(
        XL9555_OUTPUT_PORT0, output_values);
    if (result != ESP_OK) {
        return result;
    }

    if (asserted) {
        output_values[0] &= (uint8_t)~XL9555_TOUCH_RST_MASK;
    } else {
        output_values[0] |= XL9555_TOUCH_RST_MASK;
    }

    return xl9555_write_pair(
        XL9555_OUTPUT_PORT0, output_values);
}

/**
 * @brief 将两个扩展器按键和 ESP32 BOOT 按键配置为输入。
 *
 * 软件消抖由 UI 层完成；本函数只配置电气方向和 BOOT 按键的内部上拉。
 */
void board_keys_init(void)
{
    uint8_t config_values[2];
    ESP_ERROR_CHECK(
        xl9555_read_pair(XL9555_CONFIG_PORT0, config_values));
    config_values[0] |=
        XL9555_KEY0_MASK | XL9555_KEY1_MASK;
    ESP_ERROR_CHECK(
        xl9555_write_pair(XL9555_CONFIG_PORT0, config_values));

    const gpio_config_t boot_key_config = {
        .pin_bit_mask = 1ULL << BOARD_KEY0_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_key_config));
}

/**
 * @brief 采样全部物理按键并返回逻辑位掩码。
 *
 * 板上所有按键均为低电平有效。XL9555 读取失败时，其两个按键位保持清零，
 * 但 BOOT 按键仍可正常读取。
 * @return BOARD_KEY_K0、BOARD_KEY_K1、BOARD_KEY_K2 的按位或。
 */
uint8_t board_keys_read(void)
{
    uint8_t input_values[2];
    uint8_t keys = 0U;

    if (xl9555_read_pair(
            XL9555_INPUT_PORT0, input_values) == ESP_OK) {
        /* Physical KEY0 and KEY1 act as logical K1 and K2. */
        if ((input_values[0] & XL9555_KEY0_MASK) == 0U) {
            keys |= BOARD_KEY_K1;
        }
        if ((input_values[0] & XL9555_KEY1_MASK) == 0U) {
            keys |= BOARD_KEY_K2;
        }
    }
    if (gpio_get_level(BOARD_KEY0_GPIO) == 0) {
        keys |= BOARD_KEY_K0;
    }
    return keys;
}

/* ============================================================
 * LCD 驱动
 * ============================================================ */

/**
 * @brief 初始化 ST7789 和 I80 八位并口
 */
/**
 * @brief 在 8 位 I80 并口上创建并初始化 ST7789 面板。
 *
 * 本函数完成面向硬件的显示配置：本设计不读取 LCD 数据，因此 RD 始终保持高
 * 电平；随后创建 ESP-IDF I80 总线、面板 IO 和 ST7789 对象。此处的 RGB565
 * 字节序与屏幕方向必须和 LVGL、触摸坐标变换保持一致。
 */
static void board_lcd_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD RD pin");

    /*
     * 当前程序不读取 LCD 数据，因此 RD 始终保持高电平。
     */
    const gpio_config_t rd_config = {
        .pin_bit_mask = 1ULL << LCD_GPIO_RD,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&rd_config));
    ESP_ERROR_CHECK(gpio_set_level(LCD_GPIO_RD, 1));

    ESP_LOGI(TAG, "Creating I80 LCD bus");

    const esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_GPIO_DC,
        .wr_gpio_num = LCD_GPIO_WR,

        .data_gpio_nums = {
            LCD_GPIO_D0,
            LCD_GPIO_D1,
            LCD_GPIO_D2,
            LCD_GPIO_D3,
            LCD_GPIO_D4,
            LCD_GPIO_D5,
            LCD_GPIO_D6,
            LCD_GPIO_D7,
        },

        .bus_width = 8,

        .max_transfer_bytes =
            LCD_H_RES *
            LCD_DRAW_BUF_LINES *
            sizeof(uint16_t),

        /*
         * ESP-IDF 6.0 使用 dma_burst_size，
         * 不再使用旧版的 sram_trans_align。
         */
        .dma_burst_size = 64,
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_i80_bus(
            &bus_config,
            &s_lcd_bus
        )
    );

    ESP_LOGI(TAG, "Creating I80 panel IO");

    const esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 2,

        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },

        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,

        /*
         * RGB565 在内存中通常为小端字节顺序，
         * I80 屏幕需要高字节先发送。
         */
        .flags = {
            .swap_color_bytes = true,
        },
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_i80(
            s_lcd_bus,
            &io_config,
            &s_lcd_io
        )
    );

    ESP_LOGI(TAG, "Creating ST7789 panel");

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_st7789(
            s_lcd_io,
            &panel_config,
            &s_lcd_panel
        )
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_reset(s_lcd_panel)
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_init(s_lcd_panel)
    );

    /*
     * 正点原子这块 ST7789 屏需要开启颜色反转。
     */
    ESP_ERROR_CHECK(
        esp_lcd_panel_invert_color(
            s_lcd_panel,
            true
        )
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_set_gap(
            s_lcd_panel,
            0,
            0
        )
    );

    /*
     * 设置横屏方向。
     */
    ESP_ERROR_CHECK(
        esp_lcd_panel_swap_xy(
            s_lcd_panel,
            LCD_SWAP_XY
        )
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_mirror(
            s_lcd_panel,
            LCD_MIRROR_X,
            LCD_MIRROR_Y
        )
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_disp_on_off(
            s_lcd_panel,
            true
        )
    );

    ESP_LOGI(TAG, "LCD initialized");
}

/**
 * @brief 初始化 LVGL 并注册 LCD
 */
/**
 * @brief 初始化 esp_lvgl_port，并将 LCD 注册为 LVGL 显示器。
 *
 * 工程使用单个全屏 RGB565 DMA 缓冲区以保证页面切换稳定。port 会在 Core 1
 * 创建 LVGL 服务任务；在该任务以外修改 LVGL 对象前，应用代码必须调用
 * lvgl_port_lock() 获取锁。
 */
static void board_lvgl_init(void)
{
    ESP_LOGI(TAG, "Initializing LVGL port");

    lvgl_port_cfg_t lvgl_config =
        ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_config.task_priority = 8;
    lvgl_config.task_affinity = 1;
    lvgl_config.task_max_sleep_ms = 5;
    lvgl_config.timer_period_ms = 2;

    ESP_ERROR_CHECK(
        lvgl_port_init(&lvgl_config)
    );

    /*
     * buffer_size 的单位是像素，而不是字节。
     */
    const lvgl_port_display_cfg_t display_config = {
        .io_handle = s_lcd_io,
        .panel_handle = s_lcd_panel,

        .buffer_size =
            LCD_H_RES *
            LCD_DRAW_BUF_LINES,

        .double_buffer = false,

        .hres = LCD_H_RES,
        .vres = LCD_V_RES,

        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,

        /*
         * 与前面 esp_lcd 设置的旋转参数保持一致。
         */
        .rotation = {
            .swap_xy = LCD_SWAP_XY,
            .mirror_x = LCD_MIRROR_X,
            .mirror_y = LCD_MIRROR_Y,
        },

        .flags = {
            .buff_dma = true,
            .full_refresh = true,

            /*
             * 已经由 esp_lcd 的 swap_color_bytes
             * 完成字节交换，所以这里不能再次交换。
             */
            .swap_bytes = false,

            .sw_rotate = false,
        },
    };

    s_lvgl_display =
        lvgl_port_add_disp(&display_config);

    if (s_lvgl_display == NULL) {
        ESP_LOGE(TAG, "Failed to register LVGL display");
        abort();
    }

    ESP_LOGI(TAG, "LVGL display registered");
}

/* ============================================================
 * 主函数
 * ============================================================ */

/**
 * @brief 使用固件编译时间为系统时钟设置初始值。
 *
 * 板卡启动时没有 RTC 或网络校时依赖；该初值能让日志具有可用的时间基准，
 * 直到后续加入真正的校时功能。
 */
static void board_set_time_from_build(void)
{
    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char month_name[4] = {0};
    int day = 1;
    int year = 2026;
    int hour = 0;
    int minute = 0;
    int second = 0;

    if (sscanf(
            __DATE__, "%3s %d %d", month_name, &day, &year) != 3 ||
        sscanf(
            __TIME__, "%d:%d:%d", &hour, &minute, &second) != 3) {
        return;
    }

    const char *month_ptr = strstr(months, month_name);
    if (month_ptr == NULL) {
        return;
    }

    struct tm build_time = {
        .tm_year = year - 1900,
        .tm_mon = (int)((month_ptr - months) / 3),
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second,
        .tm_isdst = -1,
    };
    const time_t epoch = mktime(&build_time);
    const struct timeval now = {
        .tv_sec = epoch,
        .tv_usec = 0,
    };
    (void)settimeofday(&now, NULL);
}

/**
 * @brief 电机 HMI 的 ESP-IDF 应用入口函数。
 *
 * 初始化顺序经过设计：先建立 I/O 并熄灭屏幕；在网络组件分配内存前完成 LCD
 * 与 LVGL 初始化；随后在持有 LVGL 锁时创建 UI 和触摸设备。首帧有机会绘制
 * 完成后才打开背光。
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting DNESP32S3B Motor HMI");
    board_set_time_from_build();
    motor_link_init();
    /*
     * 先关闭背光，再初始化屏幕，
     * 可以减少启动过程中的白屏和闪屏。
     */
    board_xl9555_init();
    board_keys_init();
    board_lcd_backlight_set(false);
    board_lcd_init();
    board_lvgl_init();
    /*
     * LVGL 的整屏缓冲区需要一块连续的 DMA 内部内存。
     * 先完成显示注册，避免 Wi-Fi 网络栈和 MQTT 任务提前
     * 占用或碎片化这部分内存。
     */
    const esp_err_t wifi_result = wifi_manager_init();
    if (wifi_result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Wi-Fi initialization failed: %s",
            esp_err_to_name(wifi_result));
    }
    const esp_err_t mqtt_result = mqtt_manager_init();
    if (mqtt_result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "MQTT initialization failed: %s",
            esp_err_to_name(mqtt_result));
    }
    const esp_err_t mqtt_gateway_result = mqtt_motor_gateway_init();
    if (mqtt_gateway_result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "MQTT motor gateway initialization failed: %s",
            esp_err_to_name(mqtt_gateway_result));
    }

    ESP_LOGI(TAG, "Creating motor control UI");
    lvgl_port_lock(0);
    motor_ui_create(s_lvgl_display);
    ESP_LOGI(TAG, "Motor control UI created");
    const esp_err_t touch_result =
        board_touch_init(
            s_i2c_bus,
            s_lvgl_display,
            board_touch_reset_set);
    if (touch_result == ESP_OK) {
        motor_ui_attach_input(board_touch_get_indev());
    }
    lvgl_port_unlock();

    if (touch_result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Touch initialization failed: %s",
            esp_err_to_name(touch_result));
    }

    /*
     * 等待第一次 LVGL 刷新完成后再打开背光。
     */
    vTaskDelay(pdMS_TO_TICKS(100));
    board_lcd_backlight_set(true);

    ESP_LOGI(
        TAG,
        "Motor HMI started successfully"
    );
}
