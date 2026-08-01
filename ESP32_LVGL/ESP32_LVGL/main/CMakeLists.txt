idf_component_register(
    SRCS
        "main.c"
        "board_touch.c"
        "motor_uart.c"
        "motor_can.c"
        "motor_link.c"
        "motor_ui.c"

    INCLUDE_DIRS
        "."

    PRIV_REQUIRES
        esp_lcd
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_uart
        esp_driver_twai
        esp_timer
        esp_lvgl_port
        lvgl
)
