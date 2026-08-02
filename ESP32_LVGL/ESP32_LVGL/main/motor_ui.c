#include "motor_ui.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "board_keys.h"
#include "mqtt_manager.h"
#include "motor_link.h"
#include "wifi_manager.h"

#define UI_COLOR_BACKGROUND       0x08111FU
#define UI_COLOR_PANEL            0x111D2EU
#define UI_COLOR_PANEL_LIGHT      0x1B2A41U
#define UI_COLOR_TEXT             0xF4F7FBU
#define UI_COLOR_MUTED            0x8FA3BFU
#define UI_COLOR_BLUE             0x2D8CFFU
#define UI_COLOR_CYAN             0x20D6C7U
#define UI_COLOR_GREEN            0x32D583U
#define UI_COLOR_RED              0xFF304FU
#define UI_COLOR_YELLOW           0xFFB454U

#define UI_VIEWPORT_TOP           22
#define UI_PAGE_TOP               0
#define UI_PAGE_HEIGHT            218
#define UI_CHART_POINTS           100U
#define UI_SPEED_LIMIT_RPM        2600
#define UI_SWIPE_MIN_DISTANCE     45
#define UI_PAGE_ANIMATION_MS      160U

typedef enum
{
    UI_PAGE_HOME = 0,
    UI_PAGE_FEEDBACK,
    UI_PAGE_UART,
    UI_PAGE_CAN,
    UI_PAGE_WIFI,
    UI_PAGE_MQTT,
    UI_PAGE_SPEED,
    UI_PAGE_POSITION,
    UI_PAGE_SPEED_CHART,
    UI_PAGE_CURRENT_CHART,
    UI_PAGE_COUNT
} ui_page_t;

static const char *s_page_names[UI_PAGE_COUNT] = {
    "MENU",
    "FEEDBACK",
    "USART",
    "CAN",
    "WI-FI",
    "MQTT",
    "SPEED",
    "POSITION",
    "SPEED CURVE",
    "CURRENT CURVE",
};

static lv_obj_t *s_pages[UI_PAGE_COUNT];
static lv_obj_t *s_page_viewport;
static ui_page_t s_current_page;
static lv_obj_t *s_page_label;
static lv_obj_t *s_uart_status_label;
static lv_obj_t *s_can_status_label;
static lv_obj_t *s_wifi_status_label;
static lv_obj_t *s_mqtt_status_label;
static lv_obj_t *s_home_state_label;
static lv_obj_t *s_home_speed_measured_label;
static lv_obj_t *s_home_speed_target_label;
static lv_obj_t *s_home_position_current_label;
static lv_obj_t *s_home_position_target_label;
static lv_obj_t *s_home_current_label;
static lv_obj_t *s_home_current_reference_label;
static lv_obj_t *s_home_voltage_label;
static lv_obj_t *s_home_mode_label;
static lv_obj_t *s_home_run_label;
static lv_obj_t *s_home_fault_label;
static lv_obj_t *s_home_transport_label;
static lv_obj_t *s_can_state_label;
static lv_obj_t *s_link_diag_label;
static lv_obj_t *s_baud_dropdown;
static lv_obj_t *s_reconnect_button;
static lv_obj_t *s_uart_disconnect_button;
static lv_obj_t *s_can_connect_button;
static lv_obj_t *s_can_disconnect_button;
static lv_obj_t *s_wifi_network_dropdown;
static lv_obj_t *s_wifi_password_textarea;
static lv_obj_t *s_wifi_page_state_label;
static lv_obj_t *s_wifi_detail_label;
static lv_obj_t *s_wifi_keyboard;
static lv_obj_t *s_mqtt_uri_textarea;
static lv_obj_t *s_mqtt_page_state_label;
static lv_obj_t *s_mqtt_rx_label;
static lv_obj_t *s_mqtt_keyboard;
static lv_obj_t *s_stop_button;
static lv_obj_t *s_speed_stop_button;
static lv_obj_t *s_position_stop_button;
static lv_obj_t *s_ack_button;
static lv_obj_t *s_speed_actual_label;
static lv_obj_t *s_speed_reference_label;
static lv_obj_t *s_speed_slider;
static lv_obj_t *s_speed_slider_value;
static lv_obj_t *s_speed_mode_button;
static lv_obj_t *s_speed_mode_button_label;
static lv_obj_t *s_position_current_label;
static lv_obj_t *s_position_target_label;
static lv_obj_t *s_position_slider;
static lv_obj_t *s_position_mode_button;
static lv_obj_t *s_position_mode_button_label;
static lv_obj_t *s_electrical_label;
static lv_obj_t *s_speed_chart;
static lv_chart_series_t *s_speed_measured_series;
static lv_chart_series_t *s_speed_reference_series;
static lv_obj_t *s_speed_chart_top_label;
static lv_obj_t *s_speed_chart_mid_label;
static lv_obj_t *s_speed_chart_bottom_label;
static lv_obj_t *s_speed_chart_value_label;
static lv_obj_t *s_current_chart;
static lv_chart_series_t *s_current_measured_series;
static lv_chart_series_t *s_current_reference_series;
static lv_obj_t *s_current_chart_top_label;
static lv_obj_t *s_current_chart_mid_label;
static lv_obj_t *s_current_chart_bottom_label;
static lv_obj_t *s_current_chart_value_label;
static bool s_speed_dragging;
static bool s_speed_command_pending;
static int16_t s_pending_speed_rpm;
static uint32_t s_speed_command_tick;
static bool s_position_dragging;
static bool s_position_command_pending;
static uint16_t s_pending_position_cdeg;
static uint32_t s_position_command_tick;
static bool s_speed_chart_reverse;
static int16_t s_speed_chart_reference_display = INT16_MIN;
static int16_t s_current_chart_reference_ma = INT16_MIN;
static int32_t s_current_chart_scale_ma = 5000;
static motor_link_snapshot_t s_previous_snapshot;
static bool s_have_previous_snapshot;
static uint32_t s_last_chart_frame;
static uint32_t s_last_chart_tick;
static uint8_t s_key_candidate;
static uint8_t s_key_stable;
static uint8_t s_key_debounce_count;
static lv_point_t s_swipe_start;
static bool s_swipe_tracking;
static bool s_page_animating;
static uint32_t s_wifi_revision = UINT32_MAX;
static uint32_t s_wifi_scan_generation = UINT32_MAX;
static uint32_t s_mqtt_revision = UINT32_MAX;
static uint16_t s_wifi_network_count;
static bool s_wifi_network_secured[WIFI_MANAGER_MAX_APS];
static char s_wifi_network_ssids[WIFI_MANAGER_MAX_APS]
                                  [WIFI_MANAGER_SSID_MAX_LEN + 1U];

static const uint32_t s_uart_baud_rates[] = {
    115200U, 230400U, 460800U, 921600U, 1000000U,
    1500000U, 1843200U, 2000000U,
};

static lv_obj_t *ui_create_label(
    lv_obj_t *parent,
    const char *text,
    uint32_t color,
    const lv_font_t *font)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(
        label, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    return label;
}

static void ui_style_panel(lv_obj_t *object, int32_t radius)
{
    lv_obj_set_style_bg_color(
        object, lv_color_hex(UI_COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(object, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(
        object, lv_color_hex(UI_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_radius(object, radius, LV_PART_MAIN);
    lv_obj_set_style_pad_all(object, 10, LV_PART_MAIN);
    lv_obj_remove_flag(object, LV_OBJ_FLAG_SCROLLABLE);
}

static int32_t ui_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int16_t ui_clamp_speed(int32_t speed)
{
    if (speed > UI_SPEED_LIMIT_RPM) {
        return UI_SPEED_LIMIT_RPM;
    }
    if (speed < -UI_SPEED_LIMIT_RPM) {
        return -UI_SPEED_LIMIT_RPM;
    }
    return (int16_t)speed;
}

static void ui_hide_wifi_keyboard(void)
{
    if (s_wifi_keyboard != NULL) {
        lv_keyboard_set_textarea(s_wifi_keyboard, NULL);
        lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ui_hide_mqtt_keyboard(void)
{
    if (s_mqtt_keyboard != NULL) {
        lv_keyboard_set_textarea(s_mqtt_keyboard, NULL);
        lv_obj_add_flag(s_mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ui_show_page(ui_page_t page)
{
    if (page >= UI_PAGE_COUNT) {
        return;
    }

    if (page != UI_PAGE_WIFI) {
        ui_hide_wifi_keyboard();
    }
    if (page != UI_PAGE_MQTT) {
        ui_hide_mqtt_keyboard();
    }

    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        lv_anim_delete(s_pages[i], NULL);
        lv_obj_set_pos(s_pages[i], 0, UI_PAGE_TOP);
        if (i == page) {
            lv_obj_remove_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    s_page_animating = false;
    s_current_page = page;
    lv_label_set_text(s_page_label, s_page_names[page]);
    lv_obj_invalidate(lv_screen_active());
}

static void ui_page_animation_set_y(void *object, int32_t value)
{
    lv_obj_set_y((lv_obj_t *)object, value);
}

static void ui_page_animation_out_completed(lv_anim_t *animation)
{
    lv_obj_t *page = lv_anim_get_user_data(animation);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_y(page, UI_PAGE_TOP);
}

static void ui_page_animation_in_completed(lv_anim_t *animation)
{
    lv_obj_t *page = lv_anim_get_user_data(animation);
    lv_obj_set_y(page, UI_PAGE_TOP);
    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        if (s_pages[i] != page) {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_y(s_pages[i], UI_PAGE_TOP);
        }
    }
    s_page_animating = false;
    lv_obj_invalidate(lv_screen_active());
}

static void ui_animate_to_page(ui_page_t page, bool forward)
{
    if (page >= UI_PAGE_COUNT || page == s_current_page ||
        s_page_animating) {
        return;
    }

    /*
     * The large CAN title was the only object still leaving visible pixels
     * on this single-buffer i80 panel.  Use an atomic hide/show transition
     * whenever CAN is one side of the switch, then invalidate the complete
     * screen.  Other pages keep the requested vertical animation.
     */
    if (s_current_page == UI_PAGE_CAN || page == UI_PAGE_CAN) {
        ui_show_page(page);
        return;
    }

    if (page != UI_PAGE_WIFI) {
        ui_hide_wifi_keyboard();
    }
    if (page != UI_PAGE_MQTT) {
        ui_hide_mqtt_keyboard();
    }

    lv_obj_t *outgoing = s_pages[s_current_page];
    lv_obj_t *incoming = s_pages[page];
    const int32_t incoming_start =
        UI_PAGE_TOP + (forward ? UI_PAGE_HEIGHT : -UI_PAGE_HEIGHT);
    const int32_t outgoing_end =
        UI_PAGE_TOP + (forward ? -UI_PAGE_HEIGHT : UI_PAGE_HEIGHT);

    s_page_animating = true;
    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        lv_anim_delete(s_pages[i], NULL);
        if (s_pages[i] != outgoing && s_pages[i] != incoming) {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_y(s_pages[i], UI_PAGE_TOP);
        }
    }
    lv_obj_set_y(outgoing, UI_PAGE_TOP);
    lv_obj_remove_flag(incoming, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_y(incoming, incoming_start);
    s_current_page = page;
    lv_label_set_text(s_page_label, s_page_names[page]);

    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, outgoing);
    lv_anim_set_exec_cb(&animation, ui_page_animation_set_y);
    lv_anim_set_values(&animation, UI_PAGE_TOP, outgoing_end);
    lv_anim_set_duration(&animation, UI_PAGE_ANIMATION_MS);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_out);
    lv_anim_set_user_data(&animation, outgoing);
    lv_anim_set_completed_cb(
        &animation, ui_page_animation_out_completed);
    lv_anim_start(&animation);

    lv_anim_init(&animation);
    lv_anim_set_var(&animation, incoming);
    lv_anim_set_exec_cb(&animation, ui_page_animation_set_y);
    lv_anim_set_values(&animation, incoming_start, UI_PAGE_TOP);
    lv_anim_set_duration(&animation, UI_PAGE_ANIMATION_MS);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_out);
    lv_anim_set_user_data(&animation, incoming);
    lv_anim_set_completed_cb(
        &animation, ui_page_animation_in_completed);
    lv_anim_start(&animation);
}

static void ui_navigation_event(lv_event_t *event)
{
    const ui_page_t page =
        (ui_page_t)(uintptr_t)lv_event_get_user_data(event);
    ui_animate_to_page(page, page >= s_current_page);
}

static void ui_input_event(lv_event_t *event)
{
    lv_indev_t *indev = lv_event_get_user_data(event);
    const lv_event_code_t code = lv_event_get_code(event);
    lv_point_t point;

    if (indev == NULL) {
        return;
    }

    if (code == LV_EVENT_PRESSED &&
        s_current_page == UI_PAGE_SPEED_CHART) {
        lv_indev_get_point(indev, &s_swipe_start);
        s_swipe_tracking = true;
        return;
    }

    if (code != LV_EVENT_RELEASED || !s_swipe_tracking) {
        return;
    }

    s_swipe_tracking = false;
    if (s_current_page != UI_PAGE_SPEED_CHART) {
        return;
    }

    lv_indev_get_point(indev, &point);
    const int32_t delta_x = point.x - s_swipe_start.x;
    const int32_t delta_y = point.y - s_swipe_start.y;
    if (ui_abs_i32(delta_y) < UI_SWIPE_MIN_DISTANCE ||
        ui_abs_i32(delta_y) <= ui_abs_i32(delta_x)) {
        return;
    }

    motor_link_snapshot_t snapshot;
    motor_link_get_snapshot(&snapshot);
    const int32_t current_reference = s_speed_command_pending
        ? s_pending_speed_rpm
        : snapshot.reference_speed_rpm;
    s_pending_speed_rpm = ui_clamp_speed(
        current_reference + (delta_y < 0 ? 100 : -100));
    s_speed_command_pending = true;
    s_speed_command_tick = lv_tick_get();
    motor_link_set_mode(MOTOR_LINK_MODE_SPEED);
    motor_link_set_speed_rpm(s_pending_speed_rpm);
}

static void ui_speed_mode_event(lv_event_t *event)
{
    (void)event;
    motor_link_set_mode(MOTOR_LINK_MODE_SPEED);
    motor_link_start_motor();
}

static void ui_position_mode_event(lv_event_t *event)
{
    (void)event;
    motor_link_set_mode(MOTOR_LINK_MODE_POSITION);
    motor_link_start_motor();
}

static void ui_stop_event(lv_event_t *event)
{
    (void)event;
    s_speed_command_pending = false;
    s_position_command_pending = false;
    s_speed_dragging = false;
    s_position_dragging = false;
    motor_link_stop_motor();
}

static void ui_ack_fault_event(lv_event_t *event)
{
    (void)event;
    motor_link_acknowledge_fault();
}

static void ui_uart_reconnect_event(lv_event_t *event)
{
    (void)event;
    uint16_t selected = lv_dropdown_get_selected(s_baud_dropdown);
    const uint16_t count = (uint16_t)(sizeof(s_uart_baud_rates) /
                                      sizeof(s_uart_baud_rates[0]));
    if (selected >= count) {
        selected = 0U;
    }
    const uint32_t baud = s_uart_baud_rates[selected];
    if (motor_link_connect_uart(baud) == ESP_OK) {
        s_speed_command_pending = false;
        s_position_command_pending = false;
        lv_label_set_text_fmt(s_home_state_label,
                              "SERIAL CONNECTING %lu", (unsigned long)baud);
    } else {
        lv_label_set_text(s_home_state_label, "SERIAL CONNECT FAILED");
    }
}

static void ui_can_connect_event(lv_event_t *event)
{
    (void)event;
    if (motor_link_connect_can() == ESP_OK) {
        s_speed_command_pending = false;
        s_position_command_pending = false;
        lv_label_set_text(s_can_state_label, "CAN CONNECTING 500K");
    } else {
        lv_label_set_text(s_can_state_label, "CAN CONNECT FAILED");
    }
}

static void ui_uart_disconnect_event(lv_event_t *event)
{
    (void)event;
    s_speed_command_pending = false;
    s_position_command_pending = false;
    motor_link_disconnect_uart();
    lv_label_set_text(s_home_state_label, "USART DISCONNECTED");
}

static void ui_can_disconnect_event(lv_event_t *event)
{
    (void)event;
    s_speed_command_pending = false;
    s_position_command_pending = false;
    motor_link_disconnect_can();
    lv_label_set_text(s_can_state_label, "CAN DISCONNECTED");
}

static void ui_wifi_update_selected_detail(void)
{
    if (s_wifi_detail_label == NULL) {
        return;
    }
    if (s_wifi_network_count == 0U) {
        lv_label_set_text(s_wifi_detail_label, "No network selected");
        return;
    }

    uint16_t selected =
        lv_dropdown_get_selected(s_wifi_network_dropdown);
    if (selected >= s_wifi_network_count) {
        selected = 0U;
    }

    wifi_manager_snapshot_t snapshot;
    wifi_manager_get_snapshot(&snapshot);
    int rssi = 0;
    if (selected < snapshot.ap_count &&
        strcmp(
            snapshot.aps[selected].ssid,
            s_wifi_network_ssids[selected]) == 0) {
        rssi = snapshot.aps[selected].rssi;
    }
    lv_label_set_text_fmt(
        s_wifi_detail_label,
        "%s\n%d dBm  %s",
        s_wifi_network_ssids[selected],
        rssi,
        s_wifi_network_secured[selected] ? "SECURED" : "OPEN");
}

static void ui_wifi_network_event(lv_event_t *event)
{
    (void)event;
    ui_wifi_update_selected_detail();
}

static void ui_wifi_password_event(lv_event_t *event)
{
    (void)event;
    lv_keyboard_set_textarea(
        s_wifi_keyboard,
        s_wifi_password_textarea);
    lv_obj_remove_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_wifi_keyboard);
}

static void ui_wifi_keyboard_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        ui_hide_wifi_keyboard();
    }
}

static void ui_wifi_scan_event(lv_event_t *event)
{
    (void)event;
    const esp_err_t result = wifi_manager_scan_async();
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_wifi_page_state_label,
            "Scan unavailable: %s",
            esp_err_to_name(result));
    }
}

static void ui_wifi_connect_event(lv_event_t *event)
{
    (void)event;
    if (s_wifi_network_count == 0U) {
        lv_label_set_text(s_wifi_page_state_label, "Scan and select a network");
        return;
    }

    uint16_t selected =
        lv_dropdown_get_selected(s_wifi_network_dropdown);
    if (selected >= s_wifi_network_count) {
        selected = 0U;
    }
    const char *password =
        lv_textarea_get_text(s_wifi_password_textarea);
    if (s_wifi_network_secured[selected] && strlen(password) < 8U) {
        lv_label_set_text(
            s_wifi_page_state_label,
            "Secure network password must be 8+ characters");
        return;
    }
    if (!s_wifi_network_secured[selected]) {
        password = "";
    }

    ui_hide_wifi_keyboard();
    const esp_err_t result = wifi_manager_connect(
        s_wifi_network_ssids[selected],
        password);
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_wifi_page_state_label,
            "Connect unavailable: %s",
            esp_err_to_name(result));
    }
}

static void ui_wifi_disconnect_event(lv_event_t *event)
{
    (void)event;
    ui_hide_wifi_keyboard();
    const esp_err_t result = wifi_manager_disconnect();
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_wifi_page_state_label,
            "Disconnect failed: %s",
            esp_err_to_name(result));
    }
}

static void ui_mqtt_uri_event(lv_event_t *event)
{
    (void)event;
    lv_keyboard_set_textarea(s_mqtt_keyboard, s_mqtt_uri_textarea);
    lv_obj_remove_flag(s_mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_mqtt_keyboard);
}

static void ui_mqtt_keyboard_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        ui_hide_mqtt_keyboard();
    }
}

static void ui_mqtt_connect_event(lv_event_t *event)
{
    (void)event;
    wifi_manager_snapshot_t wifi_snapshot;
    wifi_manager_get_snapshot(&wifi_snapshot);
    if (!wifi_snapshot.connected) {
        lv_label_set_text(
            s_mqtt_page_state_label,
            "Connect Wi-Fi before MQTT");
        return;
    }

    ui_hide_mqtt_keyboard();
    const esp_err_t result = mqtt_manager_connect_async(
        lv_textarea_get_text(s_mqtt_uri_textarea));
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_mqtt_page_state_label,
            "Invalid broker URI: %s",
            esp_err_to_name(result));
    }
}

static void ui_mqtt_disconnect_event(lv_event_t *event)
{
    (void)event;
    ui_hide_mqtt_keyboard();
    const esp_err_t result = mqtt_manager_disconnect_async();
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_mqtt_page_state_label,
            "Disconnect failed: %s",
            esp_err_to_name(result));
    }
}

static void ui_mqtt_publish_test(
    const char *topic,
    const char *payload)
{
    const esp_err_t result = mqtt_manager_publish(topic, payload);
    if (result != ESP_OK) {
        lv_label_set_text(
            s_mqtt_page_state_label,
            "MQTT is offline - connect first");
    }
}

static void ui_mqtt_ping_event(lv_event_t *event)
{
    (void)event;
    ui_mqtt_publish_test(
        "motor/hmi/test/ping",
        "PING from ESP32-S3");
}

static void ui_mqtt_wifi_event(lv_event_t *event)
{
    (void)event;
    wifi_manager_snapshot_t snapshot;
    wifi_manager_get_snapshot(&snapshot);
    char payload[128];
    snprintf(
        payload,
        sizeof(payload),
        "ssid=%s ip=%s",
        snapshot.ssid,
        snapshot.ip_address);
    ui_mqtt_publish_test("motor/hmi/test/wifi", payload);
}

static void ui_mqtt_motor_event(lv_event_t *event)
{
    (void)event;
    motor_link_snapshot_t snapshot;
    motor_link_get_snapshot(&snapshot);
    char payload[160];
    snprintf(
        payload,
        sizeof(payload),
        "running=%u mode=%s speed=%d position=%u.%02u",
        snapshot.motor_running ? 1U : 0U,
        snapshot.mode == MOTOR_LINK_MODE_SPEED ? "speed" : "position",
        snapshot.measured_speed_rpm,
        snapshot.current_position_cdeg / 100U,
        snapshot.current_position_cdeg % 100U);
    ui_mqtt_publish_test("motor/hmi/test/motor", payload);
}

static void ui_speed_slider_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_PRESSED) {
        s_speed_dragging = true;
        return;
    }
    if (code == LV_EVENT_VALUE_CHANGED) {
        const int32_t speed = lv_slider_get_value(s_speed_slider);
        lv_label_set_text_fmt(
            s_speed_slider_value, "%ld RPM", (long)speed);
        return;
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (s_speed_dragging) {
            s_pending_speed_rpm =
                (int16_t)lv_slider_get_value(s_speed_slider);
            s_speed_command_pending = true;
            s_speed_command_tick = lv_tick_get();
            motor_link_set_mode(MOTOR_LINK_MODE_SPEED);
            motor_link_set_speed_rpm(s_pending_speed_rpm);
        }
        s_speed_dragging = false;
    }
}

static void ui_position_slider_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_PRESSED) {
        s_position_dragging = true;
    }
    if (code == LV_EVENT_VALUE_CHANGED) {
        const int32_t cdeg = lv_slider_get_value(s_position_slider);
        lv_label_set_text_fmt(
            s_position_target_label,
            "%3ld.%02ld deg",
            (long)(cdeg / 100L),
            (long)(cdeg % 100L));
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (s_position_dragging) {
            s_pending_position_cdeg =
                (uint16_t)lv_slider_get_value(s_position_slider);
            s_position_command_pending = true;
            s_position_command_tick = lv_tick_get();
            motor_link_set_mode(MOTOR_LINK_MODE_POSITION);
            motor_link_set_position_cdeg(s_pending_position_cdeg);
        }
        s_position_dragging = false;
    }
}

static lv_obj_t *ui_create_mode_button(
    lv_obj_t *parent,
    const char *text,
    lv_event_cb_t callback,
    lv_obj_t **label_out)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 116, 40);
    lv_obj_set_style_radius(button, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        button, lv_color_hex(UI_COLOR_BLUE), LV_PART_MAIN);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label =
        ui_create_label(button, text, UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(label);
    *label_out = label;
    return button;
}

static lv_obj_t *ui_create_stop_button(
    lv_obj_t *parent,
    int32_t width,
    int32_t height)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, width, height);
    lv_obj_set_style_radius(button, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        button, lv_color_hex(UI_COLOR_RED), LV_PART_MAIN);
    lv_obj_add_event_cb(
        button, ui_stop_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = ui_create_label(
        button, "STOP", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(label);
    return button;
}

static void ui_create_navigation_button(
    lv_obj_t *parent,
    const char *text,
    ui_page_t page,
    int32_t y)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 286, 36);
    lv_obj_set_pos(button, 1, y);
    lv_obj_set_style_radius(button, 9, LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        button, lv_color_hex(UI_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_add_event_cb(
        button, ui_navigation_event, LV_EVENT_CLICKED,
        (void *)(uintptr_t)page);
    lv_obj_t *label = ui_create_label(
        button, text, UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(label);
}

static void ui_create_navigation_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "PAGE SELECT", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_set_pos(title, 8, 2);

    lv_obj_t *list = lv_obj_create(parent);
    lv_obj_set_size(list, 304, 188);
    lv_obj_set_pos(list, 8, 26);
    lv_obj_set_style_bg_color(
        list, lv_color_hex(UI_COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(
        list, lv_color_hex(UI_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_border_width(list, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(list, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(list, 7, LV_PART_MAIN);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

    ui_create_navigation_button(list, "MOTOR FEEDBACK", UI_PAGE_FEEDBACK, 1);
    ui_create_navigation_button(list, "USART", UI_PAGE_UART, 43);
    ui_create_navigation_button(list, "CAN", UI_PAGE_CAN, 85);
    ui_create_navigation_button(list, "WI-FI", UI_PAGE_WIFI, 127);
    ui_create_navigation_button(list, "MQTT", UI_PAGE_MQTT, 169);
    ui_create_navigation_button(list, "SPEED CONTROL", UI_PAGE_SPEED, 211);
    ui_create_navigation_button(list, "POSITION CONTROL", UI_PAGE_POSITION, 253);
    ui_create_navigation_button(list, "SPEED CURVE", UI_PAGE_SPEED_CHART, 295);
    ui_create_navigation_button(list, "CURRENT CURVE", UI_PAGE_CURRENT_CHART, 337);
}

static void ui_create_feedback_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "MOTOR FEEDBACK", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_set_pos(title, 8, 1);

    lv_obj_t *speed_panel = lv_obj_create(parent);
    lv_obj_set_size(speed_panel, 151, 61);
    lv_obj_set_pos(speed_panel, 6, 20);
    ui_style_panel(speed_panel, 8);
    lv_obj_set_style_pad_all(speed_panel, 6, LV_PART_MAIN);
    lv_obj_t *label = ui_create_label(
        speed_panel, "SPEED", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);
    s_home_speed_measured_label = ui_create_label(
        speed_panel, "0 RPM", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_set_pos(s_home_speed_measured_label, 0, 14);
    s_home_speed_target_label = ui_create_label(
        speed_panel, "TARGET 0 RPM", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_speed_target_label, 0, 39);

    lv_obj_t *position_panel = lv_obj_create(parent);
    lv_obj_set_size(position_panel, 151, 61);
    lv_obj_set_pos(position_panel, 163, 20);
    ui_style_panel(position_panel, 8);
    lv_obj_set_style_pad_all(position_panel, 6, LV_PART_MAIN);
    label = ui_create_label(
        position_panel, "POSITION", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);
    s_home_position_current_label = ui_create_label(
        position_panel, "0.00 deg", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_set_pos(s_home_position_current_label, 0, 14);
    s_home_position_target_label = ui_create_label(
        position_panel, "TARGET 0.00 deg", UI_COLOR_TEXT,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_position_target_label, 0, 39);

    lv_obj_t *electrical_panel = lv_obj_create(parent);
    lv_obj_set_size(electrical_panel, 308, 66);
    lv_obj_set_pos(electrical_panel, 6, 85);
    ui_style_panel(electrical_panel, 8);
    lv_obj_set_style_pad_all(electrical_panel, 6, LV_PART_MAIN);
    label = ui_create_label(
        electrical_panel, "ELECTRICAL FEEDBACK", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);
    s_home_current_label = ui_create_label(
        electrical_panel, "Iq     0 mA\nId     0 mA", UI_COLOR_TEXT,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_current_label, 0, 17);
    s_home_current_reference_label = ui_create_label(
        electrical_panel, "Iq*    0 mA\nId*    0 mA", UI_COLOR_GREEN,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_current_reference_label, 104, 17);
    s_home_voltage_label = ui_create_label(
        electrical_panel, "Uq     0 mV\nUd     0 mV", UI_COLOR_YELLOW,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_voltage_label, 208, 17);

    lv_obj_t *status_panel = lv_obj_create(parent);
    lv_obj_set_size(status_panel, 308, 57);
    lv_obj_set_pos(status_panel, 6, 155);
    ui_style_panel(status_panel, 8);
    lv_obj_set_style_pad_all(status_panel, 6, LV_PART_MAIN);
    s_home_mode_label = ui_create_label(
        status_panel, "MODE SPEED", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_mode_label, 0, 1);
    s_home_run_label = ui_create_label(
        status_panel, "STATE STOPPED", UI_COLOR_YELLOW,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_run_label, 150, 1);
    s_home_transport_label = ui_create_label(
        status_panel, "LINK NONE", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_transport_label, 0, 25);
    s_home_fault_label = ui_create_label(
        status_panel, "FAULT 0x0000", UI_COLOR_GREEN,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_fault_label, 150, 25);
}

static void ui_create_uart_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "USART COMMUNICATION", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 12);
    lv_obj_t *port = ui_create_label(
        parent, "PORT1  GPIO18 TX / GPIO8 RX", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align_to(port, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

    s_link_diag_label = ui_create_label(
        parent, "UART RX 0  TX 0  ERR 0\nFAULT 0x0000", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align(s_link_diag_label, LV_ALIGN_CENTER, 0, -5);

    s_baud_dropdown = lv_dropdown_create(parent);
    lv_dropdown_set_options(s_baud_dropdown,
                            "115200\n230400\n460800\n921600\n1000000\n"
                            "1500000\n1843200\n2000000");
    lv_dropdown_set_selected(s_baud_dropdown, 0U);
    lv_obj_set_size(s_baud_dropdown, 108, 36);
    lv_obj_set_pos(s_baud_dropdown, 7, 158);
    lv_obj_set_style_bg_color(s_baud_dropdown,
                              lv_color_hex(UI_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_text_color(s_baud_dropdown,
                                lv_color_hex(UI_COLOR_TEXT), LV_PART_MAIN);

    s_reconnect_button = lv_button_create(parent);
    lv_obj_set_size(s_reconnect_button, 94, 36);
    lv_obj_set_pos(s_reconnect_button, 120, 158);
    lv_obj_set_style_radius(s_reconnect_button, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_reconnect_button,
                              lv_color_hex(UI_COLOR_BLUE), LV_PART_MAIN);
    lv_obj_add_event_cb(s_reconnect_button, ui_uart_reconnect_event,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t *button_label = ui_create_label(s_reconnect_button, "CONNECT",
                                              UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(button_label);

    s_uart_disconnect_button = lv_button_create(parent);
    lv_obj_set_size(s_uart_disconnect_button, 94, 36);
    lv_obj_set_pos(s_uart_disconnect_button, 219, 158);
    lv_obj_set_style_radius(s_uart_disconnect_button, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_uart_disconnect_button,
                              lv_color_hex(UI_COLOR_RED), LV_PART_MAIN);
    lv_obj_add_event_cb(s_uart_disconnect_button, ui_uart_disconnect_event,
                        LV_EVENT_CLICKED, NULL);
    button_label = ui_create_label(s_uart_disconnect_button, "DISCONNECT",
                                   UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(button_label);

    s_home_state_label = ui_create_label(parent, "SELECT BAUD AND CONNECT",
                                          UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_home_state_label, LV_ALIGN_BOTTOM_MID, 0, -7);
}

static void ui_create_can_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "CAN COMMUNICATION", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 12);
    lv_obj_t *detail = ui_create_label(
        parent, "J2/PORT1  GPIO5/6  CLASSIC CAN 500K", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align_to(detail, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    s_stop_button = lv_button_create(parent);
    lv_obj_set_size(s_stop_button, 132, 36);
    lv_obj_set_pos(s_stop_button, 22, 116);
    lv_obj_set_style_radius(s_stop_button, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_stop_button, lv_color_hex(UI_COLOR_RED), LV_PART_MAIN);
    lv_obj_add_event_cb(s_stop_button, ui_stop_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *stop = ui_create_label(s_stop_button, "STOP", UI_COLOR_TEXT,
                                     &lv_font_montserrat_12);
    lv_obj_center(stop);

    s_ack_button = lv_button_create(parent);
    lv_obj_set_size(s_ack_button, 132, 36);
    lv_obj_set_pos(s_ack_button, 166, 116);
    lv_obj_set_style_radius(s_ack_button, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ack_button, lv_color_hex(UI_COLOR_BLUE), LV_PART_MAIN);
    lv_obj_add_event_cb(s_ack_button, ui_ack_fault_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ack = ui_create_label(s_ack_button, "ACK FAULT", UI_COLOR_TEXT,
                                    &lv_font_montserrat_12);
    lv_obj_center(ack);

    s_can_connect_button = lv_button_create(parent);
    lv_obj_set_size(s_can_connect_button, 132, 40);
    lv_obj_set_pos(s_can_connect_button, 22, 64);
    lv_obj_set_style_radius(s_can_connect_button, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_can_connect_button,
                              lv_color_hex(UI_COLOR_CYAN), LV_PART_MAIN);
    lv_obj_add_event_cb(s_can_connect_button, ui_can_connect_event,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t *button_label = ui_create_label(s_can_connect_button, "CONNECT",
                                              UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(button_label);

    s_can_disconnect_button = lv_button_create(parent);
    lv_obj_set_size(s_can_disconnect_button, 132, 40);
    lv_obj_set_pos(s_can_disconnect_button, 166, 64);
    lv_obj_set_style_radius(s_can_disconnect_button, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_can_disconnect_button,
                              lv_color_hex(UI_COLOR_RED), LV_PART_MAIN);
    lv_obj_add_event_cb(s_can_disconnect_button, ui_can_disconnect_event,
                        LV_EVENT_CLICKED, NULL);
    button_label = ui_create_label(s_can_disconnect_button, "DISCONNECT",
                                   UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(button_label);

    s_can_state_label = ui_create_label(
        parent, "Connecting selects CAN as the active control transport.",
        UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_can_state_label, LV_ALIGN_BOTTOM_MID, 0, -24);
}

static lv_obj_t *ui_create_wifi_action_button(
    lv_obj_t *parent,
    const char *text,
    uint32_t color,
    int32_t x,
    int32_t y,
    lv_event_cb_t callback)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 92, 34);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_style_radius(button, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        button, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = ui_create_label(
        button, text, UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(label);
    return button;
}

static void ui_create_wifi_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "WI-FI NETWORK", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_set_pos(title, 8, 4);

    s_wifi_network_dropdown = lv_dropdown_create(parent);
    lv_dropdown_set_options(s_wifi_network_dropdown, "No networks - tap SCAN");
    lv_obj_set_size(s_wifi_network_dropdown, 210, 34);
    lv_obj_set_pos(s_wifi_network_dropdown, 8, 34);
    lv_obj_set_style_bg_color(
        s_wifi_network_dropdown,
        lv_color_hex(UI_COLOR_PANEL_LIGHT),
        LV_PART_MAIN);
    lv_obj_set_style_text_color(
        s_wifi_network_dropdown,
        lv_color_hex(UI_COLOR_TEXT),
        LV_PART_MAIN);
    lv_obj_add_event_cb(
        s_wifi_network_dropdown,
        ui_wifi_network_event,
        LV_EVENT_VALUE_CHANGED,
        NULL);

    ui_create_wifi_action_button(
        parent, "SCAN", UI_COLOR_CYAN, 220, 34, ui_wifi_scan_event);

    s_wifi_password_textarea = lv_textarea_create(parent);
    lv_obj_set_size(s_wifi_password_textarea, 204, 34);
    lv_obj_set_pos(s_wifi_password_textarea, 8, 76);
    lv_textarea_set_one_line(s_wifi_password_textarea, true);
    lv_textarea_set_password_mode(s_wifi_password_textarea, false);
    lv_textarea_set_max_length(s_wifi_password_textarea, 63U);
    lv_textarea_set_text(s_wifi_password_textarea, "13579035076");
    lv_textarea_set_placeholder_text(
        s_wifi_password_textarea,
        "Wi-Fi password");
    lv_obj_set_style_bg_color(
        s_wifi_password_textarea,
        lv_color_hex(UI_COLOR_PANEL),
        LV_PART_MAIN);
    lv_obj_set_style_text_color(
        s_wifi_password_textarea,
        lv_color_hex(UI_COLOR_TEXT),
        LV_PART_MAIN);
    lv_obj_set_style_border_color(
        s_wifi_password_textarea,
        lv_color_hex(UI_COLOR_PANEL_LIGHT),
        LV_PART_MAIN);
    lv_obj_add_event_cb(
        s_wifi_password_textarea,
        ui_wifi_password_event,
        LV_EVENT_CLICKED,
        NULL);

    ui_create_wifi_action_button(
        parent, "CONNECT", UI_COLOR_BLUE, 220, 76,
        ui_wifi_connect_event);
    ui_create_wifi_action_button(
        parent, "DISCONNECT", UI_COLOR_RED, 220, 118,
        ui_wifi_disconnect_event);

    s_wifi_detail_label = ui_create_label(
        parent, "No network selected", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_wifi_detail_label, 10, 120);
    lv_obj_set_width(s_wifi_detail_label, 202);
    lv_label_set_long_mode(s_wifi_detail_label, LV_LABEL_LONG_DOT);

    s_wifi_page_state_label = ui_create_label(
        parent, "Initializing Wi-Fi", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_wifi_page_state_label, 10, 165);
    lv_obj_set_width(s_wifi_page_state_label, 300);
    lv_label_set_long_mode(s_wifi_page_state_label, LV_LABEL_LONG_WRAP);

    s_wifi_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_set_size(s_wifi_keyboard, 320, 160);
    lv_obj_align(s_wifi_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_wifi_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_set_style_bg_color(
        s_wifi_keyboard, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_add_event_cb(
        s_wifi_keyboard,
        ui_wifi_keyboard_event,
        LV_EVENT_ALL,
        NULL);
    lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void ui_create_mqtt_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "MQTT TEST", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_set_pos(title, 8, 4);

    s_mqtt_uri_textarea = lv_textarea_create(parent);
    lv_obj_set_size(s_mqtt_uri_textarea, 204, 34);
    lv_obj_set_pos(s_mqtt_uri_textarea, 8, 34);
    lv_textarea_set_one_line(s_mqtt_uri_textarea, true);
    lv_textarea_set_max_length(
        s_mqtt_uri_textarea,
        MQTT_MANAGER_URI_MAX_LEN);
    lv_textarea_set_text(
        s_mqtt_uri_textarea,
        "mqtt://192.168.10.4:1883");
    lv_obj_set_style_bg_color(
        s_mqtt_uri_textarea,
        lv_color_hex(UI_COLOR_PANEL),
        LV_PART_MAIN);
    lv_obj_set_style_text_color(
        s_mqtt_uri_textarea,
        lv_color_hex(UI_COLOR_TEXT),
        LV_PART_MAIN);
    lv_obj_set_style_border_color(
        s_mqtt_uri_textarea,
        lv_color_hex(UI_COLOR_PANEL_LIGHT),
        LV_PART_MAIN);
    lv_obj_add_event_cb(
        s_mqtt_uri_textarea,
        ui_mqtt_uri_event,
        LV_EVENT_CLICKED,
        NULL);

    ui_create_wifi_action_button(
        parent, "CONNECT", UI_COLOR_BLUE, 220, 34,
        ui_mqtt_connect_event);

    s_mqtt_page_state_label = ui_create_label(
        parent, "Connect Wi-Fi, then connect MQTT", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_mqtt_page_state_label, 10, 75);
    lv_obj_set_width(s_mqtt_page_state_label, 300);
    lv_label_set_long_mode(s_mqtt_page_state_label, LV_LABEL_LONG_DOT);

    ui_create_wifi_action_button(
        parent, "PING", UI_COLOR_CYAN, 8, 104,
        ui_mqtt_ping_event);
    ui_create_wifi_action_button(
        parent, "WI-FI INFO", UI_COLOR_CYAN, 114, 104,
        ui_mqtt_wifi_event);
    ui_create_wifi_action_button(
        parent, "MOTOR", UI_COLOR_CYAN, 220, 104,
        ui_mqtt_motor_event);

    s_mqtt_rx_label = ui_create_label(
        parent,
        "RX motor/hmi/test/rx\nNo message from MQTTX",
        UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_set_pos(s_mqtt_rx_label, 10, 149);
    lv_obj_set_size(s_mqtt_rx_label, 202, 48);
    lv_label_set_long_mode(s_mqtt_rx_label, LV_LABEL_LONG_DOT);

    ui_create_wifi_action_button(
        parent, "DISCONNECT", UI_COLOR_RED, 220, 153,
        ui_mqtt_disconnect_event);

    s_mqtt_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_set_size(s_mqtt_keyboard, 320, 160);
    lv_obj_align(s_mqtt_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_mqtt_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_set_style_bg_color(
        s_mqtt_keyboard, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_add_event_cb(
        s_mqtt_keyboard,
        ui_mqtt_keyboard_event,
        LV_EVENT_ALL,
        NULL);
    lv_obj_add_flag(s_mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void ui_create_speed_page(lv_obj_t *parent)
{
    lv_obj_t *actual_card = lv_obj_create(parent);
    lv_obj_set_size(actual_card, 144, 68);
    lv_obj_align(actual_card, LV_ALIGN_TOP_LEFT, 8, 10);
    ui_style_panel(actual_card, 14);

    lv_obj_t *actual_title = ui_create_label(
        actual_card, "ACTUAL SPEED", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(actual_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_speed_actual_label = ui_create_label(
        actual_card, "0 RPM", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_align(s_speed_actual_label, LV_ALIGN_BOTTOM_LEFT, 0, -3);

    lv_obj_t *reference_card = lv_obj_create(parent);
    lv_obj_set_size(reference_card, 144, 68);
    lv_obj_align(reference_card, LV_ALIGN_TOP_RIGHT, -8, 10);
    ui_style_panel(reference_card, 14);

    lv_obj_t *reference_title = ui_create_label(
        reference_card, "REFERENCE", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(reference_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_speed_reference_label = ui_create_label(
        reference_card, "0 RPM", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(s_speed_reference_label, LV_ALIGN_BOTTOM_LEFT, 0, -3);

    s_speed_mode_button = ui_create_mode_button(
        parent, "ENABLE SPEED", ui_speed_mode_event,
        &s_speed_mode_button_label);
    lv_obj_align(s_speed_mode_button, LV_ALIGN_CENTER, -66, 3);

    s_speed_stop_button =
        ui_create_stop_button(parent, 116, 40);
    lv_obj_align(
        s_speed_stop_button, LV_ALIGN_CENTER, 66, 3);

    s_speed_slider_value = ui_create_label(
        parent, "0 RPM", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_align(s_speed_slider_value, LV_ALIGN_BOTTOM_MID, 0, -40);

    s_speed_slider = lv_slider_create(parent);
    lv_obj_set_size(s_speed_slider, 284, 16);
    lv_obj_align(s_speed_slider, LV_ALIGN_BOTTOM_MID, 0, -14);
    lv_slider_set_range(
        s_speed_slider, -UI_SPEED_LIMIT_RPM, UI_SPEED_LIMIT_RPM);
    lv_slider_set_value(s_speed_slider, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(
        s_speed_slider, lv_color_hex(UI_COLOR_PANEL_LIGHT),
        LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        s_speed_slider, lv_color_hex(UI_COLOR_BLUE),
        LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(
        s_speed_slider, lv_color_hex(UI_COLOR_TEXT),
        LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_speed_slider, 8, LV_PART_KNOB);
    lv_obj_add_event_cb(
        s_speed_slider, ui_speed_slider_event, LV_EVENT_ALL, NULL);
}

static void ui_create_position_page(lv_obj_t *parent)
{
    lv_obj_t *current_card = lv_obj_create(parent);
    lv_obj_set_size(current_card, 144, 64);
    lv_obj_align(current_card, LV_ALIGN_TOP_LEFT, 8, 8);
    ui_style_panel(current_card, 14);
    lv_obj_t *current_title = ui_create_label(
        current_card, "CURRENT POSITION", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align(current_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_position_current_label = ui_create_label(
        current_card, "0.00 deg", UI_COLOR_CYAN,
        &lv_font_montserrat_20);
    lv_obj_align(s_position_current_label, LV_ALIGN_BOTTOM_LEFT, 0, -2);

    lv_obj_t *target_card = lv_obj_create(parent);
    lv_obj_set_size(target_card, 144, 64);
    lv_obj_align(target_card, LV_ALIGN_TOP_RIGHT, -8, 8);
    ui_style_panel(target_card, 14);
    lv_obj_t *target_title = ui_create_label(
        target_card, "TARGET POSITION", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align(target_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_position_target_label = ui_create_label(
        target_card, "0.00 deg", UI_COLOR_TEXT,
        &lv_font_montserrat_20);
    lv_obj_align(s_position_target_label, LV_ALIGN_BOTTOM_LEFT, 0, -2);

    s_position_mode_button = ui_create_mode_button(
        parent, "ENABLE POS", ui_position_mode_event,
        &s_position_mode_button_label);
    lv_obj_align(s_position_mode_button, LV_ALIGN_CENTER, -66, -2);

    s_position_stop_button =
        ui_create_stop_button(parent, 116, 40);
    lv_obj_align(
        s_position_stop_button, LV_ALIGN_CENTER, 66, -2);

    s_electrical_label = ui_create_label(
        parent,
        "Iq 0mA  Id 0mA\nIq* 0mA  Id* 0mA",
        UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align(s_electrical_label, LV_ALIGN_BOTTOM_LEFT, 12, -48);

    s_position_slider = lv_slider_create(parent);
    lv_obj_set_size(s_position_slider, 284, 16);
    lv_obj_align(s_position_slider, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_slider_set_range(s_position_slider, 0, 36000);
    lv_slider_set_value(s_position_slider, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(
        s_position_slider, lv_color_hex(UI_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_bg_color(
        s_position_slider, lv_color_hex(UI_COLOR_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(
        s_position_slider, lv_color_hex(UI_COLOR_TEXT), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_position_slider, 8, LV_PART_KNOB);
    lv_obj_add_event_cb(
        s_position_slider, ui_position_slider_event, LV_EVENT_ALL, NULL);
}

static void ui_style_chart(lv_obj_t *chart)
{
    lv_obj_set_style_bg_color(
        chart, lv_color_hex(UI_COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(
        chart, lv_color_hex(UI_COLOR_MUTED), LV_PART_MAIN);
    lv_obj_set_style_border_width(chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_color(
        chart, lv_color_hex(UI_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);
    lv_obj_add_flag(chart, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, UI_CHART_POINTS);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_div_line_count(chart, 5, 5);
}

static void ui_create_chart_axis_labels(
    lv_obj_t *parent,
    lv_obj_t **top,
    lv_obj_t **mid,
    lv_obj_t **bottom)
{
    *top = ui_create_label(
        parent, "2600", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_width(*top, 42);
    lv_obj_set_style_text_align(*top, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(*top, 0, 20);

    *mid = ui_create_label(
        parent, "1300", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_width(*mid, 42);
    lv_obj_set_style_text_align(*mid, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(*mid, 0, 99);

    *bottom = ui_create_label(
        parent, "0", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_width(*bottom, 42);
    lv_obj_set_style_text_align(
        *bottom, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(*bottom, 0, 178);
}

static void ui_create_speed_chart_page(lv_obj_t *parent)
{
    s_speed_chart_value_label = ui_create_label(
        parent,
        "REF 0 RPM   SPEED 0 RPM",
        UI_COLOR_TEXT,
        &lv_font_montserrat_12);
    lv_obj_align(s_speed_chart_value_label, LV_ALIGN_TOP_RIGHT, -8, 0);

    ui_create_chart_axis_labels(
        parent,
        &s_speed_chart_top_label,
        &s_speed_chart_mid_label,
        &s_speed_chart_bottom_label);

    s_speed_chart = lv_chart_create(parent);
    lv_obj_set_size(s_speed_chart, 270, 164);
    lv_obj_set_pos(s_speed_chart, 44, 20);
    ui_style_chart(s_speed_chart);
    lv_chart_set_axis_range(
        s_speed_chart, LV_CHART_AXIS_PRIMARY_Y, 0, UI_SPEED_LIMIT_RPM);

    s_speed_measured_series = lv_chart_add_series(
        s_speed_chart,
        lv_color_hex(UI_COLOR_TEXT),
        LV_CHART_AXIS_PRIMARY_Y);
    s_speed_reference_series = lv_chart_add_series(
        s_speed_chart,
        lv_color_hex(UI_COLOR_RED),
        LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_values(
        s_speed_chart, s_speed_measured_series, 0);
    lv_chart_set_all_values(
        s_speed_chart, s_speed_reference_series, 0);

    lv_obj_t *time_label = ui_create_label(
        parent, "TIME 2 s", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(time_label, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
    lv_obj_t *keys_label = ui_create_label(
        parent,
        "SWIPE UP +100 / DOWN -100 RPM",
        UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align(keys_label, LV_ALIGN_BOTTOM_LEFT, 8, -6);
}

static void ui_create_current_chart_page(lv_obj_t *parent)
{
    s_current_chart_value_label = ui_create_label(
        parent,
        "REF 0 mA   Iq 0 mA",
        UI_COLOR_TEXT,
        &lv_font_montserrat_12);
    lv_obj_align(s_current_chart_value_label, LV_ALIGN_TOP_RIGHT, -8, 0);

    ui_create_chart_axis_labels(
        parent,
        &s_current_chart_top_label,
        &s_current_chart_mid_label,
        &s_current_chart_bottom_label);
    lv_label_set_text_fmt(
        s_current_chart_top_label,
        "%ld",
        (long)s_current_chart_scale_ma);
    lv_label_set_text(s_current_chart_mid_label, "0");
    lv_label_set_text_fmt(
        s_current_chart_bottom_label,
        "-%ld",
        (long)s_current_chart_scale_ma);

    s_current_chart = lv_chart_create(parent);
    lv_obj_set_size(s_current_chart, 270, 164);
    lv_obj_set_pos(s_current_chart, 44, 20);
    ui_style_chart(s_current_chart);
    lv_chart_set_axis_range(
        s_current_chart,
        LV_CHART_AXIS_PRIMARY_Y,
        -s_current_chart_scale_ma,
        s_current_chart_scale_ma);

    s_current_measured_series = lv_chart_add_series(
        s_current_chart,
        lv_color_hex(UI_COLOR_TEXT),
        LV_CHART_AXIS_PRIMARY_Y);
    s_current_reference_series = lv_chart_add_series(
        s_current_chart,
        lv_color_hex(UI_COLOR_RED),
        LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_values(
        s_current_chart, s_current_measured_series, 0);
    lv_chart_set_all_values(
        s_current_chart, s_current_reference_series, 0);

    lv_obj_t *time_label = ui_create_label(
        parent, "TIME 2 s", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(time_label, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
    lv_obj_t *display_label = ui_create_label(
        parent,
        "DISPLAY ONLY",
        UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_align(display_label, LV_ALIGN_BOTTOM_LEFT, 8, -6);
}

static int32_t ui_select_current_scale(
    int32_t measured,
    int32_t reference)
{
    const int32_t peak =
        ui_abs_i32(measured) > ui_abs_i32(reference)
            ? ui_abs_i32(measured)
            : ui_abs_i32(reference);

    if (peak <= 800) {
        return 1000;
    }
    if (peak <= 1600) {
        return 2000;
    }
    if (peak <= 4000) {
        return 5000;
    }
    if (peak <= 8000) {
        return 10000;
    }
    if (peak <= 16000) {
        return 20000;
    }
    return 30000;
}

static void ui_update_charts(const motor_link_snapshot_t *snapshot)
{
    if (lv_tick_elaps(s_last_chart_tick) < 60U) {
        return;
    }
    if (snapshot->received_frames == s_last_chart_frame) {
        return;
    }
    s_last_chart_tick = lv_tick_get();
    s_last_chart_frame = snapshot->received_frames;

    const bool reverse =
        snapshot->reference_speed_rpm < 0 ||
        (snapshot->reference_speed_rpm == 0 &&
         snapshot->measured_speed_rpm < 0);
    if (reverse != s_speed_chart_reverse) {
        s_speed_chart_reverse = reverse;
        s_speed_chart_reference_display = INT16_MIN;
        lv_chart_set_all_values(
            s_speed_chart, s_speed_measured_series, 0);
        lv_label_set_text(
            s_speed_chart_top_label, reverse ? "0" : "2600");
        lv_label_set_text(
            s_speed_chart_mid_label, reverse ? "-1300" : "1300");
        lv_label_set_text(
            s_speed_chart_bottom_label, reverse ? "-2600" : "0");
    }

    int32_t speed_value = reverse
        ? UI_SPEED_LIMIT_RPM + snapshot->measured_speed_rpm
        : snapshot->measured_speed_rpm;
    int32_t speed_reference = reverse
        ? UI_SPEED_LIMIT_RPM + snapshot->reference_speed_rpm
        : snapshot->reference_speed_rpm;
    if (speed_value < 0) {
        speed_value = 0;
    }
    if (speed_reference < 0) {
        speed_reference = 0;
    }
    if (speed_value > UI_SPEED_LIMIT_RPM) {
        speed_value = UI_SPEED_LIMIT_RPM;
    }
    if (speed_reference > UI_SPEED_LIMIT_RPM) {
        speed_reference = UI_SPEED_LIMIT_RPM;
    }
    lv_chart_set_next_value(
        s_speed_chart, s_speed_measured_series, speed_value);
    if (speed_reference != s_speed_chart_reference_display) {
        s_speed_chart_reference_display = (int16_t)speed_reference;
        lv_chart_set_all_values(
            s_speed_chart, s_speed_reference_series, speed_reference);
    }
    lv_label_set_text_fmt(
        s_speed_chart_value_label,
        "REF %d RPM   SPEED %d RPM",
        snapshot->reference_speed_rpm,
        snapshot->measured_speed_rpm);

    const int32_t new_scale = ui_select_current_scale(
        snapshot->iq_ma, snapshot->iq_reference_ma);
    if (new_scale != s_current_chart_scale_ma) {
        s_current_chart_scale_ma = new_scale;
        lv_chart_set_axis_range(
            s_current_chart,
            LV_CHART_AXIS_PRIMARY_Y,
            -new_scale,
            new_scale);
        lv_label_set_text_fmt(
            s_current_chart_top_label, "%ld", (long)new_scale);
        lv_label_set_text(
            s_current_chart_mid_label, "0");
        lv_label_set_text_fmt(
            s_current_chart_bottom_label, "-%ld", (long)new_scale);
    }
    lv_chart_set_next_value(
        s_current_chart,
        s_current_measured_series,
        snapshot->iq_ma);
    if (snapshot->iq_reference_ma !=
        s_current_chart_reference_ma) {
        s_current_chart_reference_ma = snapshot->iq_reference_ma;
        lv_chart_set_all_values(
            s_current_chart,
            s_current_reference_series,
            snapshot->iq_reference_ma);
    }
    lv_label_set_text_fmt(
        s_current_chart_value_label,
        "REF %d mA   Iq %d mA",
        snapshot->iq_reference_ma,
        snapshot->iq_ma);
}

static void ui_handle_keys(void)
{
    const uint8_t raw = board_keys_read();
    if (raw != s_key_candidate) {
        s_key_candidate = raw;
        s_key_debounce_count = 1U;
        return;
    }
    if (s_key_candidate == s_key_stable) {
        s_key_debounce_count = 0U;
        return;
    }
    if (s_key_debounce_count < 2U) {
        s_key_debounce_count++;
        if (s_key_debounce_count < 2U) {
            return;
        }
    }

    const uint8_t pressed =
        s_key_candidate & (uint8_t)~s_key_stable;
    s_key_stable = s_key_candidate;

    if ((pressed & BOARD_KEY_K0) != 0U) {
        /* K0 is an unconditional escape to the scrollable page selector. */
        ui_show_page(UI_PAGE_HOME);
    } else if ((pressed & BOARD_KEY_K1) != 0U) {
        ui_animate_to_page(
            (ui_page_t)(
                (s_current_page + UI_PAGE_COUNT - 1U) %
                UI_PAGE_COUNT),
            false);
    } else if ((pressed & BOARD_KEY_K2) != 0U) {
        ui_animate_to_page(
            (ui_page_t)((s_current_page + 1U) % UI_PAGE_COUNT),
            true);
    }
}

static void ui_update_motor_data(void)
{
    motor_link_snapshot_t snapshot;
    motor_link_get_snapshot(&snapshot);

    if (s_speed_command_pending &&
        snapshot.mode == MOTOR_LINK_MODE_SPEED &&
        snapshot.reference_speed_rpm == s_pending_speed_rpm) {
        s_speed_command_pending = false;
    } else if (s_speed_command_pending &&
               snapshot.link_active &&
               snapshot.motor_running &&
               snapshot.mode == MOTOR_LINK_MODE_SPEED &&
               lv_tick_elaps(s_speed_command_tick) > 120U) {
        motor_link_set_speed_rpm(s_pending_speed_rpm);
        s_speed_command_tick = lv_tick_get();
    }

    if (s_position_command_pending &&
        snapshot.mode == MOTOR_LINK_MODE_POSITION &&
        snapshot.target_position_cdeg ==
            (uint16_t)(s_pending_position_cdeg % 36000U)) {
        s_position_command_pending = false;
    } else if (s_position_command_pending &&
               snapshot.link_active &&
               snapshot.motor_running &&
               snapshot.mode == MOTOR_LINK_MODE_POSITION &&
               lv_tick_elaps(s_position_command_tick) > 120U) {
        motor_link_set_position_cdeg(s_pending_position_cdeg);
        s_position_command_tick = lv_tick_get();
    }

    if (!s_have_previous_snapshot ||
        snapshot.link_active != s_previous_snapshot.link_active ||
        snapshot.transport != s_previous_snapshot.transport ||
        snapshot.uart_connected != s_previous_snapshot.uart_connected ||
        snapshot.uart_link_active != s_previous_snapshot.uart_link_active ||
        snapshot.can_connected != s_previous_snapshot.can_connected ||
        snapshot.can_link_active != s_previous_snapshot.can_link_active ||
        snapshot.bus_off != s_previous_snapshot.bus_off ||
        snapshot.transceiver_fault != s_previous_snapshot.transceiver_fault ||
        snapshot.motor_running != s_previous_snapshot.motor_running ||
        snapshot.motor_fault != s_previous_snapshot.motor_fault) {
        lv_obj_set_style_text_color(
            s_uart_status_label,
            lv_color_hex(snapshot.uart_link_active
                ? UI_COLOR_GREEN : UI_COLOR_RED),
            LV_PART_MAIN);
        lv_obj_set_style_text_color(
            s_can_status_label,
            lv_color_hex(snapshot.can_link_active
                ? UI_COLOR_GREEN : UI_COLOR_RED),
            LV_PART_MAIN);
        const char *transport_name =
            snapshot.transport == MOTOR_LINK_UART ? "USART"
                : (snapshot.transport == MOTOR_LINK_CAN ? "CAN" : "NONE");
        lv_label_set_text_fmt(
            s_home_transport_label,
            "LINK %s%s",
            transport_name,
            snapshot.transport == MOTOR_LINK_NONE ? ""
                : (snapshot.link_active ? " ONLINE" : " OFFLINE"));
        lv_obj_set_style_text_color(
            s_home_transport_label,
            lv_color_hex(snapshot.link_active
                ? UI_COLOR_GREEN : UI_COLOR_RED),
            LV_PART_MAIN);
        lv_label_set_text(
            s_home_run_label,
            snapshot.motor_fault ? "STATE FAULT"
                : (snapshot.motor_running ? "STATE RUNNING" : "STATE STOPPED"));
        lv_obj_set_style_text_color(
            s_home_run_label,
            lv_color_hex(snapshot.motor_fault ? UI_COLOR_RED
                : (snapshot.motor_running ? UI_COLOR_GREEN : UI_COLOR_YELLOW)),
            LV_PART_MAIN);
        lv_label_set_text(
            s_home_state_label,
            snapshot.transport == MOTOR_LINK_NONE ? "SELECT SERIAL OR CAN"
                : (snapshot.transceiver_fault ? "CHECK VIO TXD RXD"
                : (snapshot.bus_off ? "CHECK CAN WIRING"
                : (snapshot.reconnecting ? "SERIAL RECONNECTING"
                : (!snapshot.link_active ? "WAITING FOR TELEMETRY"
                : (snapshot.motor_running ? "MOTOR RUNNING" : "MOTOR READY"))))));
    }

    if (!s_have_previous_snapshot ||
        snapshot.received_frames != s_previous_snapshot.received_frames ||
        snapshot.transmitted_frames != s_previous_snapshot.transmitted_frames ||
        snapshot.faults != s_previous_snapshot.faults ||
        snapshot.command_rejected != s_previous_snapshot.command_rejected ||
        snapshot.transmit_errors != s_previous_snapshot.transmit_errors) {
        lv_label_set_text_fmt(
            s_link_diag_label,
            "%s RX %lu   TX %lu   ERR %lu\nFAULT 0x%04X%s",
            snapshot.transport == MOTOR_LINK_UART ? "UART" : "CAN",
            (unsigned long)snapshot.received_frames,
            (unsigned long)snapshot.transmitted_frames,
            (unsigned long)snapshot.transmit_errors,
            snapshot.faults,
            snapshot.command_rejected ? "  CMD REJECTED" : "");
        lv_label_set_text_fmt(
            s_home_fault_label,
            "FAULT 0x%04X%s",
            snapshot.faults,
            snapshot.command_rejected ? " CMD" : "");
        lv_obj_set_style_text_color(
            s_home_fault_label,
            lv_color_hex((snapshot.faults != 0U || snapshot.command_rejected)
                ? UI_COLOR_RED : UI_COLOR_GREEN),
            LV_PART_MAIN);
    }

    if (!s_have_previous_snapshot ||
        snapshot.measured_speed_rpm !=
            s_previous_snapshot.measured_speed_rpm) {
        lv_label_set_text_fmt(
            s_speed_actual_label,
            "%d RPM",
            snapshot.measured_speed_rpm);
        lv_label_set_text_fmt(
            s_home_speed_measured_label,
            "%d RPM",
            snapshot.measured_speed_rpm);
    }

    if (!s_have_previous_snapshot ||
        snapshot.reference_speed_rpm !=
            s_previous_snapshot.reference_speed_rpm) {
        lv_label_set_text_fmt(
            s_speed_reference_label,
            "%d RPM",
            snapshot.reference_speed_rpm);
        lv_label_set_text_fmt(
            s_home_speed_target_label,
            "TARGET %d RPM",
            snapshot.reference_speed_rpm);
        if (!s_speed_dragging && !s_speed_command_pending) {
            lv_slider_set_value(
                s_speed_slider,
                snapshot.reference_speed_rpm,
                LV_ANIM_OFF);
            lv_label_set_text_fmt(
                s_speed_slider_value,
                "%d RPM",
                snapshot.reference_speed_rpm);
        }
    }

    if (!s_have_previous_snapshot ||
        snapshot.current_position_cdeg !=
            s_previous_snapshot.current_position_cdeg) {
        const uint16_t value = snapshot.current_position_cdeg;
        lv_label_set_text_fmt(
            s_position_current_label,
            "%3u.%02u deg",
            value / 100U,
            value % 100U);
        lv_label_set_text_fmt(
            s_home_position_current_label,
            "%u.%02u deg",
            value / 100U,
            value % 100U);
    }

    if ((!s_have_previous_snapshot ||
         snapshot.target_position_cdeg !=
             s_previous_snapshot.target_position_cdeg) &&
        !s_position_dragging &&
        !s_position_command_pending) {
        const uint16_t value = snapshot.target_position_cdeg;
        lv_slider_set_value(s_position_slider, value, LV_ANIM_OFF);
        lv_label_set_text_fmt(
            s_position_target_label,
            "%3u.%02u deg",
            value / 100U,
            value % 100U);
        lv_label_set_text_fmt(
            s_home_position_target_label,
            "TARGET %u.%02u deg",
            value / 100U,
            value % 100U);
    }

    if (!s_have_previous_snapshot ||
        snapshot.iq_ma != s_previous_snapshot.iq_ma ||
        snapshot.id_ma != s_previous_snapshot.id_ma ||
        snapshot.iq_reference_ma != s_previous_snapshot.iq_reference_ma ||
        snapshot.id_reference_ma != s_previous_snapshot.id_reference_ma) {
        lv_label_set_text_fmt(
            s_electrical_label,
            "Iq %5dmA  Id %5dmA\nIq* %4dmA  Id* %4dmA",
            snapshot.iq_ma,
            snapshot.id_ma,
            snapshot.iq_reference_ma,
            snapshot.id_reference_ma);
        lv_label_set_text_fmt(
            s_home_current_label,
            "Iq %5d mA\nId %5d mA",
            snapshot.iq_ma,
            snapshot.id_ma);
        lv_label_set_text_fmt(
            s_home_current_reference_label,
            "Iq*%5d mA\nId*%5d mA",
            snapshot.iq_reference_ma,
            snapshot.id_reference_ma);
    }

    if (!s_have_previous_snapshot ||
        snapshot.uq_mv != s_previous_snapshot.uq_mv ||
        snapshot.ud_mv != s_previous_snapshot.ud_mv) {
        lv_label_set_text_fmt(
            s_home_voltage_label,
            "Uq %5d mV\nUd %5d mV",
            snapshot.uq_mv,
            snapshot.ud_mv);
    }

    if (!s_have_previous_snapshot ||
        snapshot.mode != s_previous_snapshot.mode) {
        const bool speed_mode =
            snapshot.mode == MOTOR_LINK_MODE_SPEED;
        lv_label_set_text(
            s_speed_mode_button_label,
            speed_mode ? "SPEED ACTIVE" : "ENABLE SPEED");
        lv_label_set_text(
            s_position_mode_button_label,
            speed_mode ? "ENABLE POS" : "POS ACTIVE");
        lv_label_set_text(
            s_home_mode_label,
            speed_mode ? "MODE SPEED" : "MODE POSITION");
        lv_obj_set_style_text_color(
            s_home_mode_label,
            lv_color_hex(speed_mode ? UI_COLOR_CYAN : UI_COLOR_GREEN),
            LV_PART_MAIN);
    }

    ui_update_charts(&snapshot);
    s_previous_snapshot = snapshot;
    s_have_previous_snapshot = true;
}

static void ui_update_wifi_data(void)
{
    wifi_manager_snapshot_t snapshot;
    wifi_manager_get_snapshot(&snapshot);

    if (snapshot.scan_generation != s_wifi_scan_generation) {
        char options[WIFI_MANAGER_MAX_APS * 48U];
        size_t used = 0U;
        options[0] = '\0';
        s_wifi_network_count = snapshot.ap_count;
        if (s_wifi_network_count > WIFI_MANAGER_MAX_APS) {
            s_wifi_network_count = WIFI_MANAGER_MAX_APS;
        }

        for (uint16_t i = 0U; i < s_wifi_network_count; i++) {
            strlcpy(
                s_wifi_network_ssids[i],
                snapshot.aps[i].ssid,
                sizeof(s_wifi_network_ssids[i]));
            s_wifi_network_secured[i] = snapshot.aps[i].secured;
            const int written = snprintf(
                options + used,
                sizeof(options) - used,
                "%s%s  %d dBm%s",
                i == 0U ? "" : "\n",
                snapshot.aps[i].ssid,
                snapshot.aps[i].rssi,
                snapshot.aps[i].secured ? "  *" : "");
            if (written < 0 || (size_t)written >= sizeof(options) - used) {
                break;
            }
            used += (size_t)written;
        }

        if (s_wifi_network_count == 0U) {
            lv_dropdown_set_options(
                s_wifi_network_dropdown,
                "No networks - tap SCAN");
        } else {
            lv_dropdown_set_options(s_wifi_network_dropdown, options);
            lv_dropdown_set_selected(s_wifi_network_dropdown, 0U);
        }
        s_wifi_scan_generation = snapshot.scan_generation;
        ui_wifi_update_selected_detail();
    }

    if (snapshot.revision == s_wifi_revision) {
        return;
    }

    lv_obj_set_style_text_color(
        s_wifi_status_label,
        lv_color_hex(snapshot.connected ? UI_COLOR_GREEN : UI_COLOR_RED),
        LV_PART_MAIN);
    if (snapshot.connected) {
        lv_label_set_text_fmt(
            s_wifi_page_state_label,
            "CONNECTED: %s\nIP: %s",
            snapshot.ssid,
            snapshot.ip_address);
    } else {
        lv_label_set_text(s_wifi_page_state_label, snapshot.status);
    }
    s_wifi_revision = snapshot.revision;
}

static void ui_update_mqtt_data(void)
{
    mqtt_manager_snapshot_t snapshot;
    mqtt_manager_get_snapshot(&snapshot);
    if (snapshot.revision == s_mqtt_revision) {
        return;
    }

    lv_obj_set_style_text_color(
        s_mqtt_status_label,
        lv_color_hex(snapshot.connected ? UI_COLOR_GREEN : UI_COLOR_RED),
        LV_PART_MAIN);
    if (snapshot.connected) {
        lv_label_set_text_fmt(
            s_mqtt_page_state_label,
            "CONNECTED  TX %lu  RX %lu",
            (unsigned long)snapshot.transmitted_messages,
            (unsigned long)snapshot.received_messages);
    } else {
        lv_label_set_text(s_mqtt_page_state_label, snapshot.status);
    }

    if (snapshot.received_messages > 0U) {
        lv_label_set_text_fmt(
            s_mqtt_rx_label,
            "RX %s\n%s",
            snapshot.last_topic,
            snapshot.last_payload);
    }
    s_mqtt_revision = snapshot.revision;
}

static void ui_timer_callback(lv_timer_t *timer)
{
    (void)timer;
    ui_update_motor_data();
    ui_update_wifi_data();
    ui_update_mqtt_data();
}

static void ui_key_timer_callback(lv_timer_t *timer)
{
    (void)timer;
    ui_handle_keys();
}

void motor_ui_create(lv_display_t *display)
{
    if (display == NULL) {
        return;
    }

    lv_obj_t *screen = lv_screen_active();
    /*
     * The LCD controller can retain the previous frame across an ESP reset.
     * Remove every old LVGL child and force an opaque, full-screen redraw
     * while the backlight is still off so no legacy welcome page can remain.
     */
    lv_obj_clean(screen);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(
        screen, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    s_page_label = ui_create_label(
        screen, s_page_names[0], UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_page_label, LV_ALIGN_TOP_LEFT, 6, 5);

    s_mqtt_status_label = ui_create_label(
        screen, "MQTT", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align(s_mqtt_status_label, LV_ALIGN_TOP_RIGHT, -6, 5);
    s_wifi_status_label = ui_create_label(
        screen, "WI-FI", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align_to(
        s_wifi_status_label, s_mqtt_status_label,
        LV_ALIGN_OUT_LEFT_MID, -10, 0);
    s_can_status_label = ui_create_label(
        screen, "CAN", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align_to(
        s_can_status_label, s_wifi_status_label,
        LV_ALIGN_OUT_LEFT_MID, -10, 0);
    s_uart_status_label = ui_create_label(
        screen, "USART", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align_to(
        s_uart_status_label, s_can_status_label,
        LV_ALIGN_OUT_LEFT_MID, -12, 0);

    /*
     * Keep page animations inside one opaque clipping viewport.  Moving page
     * objects directly on the root screen can expose stale scan lines above
     * and below the content area when LVGL uses a partial display buffer.
     */
    s_page_viewport = lv_obj_create(screen);
    lv_obj_set_size(s_page_viewport, 320, UI_PAGE_HEIGHT);
    lv_obj_set_pos(s_page_viewport, 0, UI_VIEWPORT_TOP);
    lv_obj_set_style_bg_color(
        s_page_viewport, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_page_viewport, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_page_viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_width(s_page_viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_page_viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_page_viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_page_viewport, 0, LV_PART_MAIN);
    lv_obj_remove_flag(s_page_viewport, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_page_viewport, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        s_pages[i] = lv_obj_create(s_page_viewport);
        lv_obj_set_size(s_pages[i], 320, UI_PAGE_HEIGHT);
        lv_obj_set_pos(s_pages[i], 0, UI_PAGE_TOP);
        lv_obj_set_style_bg_color(
            s_pages[i], lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(
            s_pages[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(s_pages[i], 0, LV_PART_MAIN);
        lv_obj_set_style_outline_width(s_pages[i], 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(s_pages[i], 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(
            s_pages[i], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(
            s_pages[i], 0, LV_PART_MAIN);
        lv_obj_remove_flag(
            s_pages[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    ui_create_navigation_page(s_pages[UI_PAGE_HOME]);
    ui_create_feedback_page(s_pages[UI_PAGE_FEEDBACK]);
    ui_create_uart_page(s_pages[UI_PAGE_UART]);
    ui_create_can_page(s_pages[UI_PAGE_CAN]);
    ui_create_wifi_page(s_pages[UI_PAGE_WIFI]);
    ui_create_mqtt_page(s_pages[UI_PAGE_MQTT]);
    ui_create_speed_page(s_pages[UI_PAGE_SPEED]);
    ui_create_position_page(s_pages[UI_PAGE_POSITION]);
    ui_create_speed_chart_page(s_pages[UI_PAGE_SPEED_CHART]);
    ui_create_current_chart_page(s_pages[UI_PAGE_CURRENT_CHART]);

    ui_show_page(UI_PAGE_HOME);
    lv_obj_move_foreground(s_page_label);
    lv_obj_move_foreground(s_uart_status_label);
    lv_obj_move_foreground(s_can_status_label);
    lv_obj_move_foreground(s_wifi_status_label);
    lv_obj_move_foreground(s_mqtt_status_label);
    lv_timer_create(ui_timer_callback, 50, NULL);
    lv_timer_create(ui_key_timer_callback, 20, NULL);
    ui_update_motor_data();
    ui_update_wifi_data();
    ui_update_mqtt_data();
    lv_obj_update_layout(screen);
    lv_obj_invalidate(screen);
    /* The LVGL port task performs the invalidated full-screen refresh. */
}

void motor_ui_attach_input(lv_indev_t *indev)
{
    if (indev == NULL) {
        return;
    }

    lv_indev_add_event_cb(
        indev, ui_input_event, LV_EVENT_PRESSED, indev);
    lv_indev_add_event_cb(
        indev, ui_input_event, LV_EVENT_RELEASED, indev);
}
