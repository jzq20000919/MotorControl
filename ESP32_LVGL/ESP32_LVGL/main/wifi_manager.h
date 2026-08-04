#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define WIFI_MANAGER_MAX_APS 12U
#define WIFI_MANAGER_SSID_MAX_LEN 32U

typedef struct
{
    char ssid[WIFI_MANAGER_SSID_MAX_LEN + 1U];
    int8_t rssi;
    bool secured;
} wifi_manager_ap_t;

typedef struct
{
    bool initialized;
    bool scanning;
    bool connecting;
    bool connected;
    uint16_t ap_count;
    uint32_t revision;
    uint32_t scan_generation;
    char ssid[WIFI_MANAGER_SSID_MAX_LEN + 1U];
    char ip_address[16];
    char status[64];
    wifi_manager_ap_t aps[WIFI_MANAGER_MAX_APS];
} wifi_manager_snapshot_t;

/** @brief 初始化 NVS、网络接口和 Wi-Fi STA 驱动。 */
esp_err_t wifi_manager_init(void);
/** @brief 启动非阻塞无线接入点扫描。 */
esp_err_t wifi_manager_scan_async(void);
/** @brief 使用 @p password 开始连接 @p ssid；连接完成为异步过程。 */
esp_err_t wifi_manager_connect(
    const char *ssid,
    const char *password);
/** @brief 断开当前 STA 连接并取消重连任务。 */
esp_err_t wifi_manager_disconnect(void);
/** @brief 复制最新线程安全 Wi-Fi 状态和扫描结果列表。 */
void wifi_manager_get_snapshot(wifi_manager_snapshot_t *snapshot);

#endif /* WIFI_MANAGER_H */
