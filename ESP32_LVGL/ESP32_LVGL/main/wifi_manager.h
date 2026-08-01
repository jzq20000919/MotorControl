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

esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_scan_async(void);
esp_err_t wifi_manager_connect(
    const char *ssid,
    const char *password);
esp_err_t wifi_manager_disconnect(void);
void wifi_manager_get_snapshot(wifi_manager_snapshot_t *snapshot);

#endif /* WIFI_MANAGER_H */
