#include "wifi_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

static const char *TAG = "WIFI_MANAGER";

static SemaphoreHandle_t s_lock;
static wifi_manager_snapshot_t s_snapshot;
static esp_netif_t *s_station_netif;
static esp_event_handler_instance_t s_wifi_event_instance;
static esp_event_handler_instance_t s_ip_event_instance;
static bool s_ignore_next_disconnect;
static bool s_user_disconnect;

static void wifi_manager_lock(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void wifi_manager_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static void wifi_manager_set_status_locked(const char *status)
{
    strlcpy(s_snapshot.status, status, sizeof(s_snapshot.status));
    s_snapshot.revision++;
}

static void wifi_manager_set_error(esp_err_t error, const char *operation)
{
    wifi_manager_lock();
    snprintf(
        s_snapshot.status,
        sizeof(s_snapshot.status),
        "%s: %s",
        operation,
        esp_err_to_name(error));
    s_snapshot.revision++;
    wifi_manager_unlock();
}

static int wifi_manager_compare_ap(const void *left, const void *right)
{
    const wifi_ap_record_t *a = left;
    const wifi_ap_record_t *b = right;
    return (int)b->rssi - (int)a->rssi;
}

static void wifi_manager_store_scan_results(void)
{
    wifi_ap_record_t records[WIFI_MANAGER_MAX_APS];
    uint16_t record_count = WIFI_MANAGER_MAX_APS;
    uint16_t total_count = 0U;

    esp_err_t result = esp_wifi_scan_get_ap_num(&total_count);
    if (result == ESP_OK) {
        result = esp_wifi_scan_get_ap_records(&record_count, records);
    }
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_snapshot.scanning = false;
        snprintf(
            s_snapshot.status,
            sizeof(s_snapshot.status),
            "Scan failed: %s",
            esp_err_to_name(result));
        s_snapshot.revision++;
        wifi_manager_unlock();
        return;
    }

    qsort(records, record_count, sizeof(records[0]), wifi_manager_compare_ap);

    wifi_manager_lock();
    memset(s_snapshot.aps, 0, sizeof(s_snapshot.aps));
    s_snapshot.ap_count = 0U;
    for (uint16_t i = 0U;
         i < record_count && s_snapshot.ap_count < WIFI_MANAGER_MAX_APS;
         i++) {
        if (records[i].ssid[0] == '\0') {
            continue;
        }

        bool duplicate = false;
        for (uint16_t j = 0U; j < s_snapshot.ap_count; j++) {
            if (strncmp(
                    s_snapshot.aps[j].ssid,
                    (const char *)records[i].ssid,
                    WIFI_MANAGER_SSID_MAX_LEN) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        wifi_manager_ap_t *destination =
            &s_snapshot.aps[s_snapshot.ap_count++];
        strlcpy(
            destination->ssid,
            (const char *)records[i].ssid,
            sizeof(destination->ssid));
        destination->rssi = records[i].rssi;
        destination->secured = records[i].authmode != WIFI_AUTH_OPEN;
    }

    s_snapshot.scanning = false;
    s_snapshot.scan_generation++;
    if (s_snapshot.connected) {
        snprintf(
            s_snapshot.status,
            sizeof(s_snapshot.status),
            "Connected - found %u networks",
            (unsigned int)s_snapshot.ap_count);
    } else {
        snprintf(
            s_snapshot.status,
            sizeof(s_snapshot.status),
            "Found %u networks",
            (unsigned int)s_snapshot.ap_count);
    }
    s_snapshot.revision++;
    wifi_manager_unlock();

    ESP_LOGI(
        TAG,
        "Wi-Fi scan complete: %u visible, %u retained",
        (unsigned int)total_count,
        (unsigned int)record_count);
}

static void wifi_manager_event_handler(
    void *argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)argument;

    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_SCAN_DONE:
            wifi_manager_store_scan_results();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            wifi_manager_lock();
            s_snapshot.connecting = true;
            wifi_manager_set_status_locked("Connected to AP - waiting for IP");
            wifi_manager_unlock();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            const wifi_event_sta_disconnected_t *event = event_data;
            wifi_manager_lock();
            s_snapshot.connected = false;
            s_snapshot.ip_address[0] = '\0';
            if (s_ignore_next_disconnect) {
                s_ignore_next_disconnect = false;
            } else if (s_user_disconnect) {
                s_snapshot.connecting = false;
                s_user_disconnect = false;
                wifi_manager_set_status_locked("Disconnected");
            } else {
                s_snapshot.connecting = false;
                snprintf(
                    s_snapshot.status,
                    sizeof(s_snapshot.status),
                    "Connection failed (reason %u)",
                    event != NULL ? (unsigned int)event->reason : 0U);
                s_snapshot.revision++;
            }
            wifi_manager_unlock();
            break;
        }

        default:
            break;
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        wifi_manager_lock();
        s_snapshot.connected = true;
        s_snapshot.connecting = false;
        if (event != NULL) {
            esp_ip4addr_ntoa(
                &event->ip_info.ip,
                s_snapshot.ip_address,
                sizeof(s_snapshot.ip_address));
        }
        wifi_manager_set_status_locked("Connected");
        wifi_manager_unlock();
        ESP_LOGI(
            TAG,
            "Connected to %s with IP %s",
            s_snapshot.ssid,
            s_snapshot.ip_address);
    }
}

esp_err_t wifi_manager_init(void)
{
    if (s_lock != NULL) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    strlcpy(s_snapshot.status, "Initializing Wi-Fi", sizeof(s_snapshot.status));

    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "NVS init failed");
        return result;
    }

    result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        wifi_manager_set_error(result, "Network init failed");
        return result;
    }

    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        wifi_manager_set_error(result, "Event loop failed");
        return result;
    }

    s_station_netif = esp_netif_create_default_wifi_sta();
    if (s_station_netif == NULL) {
        result = ESP_ERR_NO_MEM;
        wifi_manager_set_error(result, "STA interface failed");
        return result;
    }

    const wifi_init_config_t configuration = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&configuration);
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi init failed");
        return result;
    }

    result = esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_manager_event_handler,
        NULL,
        &s_wifi_event_instance);
    if (result == ESP_OK) {
        result = esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            wifi_manager_event_handler,
            NULL,
            &s_ip_event_instance);
    }
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi event setup failed");
        return result;
    }

    result = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    if (result == ESP_OK) {
        result = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (result == ESP_OK) {
        result = esp_wifi_start();
    }
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi start failed");
        return result;
    }

    wifi_manager_lock();
    s_snapshot.initialized = true;
    wifi_manager_set_status_locked("Ready - tap SCAN");
    wifi_manager_unlock();
    ESP_LOGI(TAG, "Wi-Fi station initialized");
    return ESP_OK;
}

esp_err_t wifi_manager_scan_async(void)
{
    wifi_manager_lock();
    if (!s_snapshot.initialized) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (s_snapshot.scanning) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_snapshot.scanning = true;
    wifi_manager_set_status_locked("Scanning...");
    wifi_manager_unlock();

    const esp_err_t result = esp_wifi_scan_start(NULL, false);
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_snapshot.scanning = false;
        wifi_manager_unlock();
        wifi_manager_set_error(result, "Scan failed");
    }
    return result;
}

esp_err_t wifi_manager_connect(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0' || password == NULL ||
        strlen(ssid) > WIFI_MANAGER_SSID_MAX_LEN ||
        strlen(password) > 63U) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t configuration = {0};
    strlcpy(
        (char *)configuration.sta.ssid,
        ssid,
        sizeof(configuration.sta.ssid));
    strlcpy(
        (char *)configuration.sta.password,
        password,
        sizeof(configuration.sta.password));
    configuration.sta.threshold.authmode = WIFI_AUTH_OPEN;
    configuration.sta.pmf_cfg.capable = true;
    configuration.sta.pmf_cfg.required = false;

    wifi_manager_lock();
    if (!s_snapshot.initialized) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    strlcpy(s_snapshot.ssid, ssid, sizeof(s_snapshot.ssid));
    s_snapshot.ip_address[0] = '\0';
    s_snapshot.connected = false;
    s_snapshot.connecting = true;
    s_user_disconnect = false;
    wifi_manager_set_status_locked("Connecting...");
    wifi_manager_unlock();

    wifi_manager_lock();
    s_ignore_next_disconnect = true;
    wifi_manager_unlock();
    const esp_err_t disconnect_result = esp_wifi_disconnect();
    if (disconnect_result != ESP_OK) {
        wifi_manager_lock();
        s_ignore_next_disconnect = false;
        wifi_manager_unlock();
    }

    esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &configuration);
    if (result == ESP_OK) {
        result = esp_wifi_connect();
    }
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_snapshot.connecting = false;
        wifi_manager_unlock();
        wifi_manager_set_error(result, "Connect failed");
    }
    return result;
}

esp_err_t wifi_manager_disconnect(void)
{
    wifi_manager_lock();
    if (!s_snapshot.initialized) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_user_disconnect = true;
    s_ignore_next_disconnect = false;
    s_snapshot.connecting = false;
    wifi_manager_set_status_locked("Disconnecting...");
    wifi_manager_unlock();

    const esp_err_t result = esp_wifi_disconnect();
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_user_disconnect = false;
        s_snapshot.connected = false;
        s_snapshot.ip_address[0] = '\0';
        wifi_manager_set_status_locked("Disconnected");
        wifi_manager_unlock();
    }
    return result == ESP_ERR_WIFI_NOT_CONNECT ? ESP_OK : result;
}

void wifi_manager_get_snapshot(wifi_manager_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    wifi_manager_lock();
    *snapshot = s_snapshot;
    wifi_manager_unlock();
}
