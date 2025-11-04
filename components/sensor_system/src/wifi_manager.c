/*
 * wifi_manager.c - WiFi Manager Implementation
 *
 * This file implements the WiFi management system for the ESP32 sensor system.
 * Provides WiFi station mode connection management with automatic reconnection.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>
#include <nvs_flash.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <lwip/inet.h>

#include "wifi_manager.h"
#include "config.h"
#include "watchdog.h"
#include "error_recovery.h"
#include "types.h"

// Forward declarations to avoid circular dependency
bool mqtt_manager_is_connected(void);
esp_err_t mqtt_manager_init(void);
esp_err_t mqtt_manager_connect(void);

static const char *TAG = "WIFI_MANAGER";

// WiFi event bits
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1

// Static variables
static bool wifi_manager_initialized = false;
static EventGroupHandle_t wifi_event_group = NULL;
static TaskHandle_t wifi_manager_task_handle = NULL;
static WiFiStatus_t current_status = WIFI_STATUS_DISCONNECTED;
static WiFiStats_t stats = {0};
static WiFiConfig_t wifi_config = {0};
static uint32_t connection_start_time = 0;

// Forward declarations
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);

esp_err_t wifi_manager_init(void) {
    if (wifi_manager_initialized) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi station netif
    esp_netif_create_default_wifi_sta();

    // Set hostname (must be done after esp_netif_create_default_wifi_sta() but before esp_wifi_start())
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif != NULL) {
        esp_err_t err = esp_netif_set_hostname(sta_netif, WIFI_HOSTNAME);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Hostname set to: %s", WIFI_HOSTNAME);
        } else {
            ESP_LOGE(TAG, "Failed to set hostname: %s", esp_err_to_name(err));
            // Continue anyway - hostname is not critical
        }
    } else {
        ESP_LOGE(TAG, "Failed to get STA netif handle for hostname");
        // Continue anyway - hostname is not critical
    }

    // WiFi configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &ip_event_handler, NULL));

    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Create event group
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_ERR_NO_MEM;
    }

    // Initialize statistics
    memset(&stats, 0, sizeof(WiFiStats_t));
    memset(&wifi_config, 0, sizeof(WiFiConfig_t));
    current_status = WIFI_STATUS_DISCONNECTED;

    wifi_manager_initialized = true;
    ESP_LOGI(TAG, "WiFi manager initialized");
    return ESP_OK;
}

BaseType_t wifi_manager_start_task(void) {
    if (!wifi_manager_initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return pdFAIL;
    }

    if (wifi_manager_task_handle != NULL) {
        ESP_LOGW(TAG, "WiFi manager task already running");
        return pdPASS;
    }

    BaseType_t result = xTaskCreate(
        vWiFiManagerTask,
        "WiFiManager",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_IDLE,  // Low priority - runs when system is idle
        &wifi_manager_task_handle
    );

    if (result == pdPASS) {
        ESP_LOGI(TAG, "WiFi manager task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(wifi_manager_task_handle, "WiFiManager", 30000);
    } else {
        ESP_LOGE(TAG, "Failed to create WiFi manager task");
    }

    return result;
}

esp_err_t wifi_manager_connect(const char* ssid, const char* password) {
    if (!wifi_manager_initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    // Store configuration
    strncpy(wifi_config.ssid, ssid, WIFI_MAX_SSID_LEN - 1);
    wifi_config.ssid[WIFI_MAX_SSID_LEN - 1] = '\0';
    
    if (password != NULL) {
        strncpy(wifi_config.password, password, WIFI_MAX_PASSWORD_LEN - 1);
        wifi_config.password[WIFI_MAX_PASSWORD_LEN - 1] = '\0';
    } else {
        wifi_config.password[0] = '\0';
    }

    wifi_config.timeout_ms = WIFI_CONNECT_TIMEOUT_MS;
    wifi_config.max_retry = WIFI_MAX_RETRY_COUNT;

    // Configure WiFi
    wifi_config_t wifi_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    strncpy((char*)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    if (password != NULL && strlen(password) > 0) {
        strncpy((char*)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    // Start WiFi
    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "wifi_manager_connect", NULL);
        return ret;
    }

    // Clear event bits
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    // Connect to WiFi
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate WiFi connection: %s", esp_err_to_name(ret));
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "wifi_manager_connect", NULL);
        return ret;
    }

    current_status = WIFI_STATUS_CONNECTING;
    stats.connectAttempts++;
    connection_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "Initiated WiFi connection to: %s", ssid);
    return ESP_OK;
}

esp_err_t wifi_manager_disconnect(void) {
    if (!wifi_manager_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_wifi_disconnect();
    if (ret == ESP_OK) {
        current_status = WIFI_STATUS_DISCONNECTED;
        stats.disconnections++;
        ESP_LOGI(TAG, "WiFi disconnected");
    }
    return ret;
}

WiFiStatus_t wifi_manager_get_status(void) {
    return current_status;
}

bool wifi_manager_is_connected(void) {
    return (current_status == WIFI_STATUS_CONNECTED);
}

void wifi_manager_get_stats(WiFiStats_t* stats_out) {
    if (stats_out != NULL) {
        *stats_out = stats;
        
        // Update uptime if connected
        if (current_status == WIFI_STATUS_CONNECTED && connection_start_time > 0) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            stats_out->uptime = (current_time - connection_start_time) / 1000;
        }
        
        // Update RSSI if connected
        if (current_status == WIFI_STATUS_CONNECTED) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                stats_out->rssi = ap_info.rssi;
            }
        }
    }
}

void wifi_manager_reset_stats(void) {
    memset(&stats, 0, sizeof(WiFiStats_t));
}

int8_t wifi_manager_get_rssi(void) {
    if (current_status != WIFI_STATUS_CONNECTED) {
        return 0;
    }

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;
}

esp_err_t wifi_manager_get_ip_address(char* ip_str, size_t len) {
    if (ip_str == NULL || len < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current_status != WIFI_STATUS_CONNECTED) {
        strncpy(ip_str, "Not connected", len - 1);
        ip_str[len - 1] = '\0';
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected");
        current_status = WIFI_STATUS_DISCONNECTED;
        stats.disconnections++;
        
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        
        // Report error for monitoring
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "wifi_event_handler", NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP");
        current_status = WIFI_STATUS_CONNECTING;
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        
        current_status = WIFI_STATUS_CONNECTED;
        stats.successfulConnections++;
        connection_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Initialize and connect MQTT now that WiFi is connected
        if (!mqtt_manager_is_connected()) {
            esp_err_t ret = mqtt_manager_init();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "MQTT manager initialized after WiFi connection");
                ret = mqtt_manager_connect();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "MQTT connection initiated after WiFi connection");
                } else {
                    ESP_LOGW(TAG, "Failed to connect MQTT after WiFi connection: %s", 
                             esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "Failed to initialize MQTT after WiFi connection: %s",
                         esp_err_to_name(ret));
            }
        }
    }
}

void vWiFiManagerTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "WiFi manager task started");

    // Register with watchdog
    watchdog_register_current_task("WiFiManager", 30000);

    uint32_t reconnect_delay = WIFI_RECONNECT_DELAY_MS;
    const TickType_t xMonitorInterval = pdMS_TO_TICKS(5000); // Check every 5 seconds

    for (;;) {
        // Check connection status
        if (current_status == WIFI_STATUS_DISCONNECTED && 
            strlen(wifi_config.ssid) > 0) {
            // Attempt reconnection
            current_status = WIFI_STATUS_RECONNECTING;
            stats.reconnectAttempts++;
            
            ESP_LOGI(TAG, "Attempting WiFi reconnection (attempt %lu)...", 
                     stats.reconnectAttempts);
            
            esp_err_t ret = esp_wifi_connect();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Reconnection attempt failed: %s", esp_err_to_name(ret));
                current_status = WIFI_STATUS_ERROR;
                error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, 
                            "vWiFiManagerTask", NULL);
            } else {
                // Wait for connection result
                EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                    WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                    pdFALSE,
                    pdFALSE,
                    pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

                if (bits & WIFI_CONNECTED_BIT) {
                    ESP_LOGI(TAG, "WiFi reconnection successful");
                    reconnect_delay = WIFI_RECONNECT_DELAY_MS; // Reset delay
                } else if (bits & WIFI_FAIL_BIT) {
                    ESP_LOGW(TAG, "WiFi reconnection failed");
                    // Exponential backoff
                    reconnect_delay = (reconnect_delay * 2 < WIFI_MAX_RECONNECT_DELAY_MS) ?
                                      reconnect_delay * 2 : WIFI_MAX_RECONNECT_DELAY_MS;
                    ESP_LOGI(TAG, "Next reconnect attempt in %lu ms", reconnect_delay);
                }
            }
        }

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        // Wait before next check
        vTaskDelay(xMonitorInterval);
    }
}

