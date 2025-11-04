/*
 * system_init.c - System Initialization Task Implementation
 *
 * This file implements the system initialization tasks for the ESP32 FreeRTOS
 * sensor system. The system init task performs basic system setup and
 * creates core application tasks.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_chip_info.h>
#include <esp_idf_version.h>
#include <string.h>
#include <stdbool.h>
#include <driver/gpio.h>

#if CONFIG_SPIRAM
#include <esp_spiram.h>
#endif

#include "system_init.h"
#include "config.h"
#include "types.h"
#include "pins.h"
#include "watchdog.h"
#include "error_recovery.h"
#include "wifi_manager.h"

static const char *TAG = "SYSTEM_INIT";

// Task handles for heartbeat, information, and monitoring tasks
TaskHandle_t xHeartbeatTaskHandle = NULL;
TaskHandle_t xSystemInfoTaskHandle = NULL;
TaskHandle_t xSystemMonitorTaskHandle = NULL;

/*
 * Heartbeat Task
 * Simple task that periodically prints status to verify FreeRTOS is running
 * Also blinks onboard LED for visual indication
 */
void vHeartbeatTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // Register with watchdog
    watchdog_register_current_task("Heartbeat", 6000);

    // Initialize onboard LED
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << PIN_ONBOARD_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));

    // Start with LED off
    ESP_ERROR_CHECK(gpio_set_level(PIN_ONBOARD_LED, 0));

    const TickType_t xDelay = pdMS_TO_TICKS(5000);  // 5 second delay
    static bool led_state = false;  // Track LED state

    for (;;) {
        // Toggle LED state for visual heartbeat
        led_state = !led_state;
        ESP_ERROR_CHECK(gpio_set_level(PIN_ONBOARD_LED, led_state ? 1 : 0));

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        vTaskDelay(xDelay);
    }
}

/*
 * System Information Task
 * Displays comprehensive system information during startup
 */
void vSystemInfoTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // Small delay to ensure logging is ready
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "=== GenericSensorZ System Information ===");
    ESP_LOGI(TAG, "System Info Task: Starting information display...");
    ESP_LOGI(TAG, "Firmware Version: %s v%d.%d",
             FIRMWARE_NAME, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 Chip Revision: %d", chip_info.revision);

    // Flash size can be determined from partition table or chip capabilities
    ESP_LOGI(TAG, "ESP32 Flash: Available");

    ESP_LOGI(TAG, "FreeRTOS Kernel Version: %s", tskKERNEL_VERSION_NUMBER);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);

    // PSRAM information
#if CONFIG_SPIRAM
    size_t psram_size = esp_spiram_get_size();
    ESP_LOGI(TAG, "PSRAM Available: Yes, Size: %d MB", psram_size / (1024 * 1024));
#else
    ESP_LOGI(TAG, "PSRAM Available: No");
#endif

    ESP_LOGI(TAG, "=== System Information Complete ===");

    // Task completes and deletes itself
    vTaskDelete(NULL);
}

/*
 * System Monitoring Task
 * Continuously monitors system health and resource usage
 */
void vSystemMonitorTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // Register with watchdog
    watchdog_register_current_task("SysMonitor", MONITOR_INTERVAL_MS + 1000);

    ESP_LOGI(TAG, "System Monitor Task: Started successfully!");

    // Initialize monitoring variables on first run
    static uint32_t startTime = 0;
    static size_t lastHeapFree = 0;
    static size_t minHeapFree = 0;
    static UBaseType_t lastTaskCount = 0;
    static bool initialized = false;

    if (!initialized) {
        startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        lastHeapFree = esp_get_free_heap_size();
        minHeapFree = esp_get_free_heap_size();
        lastTaskCount = uxTaskGetNumberOfTasks();
        initialized = true;
    }

    const TickType_t xMonitorInterval = pdMS_TO_TICKS(MONITOR_INTERVAL_MS);

    for (;;) {
        // Calculate uptime
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t uptimeMs = currentTime - startTime;
        uint32_t uptimeSeconds = uptimeMs / 1000;
        uint32_t uptimeHours = uptimeSeconds / 3600;
        uint32_t uptimeMinutes = (uptimeSeconds % 3600) / 60;
        uint32_t uptimeSecs = uptimeSeconds % 60;

        // Get current memory stats
        size_t currentHeapFree = esp_get_free_heap_size();

        // Update minimum heap tracking
        if (currentHeapFree < minHeapFree) {
            minHeapFree = currentHeapFree;
        }

        // Get task information
        UBaseType_t currentTaskCount = uxTaskGetNumberOfTasks();

        // Display monitoring information
        ESP_LOGI(TAG, "--- System Monitor ---");
        ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu (HH:MM:SS)",
                 uptimeHours, uptimeMinutes, uptimeSecs);
        ESP_LOGI(TAG, "Free Heap: %zu bytes (Min: %zu bytes)",
                 currentHeapFree, minHeapFree);
        ESP_LOGI(TAG, "Active Tasks: %d", currentTaskCount);

        // Show task count changes
        if (currentTaskCount != lastTaskCount) {
            ESP_LOGW(TAG, "Task count changed from %d to %d",
                     lastTaskCount, currentTaskCount);
            lastTaskCount = currentTaskCount;
        }

        // Show heap usage trend
        if (currentHeapFree != lastHeapFree) {
            int32_t heapChange = (int32_t)currentHeapFree - (int32_t)lastHeapFree;
            ESP_LOGD(TAG, "Heap change: %ld bytes", heapChange);
            lastHeapFree = currentHeapFree;
        }

        // Get system state from error recovery
        SystemState_t state = error_recovery_get_system_state();
        if (state != SYSTEM_STATE_RUNNING) {
            ESP_LOGW(TAG, "System state: %d", state);
        }

        // Get error statistics
        ErrorRecoveryStats_t error_stats;
        error_recovery_get_stats(&error_stats);
        if (error_stats.total_errors > 0) {
            ESP_LOGI(TAG, "Errors: total=%lu, recovered=%lu, critical=%lu",
                     error_stats.total_errors,
                     error_stats.recovered_errors,
                     error_stats.critical_errors);
        }

        // Get watchdog statistics
        WatchdogStats_t wdt_stats;
        watchdog_get_stats(&wdt_stats);
        ESP_LOGI(TAG, "Watchdog: feeds=%lu, timeouts=%lu, tasks=%lu",
                 wdt_stats.totalFeeds,
                 wdt_stats.timeoutCount,
                 wdt_stats.tasksMonitored);

        // Get WiFi status
        WiFiStatus_t wifi_status = wifi_manager_get_status();
        bool wifi_connected = wifi_manager_is_connected();
        
        if (wifi_connected) {
            char ip_str[16];
            int8_t rssi = wifi_manager_get_rssi();
            WiFiStats_t wifi_stats;
            wifi_manager_get_stats(&wifi_stats);
            
            if (wifi_manager_get_ip_address(ip_str, sizeof(ip_str)) == ESP_OK) {
                ESP_LOGI(TAG, "WiFi: Connected | IP: %s | RSSI: %d dBm | Uptime: %lu sec",
                         ip_str, rssi, wifi_stats.uptime);
            } else {
                ESP_LOGI(TAG, "WiFi: Connected | RSSI: %d dBm | Uptime: %lu sec",
                         rssi, wifi_stats.uptime);
            }
        } else {
            const char* status_str = "Unknown";
            switch (wifi_status) {
                case WIFI_STATUS_DISCONNECTED:
                    status_str = "Disconnected";
                    break;
                case WIFI_STATUS_CONNECTING:
                    status_str = "Connecting";
                    break;
                case WIFI_STATUS_RECONNECTING:
                    status_str = "Reconnecting";
                    break;
                case WIFI_STATUS_ERROR:
                    status_str = "Error";
                    break;
                default:
                    status_str = "Unknown";
                    break;
            }
            ESP_LOGI(TAG, "WiFi: %s", status_str);
        }

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        ESP_LOGI(TAG, "--- Monitor Complete ---");

        // Wait for next monitoring interval
        vTaskDelay(xMonitorInterval);
    }
}

/*
 * Info Response Task
 * Provides comprehensive system information
 */
void vInfoResponseTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "=== System Information ===");

    // Basic system info
    ESP_LOGI(TAG, "Firmware: %s v%d.%d", FIRMWARE_NAME,
             FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);

    // ESP32 chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 Chip: Revision %d", chip_info.revision);

    // Flash size can be determined from partition table or chip capabilities
    ESP_LOGI(TAG, "Flash: Available");

    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());

    // Task count and states
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    ESP_LOGI(TAG, "Active Tasks: %d", taskCount);

    // Memory information
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "Free Heap: %zu bytes", free_heap);
    ESP_LOGI(TAG, "Min Free Heap: %zu bytes", min_free_heap);

    // PSRAM information
#if CONFIG_SPIRAM
    size_t psram_size = esp_spiram_get_size();
    size_t psram_free = esp_spiram_get_free_size();
    ESP_LOGI(TAG, "PSRAM: %zu bytes total, %zu bytes free", psram_size, psram_free);
#endif

    // Uptime calculation
    uint32_t uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t uptime_sec = uptime_ms / 1000;
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    uint32_t seconds = uptime_sec % 60;
    ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu", hours, minutes, seconds);

    ESP_LOGI(TAG, "=== End System Information ===");

    // Task completes and deletes itself
    vTaskDelete(NULL);
}
