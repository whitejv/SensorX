// main/main.c - ESP-IDF Application Entry Point

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"         // From sensor_system component
#include "types.h"          // From sensor_system component
#include "system_init.h"    // From sensor_system component
#include "watchdog.h"       // From sensor_system component
#include "error_recovery.h" // From sensor_system component
#include "wifi_manager.h"   // From sensor_system component

static const char *TAG = "MAIN";

void app_main(void) {
    // Initialize ESP_LOG system
    esp_log_level_set("*", ESP_LOG_INFO);  // Set global log level
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Set specific tag level

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "===  Sensor Base ESP-IDF Starting  ===");
    ESP_LOGI(TAG, "======================================");

    // 1. Initialize error recovery FIRST (needed for error handling)
    esp_err_t ret = error_recovery_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize error recovery: %s", esp_err_to_name(ret));
        abort();
    }

    ret = error_recovery_start_task();
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to start error recovery task");
        abort();
    }

    // 2. Initialize watchdog timer
    ret = watchdog_init(WATCHDOG_TIMEOUT_SEC);
    if (ret != ESP_OK) {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_CRITICAL, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to initialize watchdog: %s", esp_err_to_name(ret));
        abort();
    }

    ret = watchdog_start_task();
    if (ret != pdPASS) {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_CRITICAL, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to start watchdog task");
        abort();
    }

    // ============================================================================
    // HARDWARE DEBUGGER SUPPORT: Uncomment the following to disable watchdog
    // when using a hardware debugger. The watchdog will trigger if execution
    // is paused (e.g., breakpoints), so disable it during debugging sessions.
    // ============================================================================
    // watchdog_disable();
    // ESP_LOGW(TAG, "WATCHDOG DISABLED FOR DEBUGGING - DO NOT USE IN PRODUCTION!");
    // ============================================================================

    // 3. Initialize WiFi Manager (non-critical - continue if fails)
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "Failed to initialize WiFi manager: %s", esp_err_to_name(ret));
        // Continue anyway - WiFi is not critical for basic operation
    } else {
        ret = wifi_manager_start_task();
        if (ret != pdPASS) {
            error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to start WiFi manager task");
            // Continue anyway - WiFi is not critical for basic operation
        } else {
            ESP_LOGI(TAG, "WiFi manager initialized and task started");
            
#if WIFI_AUTO_CONNECT
            // Auto-connect to WiFi on startup
            ret = wifi_manager_connect(WIFI_SSID, WIFI_PASSWORD);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initiate WiFi connection: %s", esp_err_to_name(ret));
                error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            } else {
                ESP_LOGI(TAG, "WiFi connection initiated to: %s", WIFI_SSID);
            }
#endif
        }
    }

    // 4. Create the system information task
    BaseType_t xResult = xTaskCreate(
        vSystemInfoTask,              // Task function
        "SystemInfo",                 // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_FIXED_FREQ,     // Higher priority (3) to run first
        &xSystemInfoTaskHandle        // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System information task created successfully");
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_ERROR, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create system information task");
    }

    // 5. Create the system monitoring task
    xResult = xTaskCreate(
        vSystemMonitorTask,           // Task function
        "SysMonitor",                 // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_BACKGROUND,     // Priority from config.h
        &xSystemMonitorTaskHandle     // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System monitoring task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(xSystemMonitorTaskHandle, "SysMonitor", MONITOR_INTERVAL_MS + 1000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_FATAL, "app_main", NULL);
        ESP_LOGE(TAG, "ERROR: Failed to create system monitoring task!");
        abort();
    }

    // 6. Create the heartbeat task
    xResult = xTaskCreate(
        vHeartbeatTask,               // Task function
        "Heartbeat",                  // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_BACKGROUND,     // Priority from config.h
        &xHeartbeatTaskHandle         // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "Heartbeat task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(xHeartbeatTaskHandle, "Heartbeat", 6000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create heartbeat task");
    }

    // FreeRTOS scheduler is already running in ESP-IDF
    // Tasks will start executing immediately
    ESP_LOGI(TAG, "All tasks created successfully. System is now running...");
}