# WiFi Manager Integration Prototype

## Design Overview

The WiFi manager is implemented as a separate component following the existing architecture pattern. It provides:
- Non-blocking WiFi connection management
- Automatic reconnection with exponential backoff
- Integration with error recovery and watchdog systems
- Connection status monitoring
- Event-driven architecture using ESP-IDF WiFi APIs

## Integration in main.c

Here's how to integrate WiFi into `main.c` while keeping it clean:

```c
// main/main.c - ESP-IDF Application Entry Point

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"         // From sensor_system component
#include "types.h"          // From sensor_system component
#include "system_init.h"    // From sensor_system component
#include "watchdog.h"       // From sensor_system component
#include "error_recovery.h" // From sensor_system component
#include "wifi_manager.h"  // From sensor_system component - NEW

static const char *TAG = "MAIN";

void app_main(void) {
    // Initialize ESP_LOG system
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

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
    // 3. Initialize WiFi Manager (NEW)
    // ============================================================================
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "Failed to initialize WiFi manager: %s", esp_err_to_name(ret));
        // Continue anyway - WiFi is not critical for basic operation
    }

    ret = wifi_manager_start_task();
    if (ret != pdPASS) {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "Failed to start WiFi manager task");
        // Continue anyway - WiFi is not critical for basic operation
    }

    // Connect to WiFi (optional - can be done later or via command)
    // wifi_manager_connect("YOUR_SSID", "YOUR_PASSWORD");

    // ============================================================================

    // 4. Create the system information task
    BaseType_t xResult = xTaskCreate(
        vSystemInfoTask,
        "SystemInfo",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_FIXED_FREQ,
        &xSystemInfoTaskHandle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System information task created successfully");
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_ERROR, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create system information task");
    }

    // 5. Create the system monitoring task
    xResult = xTaskCreate(
        vSystemMonitorTask,
        "SysMonitor",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_BACKGROUND,
        &xSystemMonitorTaskHandle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System monitoring task created successfully");
        watchdog_register_task(xSystemMonitorTaskHandle, "SysMonitor", MONITOR_INTERVAL_MS + 1000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_FATAL, "app_main", NULL);
        ESP_LOGE(TAG, "ERROR: Failed to create system monitoring task!");
        abort();
    }

    // 6. Create the heartbeat task
    xResult = xTaskCreate(
        vHeartbeatTask,
        "Heartbeat",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_BACKGROUND,
        &xHeartbeatTaskHandle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "Heartbeat task created successfully");
        watchdog_register_task(xHeartbeatTaskHandle, "Heartbeat", 6000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create heartbeat task");
    }

    ESP_LOGI(TAG, "All tasks created successfully. System is now running...");
}
```

## Component Architecture

```
components/sensor_system/
├── include/
│   └── wifi_manager.h      # Public API declarations
├── src/
│   └── wifi_manager.c      # Implementation
└── CMakeLists.txt          # Updated to include wifi_manager.c
```

## Key Features

### 1. Clean Separation
- All WiFi functionality is in `wifi_manager.c`
- Only 2 function calls needed in `main.c`:
  - `wifi_manager_init()` - Initialize WiFi subsystem
  - `wifi_manager_start_task()` - Start background connection manager

### 2. Connection Management
```c
// Connect to WiFi
wifi_manager_connect("MyNetwork", "MyPassword");

// Check connection status
if (wifi_manager_is_connected()) {
    char ip[16];
    wifi_manager_get_ip_address(ip, sizeof(ip));
    ESP_LOGI(TAG, "Connected! IP: %s", ip);
}

// Get signal strength
int8_t rssi = wifi_manager_get_rssi();
ESP_LOGI(TAG, "Signal strength: %d dBm", rssi);
```

### 3. Automatic Reconnection
- Background task monitors connection
- Exponential backoff on reconnection failures
- Configurable retry limits
- Integrated with error recovery system

### 4. Statistics and Monitoring
```c
WiFiStats_t stats;
wifi_manager_get_stats(&stats);
ESP_LOGI(TAG, "Connections: %lu, Uptime: %lu sec, RSSI: %d",
         stats.successfulConnections, stats.uptime, stats.rssi);
```

## Configuration Options

WiFi behavior can be configured via constants in `wifi_manager.h`:
- `WIFI_MAX_RETRY_COUNT` - Maximum connection retries
- `WIFI_CONNECT_TIMEOUT_MS` - Connection timeout
- `WIFI_RECONNECT_DELAY_MS` - Initial reconnection delay
- `WIFI_MAX_RECONNECT_DELAY_MS` - Maximum reconnection delay

## Integration Points

1. **Error Recovery**: WiFi errors are reported to error recovery system
2. **Watchdog**: WiFi manager task registered with watchdog
3. **System Monitor**: Can query WiFi status and statistics
4. **Event-Driven**: Uses ESP-IDF event system for connection events

## Usage Example

```c
// In your application code:
void connect_to_wifi(void) {
    esp_err_t ret = wifi_manager_connect("MySSID", "MyPassword");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate WiFi connection");
        return;
    }
    
    // Wait for connection (or check periodically)
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if (wifi_manager_is_connected()) {
        char ip[16];
        wifi_manager_get_ip_address(ip, sizeof(ip));
        ESP_LOGI(TAG, "WiFi connected! IP: %s", ip);
    }
}
```

## Benefits

1. **Clean main.c**: Only 2 function calls needed
2. **Modular**: Can be easily disabled or replaced
3. **Robust**: Automatic reconnection with backoff
4. **Integrated**: Works with existing error recovery and watchdog
5. **Configurable**: Easy to adjust timing and retry parameters
6. **Non-blocking**: Connection management runs in background task

