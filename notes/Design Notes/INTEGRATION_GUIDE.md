# Watchdog and Error Recovery Integration Guide

This document describes how the watchdog timer and error recovery systems integrate with the existing codebase.

## Overview

The watchdog and error recovery systems are designed to work together to provide:
1. **Hardware-level protection** via ESP32 Task Watchdog Timer
2. **Task-level health monitoring** for early problem detection
3. **Automatic error recovery** with configurable strategies
4. **System state management** based on error conditions

## Integration Points

### 1. Initialization Sequence (in `app_main()`)

```c
void app_main(void) {
    // ... existing logging setup ...
    
    // 1. Initialize error recovery FIRST (needed for error handling)
    error_recovery_init();
    error_recovery_start_task();
    
    // 2. Initialize watchdog timer
    watchdog_init(WATCHDOG_TIMEOUT_SEC);
    watchdog_start_task();
    
    // 3. Create existing tasks (SystemInfo, SystemMonitor, Heartbeat)
    // ... existing task creation ...
    
    // 4. Register all tasks with watchdog after creation
    watchdog_register_current_task("SystemInfo", 5000);  // Expected 5s heartbeat
    watchdog_register_current_task("SysMonitor", 6000); // Expected 6s heartbeat
    watchdog_register_current_task("Heartbeat", 6000);  // Expected 6s heartbeat
}
```

### 2. Task Integration

Each task should call `watchdog_task_heartbeat()` in its main loop:

```c
void vSystemMonitorTask(void *pvParameters) {
    // Register with watchdog
    watchdog_register_current_task("SysMonitor", MONITOR_INTERVAL_MS + 1000);
    
    for (;;) {
        // ... existing monitoring code ...
        
        // Heartbeat to watchdog
        watchdog_task_heartbeat();
        
        vTaskDelay(xMonitorInterval);
    }
}
```

### 3. Error Reporting

Replace direct error logging with error reporting:

```c
// Instead of:
ESP_LOGE(TAG, "I2C timeout");
return ESP_FAIL;

// Use:
error_report(ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, "I2C_Sensor", NULL);
return ESP_FAIL;
```

### 4. Sensor Driver Integration Example

```c
esp_err_t sensor_read_i2c(uint8_t address, uint8_t* data, size_t len) {
    esp_err_t ret = i2c_master_write_read_device(...);
    
    if (ret == ESP_ERR_TIMEOUT) {
        error_report(ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, 
                     "sensor_read_i2c", NULL);
        // Recovery system will attempt device reset automatically
    } else if (ret == ESP_ERR_NOT_FOUND) {
        error_report(ERROR_SENSOR_NOT_FOUND, ERROR_SEVERITY_WARNING,
                     "sensor_read_i2c", NULL);
    }
    
    return ret;
}
```

## Configuration Updates Needed

Add to `config.h`:

```c
// Watchdog configuration
#define WATCHDOG_TIMEOUT_SEC          30    // Hardware watchdog timeout
#define WATCHDOG_FEED_INTERVAL_MS     50    // Feed interval (must be < timeout)
#define WATCHDOG_HEALTH_CHECK_INTERVAL_MS 1000

// Error recovery configuration
#define ERROR_HISTORY_SIZE            32
#define MAX_RECOVERY_STRATEGIES       16
#define ERROR_RECOVERY_TASK_INTERVAL_MS 5000
```

## Task Priority Adjustments

Update task priorities in `config.h`:

```c
#define TASK_PRIORITY_CRITICAL    5    // Watchdog task, sensor acquisition
#define TASK_PRIORITY_FIXED_FREQ  3    // Error recovery, MQTT publisher
#define TASK_PRIORITY_BACKGROUND  1    // SD logger, system monitor
#define TASK_PRIORITY_IDLE        0    // WiFi manager, OTA handler
```

## System State Integration

Update `vSystemMonitorTask()` to use error recovery system state:

```c
void vSystemMonitorTask(void *pvParameters) {
    for (;;) {
        // ... existing monitoring ...
        
        // Get system state from error recovery
        SystemState_t state = error_recovery_get_system_state();
        if (state != SYSTEM_STATE_RUNNING) {
            ESP_LOGW(TAG, "System state: %d", state);
        }
        
        // Get error statistics
        ErrorRecoveryStats_t error_stats;
        error_recovery_get_stats(&error_stats);
        ESP_LOGI(TAG, "Errors: total=%lu, recovered=%lu, critical=%lu",
                 error_stats.total_errors,
                 error_stats.recovered_errors,
                 error_stats.critical_errors);
    }
}
```

## Recovery Strategy Customization

Register custom recovery strategies in `app_main()`:

```c
// Custom strategy for a specific sensor
RecoveryStrategy_t custom_strategy = {
    .error_code = ERROR_I2C_TIMEOUT,
    .severity = ERROR_SEVERITY_ERROR,
    .action = RECOVERY_ACTION_RESET_DEVICE,
    .max_retries = 5,
    .retry_delay_ms = 200,
    .escalation_threshold = 10,
    .auto_recover = true
};
error_recovery_register_strategy(&custom_strategy);
```

## Watchdog Statistics

Monitor watchdog health:

```c
WatchdogStats_t wdt_stats;
watchdog_get_stats(&wdt_stats);
ESP_LOGI(TAG, "Watchdog: feeds=%lu, timeouts=%lu, tasks=%lu",
         wdt_stats.totalFeeds,
         wdt_stats.timeoutCount,
         wdt_stats.tasksMonitored);
```

## Testing Considerations

1. **Watchdog Test**: Temporarily comment out `watchdog_task_heartbeat()` in a task to verify watchdog triggers
2. **Error Recovery Test**: Manually call `error_report()` with various error codes
3. **Recovery Test**: Simulate I2C failures to test automatic recovery
4. **Fatal Error Test**: Trigger a fatal error to verify restart mechanism

## Debugging

- Use `watchdog_disable()` only during development (NEVER in production)
- Enable verbose error logging to track recovery attempts
- Monitor error history with `error_recovery_get_history()`
- Check task health with `watchdog_get_task_health()`

## Files to Add

1. `components/sensor_system/include/watchdog.h` - Header file
2. `components/sensor_system/src/watchdog.c` - Implementation
3. `components/sensor_system/include/error_recovery.h` - Header file
4. `components/sensor_system/src/error_recovery.c` - Implementation
5. Update `components/sensor_system/CMakeLists.txt` to include new sources

## CMakeLists.txt Update

```cmake
idf_component_register(SRCS "src/system_init.c"
                            "src/watchdog.c"
                            "src/error_recovery.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support driver)
```

