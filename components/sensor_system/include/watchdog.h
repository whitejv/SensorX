/*
 * watchdog.h - Watchdog Timer System Prototype
 *
 * This file defines the watchdog timer interface for the ESP32 sensor system.
 * The watchdog provides system monitoring and automatic recovery from hangs.
 *
 * Design Principles:
 * - Use ESP32 Task Watchdog Timer (TWDT) for hardware-level protection
 * - Track task health individually before feeding watchdog
 * - Provide task registration/unregistration for monitoring
 * - Automatic feeding from a dedicated watchdog task
 * - Configurable timeout values per priority level
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <stdbool.h>
#include <stdint.h>

// Watchdog configuration constants
#define WATCHDOG_TIMEOUT_SEC          30    // Maximum timeout in seconds (hardware limit: 60s)
#define WATCHDOG_FEED_INTERVAL_MS     50    // Feed interval (must be < timeout)
#define WATCHDOG_HEALTH_CHECK_INTERVAL_MS 1000  // Check task health every 1 second
#define WATCHDOG_MAX_REGISTERED_TASKS 16    // Maximum number of monitored tasks

// Task health status
typedef enum {
    TASK_HEALTH_OK = 0,
    TASK_HEALTH_STUCK,
    TASK_HEALTH_NOT_RESPONDING,
    TASK_HEALTH_UNKNOWN
} TaskHealthStatus_t;

// Task health information structure
typedef struct {
    TaskHandle_t handle;              // Task handle
    const char* name;                 // Task name
    uint32_t lastHeartbeat;            // Last heartbeat timestamp (ms)
    uint32_t expectedInterval;         // Expected heartbeat interval (ms)
    TaskHealthStatus_t status;        // Current health status
    uint32_t stuckCount;               // Number of times detected as stuck
    uint32_t recoveryCount;            // Number of recovery attempts
} TaskHealthInfo_t;

// Watchdog statistics
typedef struct {
    uint32_t totalFeeds;              // Total number of watchdog feeds
    uint32_t timeoutCount;             // Number of watchdog timeouts detected
    uint32_t recoveryCount;       // Number of recovery attempts
    uint32_t tasksMonitored;           // Number of tasks currently monitored
    bool watchdogArmed;                // Whether watchdog is currently armed
} WatchdogStats_t;

/*
 * Initialize the watchdog timer system
 * 
 * @param timeout_seconds: Watchdog timeout in seconds (max 60)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t watchdog_init(uint32_t timeout_seconds);

/*
 * Start the watchdog feeding task
 * This task runs at high priority and feeds the watchdog periodically
 * 
 * @return pdPASS on success, pdFAIL on failure
 */
BaseType_t watchdog_start_task(void);

/*
 * Register a task for health monitoring
 * The task must call watchdog_task_heartbeat() periodically
 * 
 * @param task_handle: Handle of the task to monitor
 * @param task_name: Human-readable task name
 * @param expected_interval_ms: Expected heartbeat interval in milliseconds
 * @return ESP_OK on success, error code on failure
 */
esp_err_t watchdog_register_task(TaskHandle_t task_handle, 
                                   const char* task_name,
                                   uint32_t expected_interval_ms);

/*
 * Unregister a task from health monitoring
 * 
 * @param task_handle: Handle of the task to unregister
 * @return ESP_OK on success, error code on failure
 */
esp_err_t watchdog_unregister_task(TaskHandle_t task_handle);

/*
 * Feed the watchdog timer
 * Tasks should call this regularly to prevent timeout
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t watchdog_feed(void);

/*
 * Task heartbeat function
 * Tasks should call this periodically to indicate they are alive
 * This is separate from feeding the watchdog - it tracks individual task health
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t watchdog_task_heartbeat(void);

/*
 * Add current task to watchdog monitoring
 * Convenience function that gets the current task handle automatically
 * 
 * @param task_name: Human-readable task name
 * @param expected_interval_ms: Expected heartbeat interval in milliseconds
 * @return ESP_OK on success, error code on failure
 */
esp_err_t watchdog_register_current_task(const char* task_name,
                                          uint32_t expected_interval_ms);

/*
 * Check health of all registered tasks
 * Called by the watchdog task to detect stuck tasks
 * 
 * @return Number of unhealthy tasks detected
 */
uint32_t watchdog_check_task_health(void);

/*
 * Get health information for a specific task
 * 
 * @param task_handle: Task handle to query
 * @param health_info: Pointer to structure to fill with health info
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if task not registered
 */
esp_err_t watchdog_get_task_health(TaskHandle_t task_handle,
                                     TaskHealthInfo_t* health_info);

/*
 * Get watchdog statistics
 * 
 * @param stats: Pointer to structure to fill with statistics
 */
void watchdog_get_stats(WatchdogStats_t* stats);

/*
 * Reset watchdog statistics
 */
void watchdog_reset_stats(void);

/*
 * Disable watchdog timer (for debugging only)
 * WARNING: This should only be used during development
 */
esp_err_t watchdog_disable(void);

/*
 * Re-enable watchdog timer after disabling
 */
esp_err_t watchdog_enable(uint32_t timeout_seconds);

// Watchdog task function (internal use)
void vWatchdogTask(void *pvParameters);

#endif /* WATCHDOG_H */

