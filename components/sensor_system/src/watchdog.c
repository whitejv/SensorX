/*
 * watchdog.c - Watchdog Timer System Implementation
 *
 * This file implements the watchdog timer system for the ESP32 sensor system.
 * Uses ESP32 Task Watchdog Timer (TWDT) for hardware-level protection.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <stdbool.h>

#include "watchdog.h"
#include "config.h"

static const char *TAG = "WATCHDOG";

// Static variables
static bool watchdog_initialized = false;
static bool watchdog_armed = false;
static uint32_t watchdog_timeout_sec = WATCHDOG_TIMEOUT_SEC;
static TaskHealthInfo_t registered_tasks[WATCHDOG_MAX_REGISTERED_TASKS];
static uint32_t num_registered_tasks = 0;
static WatchdogStats_t stats = {0};
static TaskHandle_t watchdog_task_handle = NULL;

esp_err_t watchdog_init(uint32_t timeout_seconds) {
    if (watchdog_initialized) {
        ESP_LOGW(TAG, "Watchdog already initialized");
        return ESP_OK;
    }

    // Validate timeout (max 60 seconds for ESP32)
    if (timeout_seconds == 0 || timeout_seconds > 60) {
        ESP_LOGE(TAG, "Invalid watchdog timeout: %lu (must be 1-60 seconds)", timeout_seconds);
        return ESP_ERR_INVALID_ARG;
    }

    watchdog_timeout_sec = timeout_seconds;

    // Initialize ESP32 Task Watchdog Timer
    // ESP-IDF v5.x uses configuration structure
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = timeout_seconds * 1000,
        .idle_core_mask = 0,  // Don't monitor idle tasks
        .trigger_panic = true
    };
    esp_err_t ret = esp_task_wdt_init(&wdt_config);
    if (ret == ESP_ERR_INVALID_STATE) {
        // Watchdog already initialized (possibly by ESP-IDF or previous call)
        ESP_LOGW(TAG, "Watchdog already initialized, reconfiguring...");
        // Try to reconfigure it
        ret = esp_task_wdt_reconfigure(&wdt_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Could not reconfigure watchdog: %s, continuing anyway", esp_err_to_name(ret));
            // Continue - watchdog is already active
        }
        // Mark as initialized since watchdog hardware is active
        watchdog_initialized = true;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize watchdog: %s", esp_err_to_name(ret));
        return ret;
    } else {
        watchdog_initialized = true;
    }

    // Don't add main task to watchdog - it becomes idle after app_main() returns
    // and won't be able to feed the watchdog

    // Initialize registered tasks array
    memset(registered_tasks, 0, sizeof(registered_tasks));
    num_registered_tasks = 0;

    // Initialize statistics
    memset(&stats, 0, sizeof(WatchdogStats_t));

    watchdog_armed = true;

    ESP_LOGI(TAG, "Watchdog initialized with timeout: %lu seconds", timeout_seconds);
    return ESP_OK;
}

BaseType_t watchdog_start_task(void) {
    if (!watchdog_initialized) {
        ESP_LOGE(TAG, "Watchdog not initialized");
        return pdFAIL;
    }

    if (watchdog_task_handle != NULL) {
        ESP_LOGW(TAG, "Watchdog task already running");
        return pdPASS;
    }

    BaseType_t result = xTaskCreate(
        vWatchdogTask,
        "WatchdogTask",
        TASK_STACK_SIZE_CRITICAL,
        NULL,
        TASK_PRIORITY_CRITICAL,
        &watchdog_task_handle
    );

    if (result == pdPASS) {
        ESP_LOGI(TAG, "Watchdog task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create watchdog task");
    }

    return result;
}

void vWatchdogTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Watchdog task started");

    // Register this task with itself
    watchdog_register_current_task("WatchdogTask", WATCHDOG_FEED_INTERVAL_MS * 2);

    const TickType_t xFeedInterval = pdMS_TO_TICKS(WATCHDOG_FEED_INTERVAL_MS);
    const TickType_t xHealthCheckInterval = pdMS_TO_TICKS(WATCHDOG_HEALTH_CHECK_INTERVAL_MS);
    TickType_t xLastHealthCheck = xTaskGetTickCount();

    for (;;) {
        // Check task health periodically
        TickType_t xCurrentTime = xTaskGetTickCount();
        if ((xCurrentTime - xLastHealthCheck) >= xHealthCheckInterval) {
            uint32_t unhealthy_count = watchdog_check_task_health();
            if (unhealthy_count > 0) {
                ESP_LOGW(TAG, "Detected %lu unhealthy tasks", unhealthy_count);
            }
            xLastHealthCheck = xCurrentTime;
        }

        // Feed the hardware watchdog
        if (watchdog_armed) {
            esp_err_t ret = esp_task_wdt_reset();
            if (ret == ESP_OK) {
                stats.totalFeeds++;
            } else {
                ESP_LOGE(TAG, "Failed to feed watchdog: %s", esp_err_to_name(ret));
            }
        }

        // Send heartbeat for this task
        watchdog_task_heartbeat();

        vTaskDelay(xFeedInterval);
    }
}

esp_err_t watchdog_register_task(TaskHandle_t task_handle,
                                   const char* task_name,
                                   uint32_t expected_interval_ms) {
    // Check if watchdog hardware is initialized (even if our wrapper isn't)
    // This allows tasks to register even if called before watchdog_init()
    if (!watchdog_initialized) {
        ESP_LOGW(TAG, "Watchdog wrapper not initialized yet, registering anyway");
        // Continue to register - the hardware watchdog may already be initialized
    }

    if (task_handle == NULL || task_name == NULL || expected_interval_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if task already registered
    for (uint32_t i = 0; i < num_registered_tasks; i++) {
        if (registered_tasks[i].handle == task_handle) {
            ESP_LOGW(TAG, "Task %s already registered", task_name);
            return ESP_OK;
        }
    }

    // Check if we have space
    if (num_registered_tasks >= WATCHDOG_MAX_REGISTERED_TASKS) {
        ESP_LOGE(TAG, "Maximum number of registered tasks reached");
        return ESP_ERR_NO_MEM;
    }

    // Find empty slot
    uint32_t index = num_registered_tasks;
    registered_tasks[index].handle = task_handle;
    registered_tasks[index].name = task_name;
    registered_tasks[index].expectedInterval = expected_interval_ms;
    registered_tasks[index].lastHeartbeat = xTaskGetTickCount() * portTICK_PERIOD_MS;
    registered_tasks[index].status = TASK_HEALTH_OK;
    registered_tasks[index].stuckCount = 0;
    registered_tasks[index].recoveryCount = 0;

    // Add task to ESP32 watchdog
    esp_err_t ret = esp_task_wdt_add(task_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add task %s to watchdog: %s", task_name, esp_err_to_name(ret));
        return ret;
    }

    num_registered_tasks++;
    stats.tasksMonitored = num_registered_tasks;

    ESP_LOGI(TAG, "Registered task: %s (interval: %lu ms)", task_name, expected_interval_ms);
    return ESP_OK;
}

esp_err_t watchdog_unregister_task(TaskHandle_t task_handle) {
    if (!watchdog_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Find task in array
    for (uint32_t i = 0; i < num_registered_tasks; i++) {
        if (registered_tasks[i].handle == task_handle) {
            // Remove from ESP32 watchdog
            esp_task_wdt_delete(task_handle);

            // Shift remaining tasks
            for (uint32_t j = i; j < num_registered_tasks - 1; j++) {
                registered_tasks[j] = registered_tasks[j + 1];
            }
            num_registered_tasks--;
            stats.tasksMonitored = num_registered_tasks;

            ESP_LOGI(TAG, "Unregistered task");
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t watchdog_task_heartbeat(void) {
    if (!watchdog_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Find task in registered tasks
    bool task_found = false;
    for (uint32_t i = 0; i < num_registered_tasks; i++) {
        if (registered_tasks[i].handle == current_task) {
            registered_tasks[i].lastHeartbeat = current_time;
            
            // Reset stuck count if task was stuck
            if (registered_tasks[i].status != TASK_HEALTH_OK) {
                registered_tasks[i].status = TASK_HEALTH_OK;
                registered_tasks[i].stuckCount = 0;
            }
            task_found = true;
            break;
        }
    }

    // Feed the hardware watchdog (required for registered tasks)
    // In ESP-IDF, each registered task must reset its own watchdog
    if (task_found && watchdog_armed) {
        esp_err_t ret = esp_task_wdt_reset();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to reset watchdog: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Task not registered - this is OK, just return
    return ESP_OK;
}

esp_err_t watchdog_register_current_task(const char* task_name,
                                          uint32_t expected_interval_ms) {
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    return watchdog_register_task(current_task, task_name, expected_interval_ms);
}

uint32_t watchdog_check_task_health(void) {
    if (!watchdog_initialized) {
        return 0;
    }

    uint32_t unhealthy_count = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (uint32_t i = 0; i < num_registered_tasks; i++) {
        if (registered_tasks[i].handle == NULL) {
            continue;
        }

        uint32_t time_since_heartbeat = current_time - registered_tasks[i].lastHeartbeat;
        uint32_t expected_interval = registered_tasks[i].expectedInterval;

        // Check if task is stuck (exceeded 2x expected interval)
        if (time_since_heartbeat > (expected_interval * 2)) {
            if (registered_tasks[i].status != TASK_HEALTH_STUCK &&
                registered_tasks[i].status != TASK_HEALTH_NOT_RESPONDING) {
                registered_tasks[i].status = TASK_HEALTH_STUCK;
                registered_tasks[i].stuckCount++;
                ESP_LOGW(TAG, "Task %s appears stuck (no heartbeat for %lu ms, expected %lu ms)",
                         registered_tasks[i].name, time_since_heartbeat, expected_interval);
            }
            unhealthy_count++;
        }

        // Check if task is not responding (exceeded 5x expected interval)
        if (time_since_heartbeat > (expected_interval * 5)) {
            if (registered_tasks[i].status != TASK_HEALTH_NOT_RESPONDING) {
                registered_tasks[i].status = TASK_HEALTH_NOT_RESPONDING;
                ESP_LOGE(TAG, "Task %s not responding (no heartbeat for %lu ms)",
                         registered_tasks[i].name, time_since_heartbeat);
            }
        }
    }

    return unhealthy_count;
}

esp_err_t watchdog_get_task_health(TaskHandle_t task_handle,
                                     TaskHealthInfo_t* health_info) {
    if (!watchdog_initialized || health_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint32_t i = 0; i < num_registered_tasks; i++) {
        if (registered_tasks[i].handle == task_handle) {
            *health_info = registered_tasks[i];
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t watchdog_feed(void) {
    if (!watchdog_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!watchdog_armed) {
        return ESP_OK;
    }

    esp_err_t ret = esp_task_wdt_reset();
    if (ret == ESP_OK) {
        stats.totalFeeds++;
    }
    return ret;
}

void watchdog_get_stats(WatchdogStats_t* stats_out) {
    if (stats_out != NULL) {
        *stats_out = stats;
        stats_out->watchdogArmed = watchdog_armed;
    }
}

void watchdog_reset_stats(void) {
    memset(&stats, 0, sizeof(WatchdogStats_t));
    stats.watchdogArmed = watchdog_armed;
    stats.tasksMonitored = num_registered_tasks;
}

esp_err_t watchdog_disable(void) {
    if (!watchdog_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "WARNING: Disabling watchdog (development only)");
    watchdog_armed = false;
    return ESP_OK;
}

esp_err_t watchdog_enable(uint32_t timeout_seconds) {
    if (!watchdog_initialized) {
        return watchdog_init(timeout_seconds);
    }

    watchdog_armed = true;
    ESP_LOGI(TAG, "Watchdog re-enabled");
    return ESP_OK;
}

