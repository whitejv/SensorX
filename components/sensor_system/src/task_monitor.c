/*
 * task_monitor.c - Boot Button Task Monitor Implementation
 *
 * This file implements the boot button-triggered task monitor for FreeRTOS.
 * Provides task statistics via serial output only.
 *
 * Features:
 * - Boot button (GPIO9) polling with debouncing
 * - Serial output using vTaskList() and vTaskGetRunTimeStats()
 * - Non-intrusive (lowest priority task)
 */

#include "task_monitor.h"
#include "config.h"
#include "watchdog.h"
// MQTT and GPIO removed - serial output only, periodic timer-based
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "TASK_MONITOR";

// Task handle
static TaskHandle_t task_monitor_task_handle = NULL;

/**
 * Check if task monitor output is enabled (production gate)
 */
static bool task_monitor_output_enabled(void) {
#if PRODUCTION_MODE
    return (ENABLE_TASK_MONITOR_OUTPUT == 1);
#else
    return true;  // Always enabled in development mode
#endif
}

// MQTT publishing removed - serial output only to reduce stack usage

/**
 * Helper function to print formatted task list with proper column alignment
 */
static void print_formatted_task_list(const char* raw_buffer) {
    const char* line = raw_buffer;
    bool header_printed = false;
    int task_count = 0;  // Track number of tasks printed for watchdog resets
    
    while (*line != '\0') {
        // Find end of line
        const char* line_end = strchr(line, '\n');
        if (line_end == NULL) {
            line_end = line + strlen(line);
        }
        
        // Skip empty lines
        if (line_end == line) {
            line = line_end + 1;
            continue;
        }
        
        // Parse line (tab-separated)
        char name[32] = {0};
        char state[16] = {0};
        int prio = 0;
        int hwm = 0;
        int core = -1;
        int task_num = -1;
        
        // Try to parse: Name\t\tState\tPrio\tHWM\t\tCore\tTaskNum
        int parsed = sscanf(line, "%31s\t\t%15s\t%d\t%d\t\t%d\t%d", 
                           name, state, &prio, &hwm, &core, &task_num);
        
        if (parsed >= 4) {
            // Print formatted line
            if (core >= 0 && task_num >= 0) {
                ESP_LOGI(TAG, "%-16s %-8s %4d  %6d  %4d  %4d", 
                        name, state, prio, hwm, core, task_num);
            } else {
                ESP_LOGI(TAG, "%-16s %-8s %4d  %6d", 
                        name, state, prio, hwm);
            }
            task_count++;
            
            // Reset watchdog every 5 tasks to prevent timeout during long printing operations
            // ESP_LOGI() calls can take 10-50ms each, so 5 tasks = ~250ms max between resets
            if (task_count % 5 == 0) {
                watchdog_task_heartbeat();
            }
        } else {
            // Header or unparseable line - print as-is
            if (!header_printed && strstr(line, "Name") != NULL) {
                ESP_LOGI(TAG, "%-16s %-8s %4s  %6s  %4s  %4s", 
                        "Name", "State", "Prio", "HWM", "Core", "Task#");
                header_printed = true;
            } else if (strlen(line) > 0) {
                ESP_LOGI(TAG, "%.*s", (int)(line_end - line), line);
            }
        }
        
        line = (*line_end == '\0') ? line_end : line_end + 1;
    }
}

/**
 * Helper function to print formatted runtime stats with proper column alignment
 */
static void print_formatted_runtime_stats(const char* raw_buffer) {
    const char* line = raw_buffer;
    bool header_printed = false;
    int task_count = 0;  // Track number of tasks printed for watchdog resets
    
    while (*line != '\0') {
        // Find end of line
        const char* line_end = strchr(line, '\n');
        if (line_end == NULL) {
            line_end = line + strlen(line);
        }
        
        // Skip empty lines
        if (line_end == line) {
            line = line_end + 1;
            continue;
        }
        
        // Parse line: Name\t\tRuntime\t\tPercentage
        char name[32] = {0};
        unsigned long runtime = 0;
        float percentage = 0.0f;
        char percentage_str[16] = {0};
        
        // Try to parse percentage (could be "<1%" or "12.34%")
        int parsed = sscanf(line, "%31s\t\t%lu\t\t%15s", name, &runtime, percentage_str);
        
        if (parsed >= 2) {
            // Print formatted line
            if (parsed >= 3) {
                ESP_LOGI(TAG, "%-16s %12lu  %10s", name, runtime, percentage_str);
            } else {
                ESP_LOGI(TAG, "%-16s %12lu", name, runtime);
            }
            task_count++;
            
            // Reset watchdog every 5 tasks to prevent timeout during long printing operations
            // ESP_LOGI() calls can take 10-50ms each, so 5 tasks = ~250ms max between resets
            if (task_count % 5 == 0) {
                watchdog_task_heartbeat();
            }
        } else {
            // Header or unparseable line - print as-is
            if (!header_printed && strstr(line, "Name") != NULL) {
                ESP_LOGI(TAG, "%-16s %12s  %10s", "Name", "Runtime", "Percentage");
                header_printed = true;
            } else if (strlen(line) > 0) {
                ESP_LOGI(TAG, "%.*s", (int)(line_end - line), line);
            }
        }
        
        line = (*line_end == '\0') ? line_end : line_end + 1;
    }
}

/**
 * Print task information to serial output only
 */
void task_monitor_print_task_info(void) {
    // Use static buffers to reduce stack usage
    // Buffer sizes increased to safely handle many tasks (18+ tasks observed)
    // Each task line: ~50-60 bytes for list, ~40-50 bytes for stats
    static char list_buffer[1536] = {0};   // Increased from 512 to 1536 (30+ tasks)
    static char stats_buffer[1536] = {0};  // Increased from 800 to 1536 (30+ tasks)

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "=== FreeRTOS Task List (vTaskList) ===");

    vTaskList(list_buffer);
    print_formatted_task_list(list_buffer);

    // Feed watchdog between long operations to prevent timeout
    // vTaskGetRunTimeStats() can take several seconds, exceeding the 5-second watchdog timeout
    watchdog_task_heartbeat();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== FreeRTOS Runtime Stats (vTaskGetRunTimeStats) ===");

    // Reset watchdog immediately before vTaskGetRunTimeStats() since it can take 3-5+ seconds
    watchdog_task_heartbeat();
    
    vTaskGetRunTimeStats(stats_buffer);
    print_formatted_runtime_stats(stats_buffer);

    // Feed watchdog again after the second long operation
    watchdog_task_heartbeat();

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "");
}

/**
 * Task monitor task - outputs task statistics periodically (every 2 minutes)
 */
static void vTaskMonitorTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Task Monitor Task started (periodic mode, interval: %d ms)", TASK_MONITOR_INTERVAL_MS);
    watchdog_register_current_task("TaskMonitor", TASK_MONITOR_INTERVAL_MS * 2);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        watchdog_task_heartbeat();
        
        // Check production gate before output
        if (task_monitor_output_enabled()) {
            ESP_LOGI(TAG, "Periodic task statistics (every %d ms)", TASK_MONITOR_INTERVAL_MS);
            task_monitor_print_task_info();
        } else {
            ESP_LOGD(TAG, "Task monitor output disabled (production mode)");
        }
        
        // Break up long delay into smaller chunks with watchdog resets
        // ESP-IDF watchdog timeout is 5 seconds, so we need to reset every 4 seconds max
        TickType_t target_wake_time = last_wake_time + pdMS_TO_TICKS(TASK_MONITOR_INTERVAL_MS);
        const TickType_t max_delay_between_resets = pdMS_TO_TICKS(4000); // 4 seconds max between resets
        
        while (xTaskGetTickCount() < target_wake_time) {
            TickType_t now = xTaskGetTickCount();
            TickType_t remaining = target_wake_time - now;
            TickType_t delay_chunk = (remaining > max_delay_between_resets) ? 
                                     max_delay_between_resets : remaining;
            
            if (delay_chunk > 0) {
                vTaskDelay(delay_chunk);
            }
            
            // Feed watchdog during long delays to prevent timeout
            if (xTaskGetTickCount() < target_wake_time) {
                watchdog_task_heartbeat();
            }
        }
        
        // Update last_wake_time for next cycle (maintains periodicity)
        last_wake_time = target_wake_time;
    }
}

/**
 * Initialize task monitor (simplified - no GPIO needed)
 */
esp_err_t task_monitor_init(void) {
    if (task_monitor_task_handle != NULL) {
        ESP_LOGW(TAG, "Task monitor already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Task Monitor initialized (periodic mode, interval: %d ms)", TASK_MONITOR_INTERVAL_MS);
#if PRODUCTION_MODE
    ESP_LOGI(TAG, "Production mode: Task monitor output %s", 
             ENABLE_TASK_MONITOR_OUTPUT ? "enabled" : "disabled");
#else
    ESP_LOGI(TAG, "Development mode: Task monitor output enabled");
#endif
    return ESP_OK;
}

/**
 * Start task monitor task
 */
BaseType_t task_monitor_start_task(void) {
    if (task_monitor_task_handle != NULL) {
        ESP_LOGW(TAG, "Task monitor task already started");
        return pdFAIL;
    }

    BaseType_t ret = xTaskCreate(
        vTaskMonitorTask,
        "TaskMonitor",
        TASK_MONITOR_STACK_SIZE,
        NULL,
        TASK_MONITOR_PRIORITY,
        &task_monitor_task_handle
    );

    if (ret == pdPASS) {
        ESP_LOGI(TAG, "Task monitor task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create task monitor task");
    }

    return ret;
}

