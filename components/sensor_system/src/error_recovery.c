/*
 * error_recovery.c - Error Recovery System Implementation
 *
 * This file implements the error recovery system for the ESP32 sensor system.
 * Provides centralized error tracking, reporting, and automatic recovery.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include <stdbool.h>

#include "error_recovery.h"
#include "config.h"
#include "watchdog.h"

static const char *TAG = "ERROR_RECOVERY";

// Static variables
static bool error_recovery_initialized = false;
static ErrorRecord_t error_history[ERROR_HISTORY_SIZE];
static uint32_t error_history_count = 0;
static uint32_t error_history_index = 0;
static RecoveryStrategy_t strategies[MAX_RECOVERY_STRATEGIES];
static uint32_t num_strategies = 0;
static ErrorRecoveryStats_t stats = {0};
static SystemState_t current_system_state = SYSTEM_STATE_INIT;
static TaskHandle_t error_recovery_task_handle = NULL;
static esp_err_t (*recovery_callback)(ErrorCode_t, RecoveryAction_t, const char*) = NULL;

// Default recovery strategies
static const RecoveryStrategy_t default_strategies[] = {
    {ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, RECOVERY_ACTION_RESET_DEVICE, 3, 100, 5, true},
    {ERROR_I2C_NACK, ERROR_SEVERITY_WARNING, RECOVERY_ACTION_DISABLE, 1, 0, 10, true},
    {ERROR_SENSOR_NOT_FOUND, ERROR_SEVERITY_WARNING, RECOVERY_ACTION_DISABLE, 2, 500, 3, true},
    {ERROR_BUFFER_OVERFLOW, ERROR_SEVERITY_CRITICAL, RECOVERY_ACTION_REINIT, 2, 1000, 2, true},
    {ERROR_MEMORY_ALLOCATION, ERROR_SEVERITY_CRITICAL, RECOVERY_ACTION_RESTART_TASK, 2, 2000, 2, true},
    {ERROR_TASK_CREATION, ERROR_SEVERITY_FATAL, RECOVERY_ACTION_SOFT_RESTART, 0, 0, 1, true},
    {ERROR_QUEUE_FULL, ERROR_SEVERITY_WARNING, RECOVERY_ACTION_RETRY, 5, 100, 10, true}
};

static const uint32_t default_strategies_count = sizeof(default_strategies) / sizeof(default_strategies[0]);

esp_err_t error_recovery_init(void) {
    if (error_recovery_initialized) {
        ESP_LOGW(TAG, "Error recovery already initialized");
        return ESP_OK;
    }

    // Initialize error history
    memset(error_history, 0, sizeof(error_history));
    error_history_count = 0;
    error_history_index = 0;

    // Initialize strategies array
    memset(strategies, 0, sizeof(strategies));
    num_strategies = 0;

    // Register default strategies
    for (uint32_t i = 0; i < default_strategies_count && i < MAX_RECOVERY_STRATEGIES; i++) {
        error_recovery_register_strategy(&default_strategies[i]);
    }

    // Initialize statistics
    memset(&stats, 0, sizeof(ErrorRecoveryStats_t));

    // Set initial system state
    current_system_state = SYSTEM_STATE_INIT;

    error_recovery_initialized = true;

    ESP_LOGI(TAG, "Error recovery system initialized with %lu default strategies", default_strategies_count);
    return ESP_OK;
}

BaseType_t error_recovery_start_task(void) {
    if (!error_recovery_initialized) {
        ESP_LOGE(TAG, "Error recovery not initialized");
        return pdFAIL;
    }

    if (error_recovery_task_handle != NULL) {
        ESP_LOGW(TAG, "Error recovery task already running");
        return pdPASS;
    }

    BaseType_t result = xTaskCreate(
        vErrorRecoveryTask,
        "ErrorRecovery",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_FIXED_FREQ,
        &error_recovery_task_handle
    );

    if (result == pdPASS) {
        ESP_LOGI(TAG, "Error recovery task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create error recovery task");
    }

    return result;
}

void vErrorRecoveryTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Error recovery monitoring task started");

    // Wait a bit for watchdog to be initialized, then register
    // Watchdog will be initialized after error recovery task starts
    vTaskDelay(pdMS_TO_TICKS(100));
    watchdog_register_current_task("ErrorRecovery", 6000);

    const TickType_t xMonitorInterval = pdMS_TO_TICKS(5000); // 5 second monitoring interval

    for (;;) {
        // Check error history for threshold violations
        for (uint32_t i = 0; i < ERROR_HISTORY_SIZE && i < error_history_count; i++) {
            uint32_t idx = (error_history_index - 1 - i + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
            ErrorRecord_t* record = &error_history[idx];

            if (record->error_code == ERROR_NONE) {
                continue;
            }

            // Check if threshold exceeded
            if (error_recovery_threshold_exceeded(record->error_code)) {
                // Find recovery strategy
                RecoveryStrategy_t* strategy = NULL;
                for (uint32_t j = 0; j < num_strategies; j++) {
                    if (strategies[j].error_code == record->error_code) {
                        strategy = &strategies[j];
                        break;
                    }
                }

                if (strategy && strategy->auto_recover && !record->recovered) {
                    ESP_LOGW(TAG, "Threshold exceeded for error %d, attempting recovery", record->error_code);
                    error_recovery_attempt(record->error_code, record->context);
                }
            }
        }

        // Update system state based on error conditions
        SystemState_t new_state = error_recovery_get_system_state();
        if (new_state != current_system_state) {
            ESP_LOGI(TAG, "System state changed: %d -> %d", current_system_state, new_state);
            current_system_state = new_state;
        }

        // Send heartbeat
        watchdog_task_heartbeat();

        vTaskDelay(xMonitorInterval);
    }
}

esp_err_t error_report(ErrorCode_t error_code,
                        ErrorSeverity_t severity,
                        const char* context,
                        void* additional_data) {
    if (!error_recovery_initialized) {
        ESP_LOGW(TAG, "Error recovery not initialized, logging error directly");
        ESP_LOGE(TAG, "Error: %d, Severity: %d, Context: %s", error_code, severity, context ? context : "unknown");
        return ESP_ERR_INVALID_STATE;
    }

    if (error_code == ERROR_NONE) {
        return ESP_OK;
    }

    // Find or create error record
    ErrorRecord_t* record = NULL;
    for (uint32_t i = 0; i < ERROR_HISTORY_SIZE; i++) {
        if (error_history[i].error_code == error_code) {
            record = &error_history[i];
            break;
        }
    }

    // If not found, create new record
    if (record == NULL) {
        record = &error_history[error_history_index];
        error_history_index = (error_history_index + 1) % ERROR_HISTORY_SIZE;
        if (error_history_count < ERROR_HISTORY_SIZE) {
            error_history_count++;
        }
        memset(record, 0, sizeof(ErrorRecord_t));
        record->error_code = error_code;
        record->occurrence_count = 0;
        record->recovery_attempts = 0;
    }

    // Update error record
    record->severity = severity;
    record->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    record->occurrence_count++;
    record->context = context;
    record->additional_data = additional_data;
    record->recovered = false;

    // Update statistics
    stats.total_errors++;
    if (severity == ERROR_SEVERITY_CRITICAL) {
        stats.critical_errors++;
    } else if (severity == ERROR_SEVERITY_FATAL) {
        stats.fatal_errors++;
    }

    // Log error with appropriate level
    if (severity == ERROR_SEVERITY_FATAL) {
        ESP_LOGE(TAG, "FATAL ERROR: %d (occurrences: %lu) in %s", error_code, record->occurrence_count, context ? context : "unknown");
    } else if (severity == ERROR_SEVERITY_CRITICAL) {
        ESP_LOGE(TAG, "CRITICAL ERROR: %d (occurrences: %lu) in %s", error_code, record->occurrence_count, context ? context : "unknown");
    } else if (severity == ERROR_SEVERITY_ERROR) {
        ESP_LOGE(TAG, "ERROR: %d (occurrences: %lu) in %s", error_code, record->occurrence_count, context ? context : "unknown");
    } else if (severity == ERROR_SEVERITY_WARNING) {
        ESP_LOGW(TAG, "WARNING: %d (occurrences: %lu) in %s", error_code, record->occurrence_count, context ? context : "unknown");
    } else {
        ESP_LOGI(TAG, "INFO: %d in %s", error_code, context ? context : "unknown");
    }

    // Find recovery strategy
    RecoveryStrategy_t* strategy = NULL;
    for (uint32_t i = 0; i < num_strategies; i++) {
        if (strategies[i].error_code == error_code) {
            strategy = &strategies[i];
            break;
        }
    }

    // Attempt recovery if strategy exists and auto_recover enabled
    if (strategy && strategy->auto_recover && record->occurrence_count <= strategy->max_retries) {
        esp_err_t ret = error_recovery_attempt(error_code, context);
        if (ret == ESP_OK) {
            record->recovered = true;
            stats.recovered_errors++;
        }
    }

    // Handle fatal errors immediately
    if (severity == ERROR_SEVERITY_FATAL || severity == ERROR_SEVERITY_CRITICAL) {
        current_system_state = SYSTEM_STATE_ERROR;
        if (severity == ERROR_SEVERITY_FATAL) {
            error_recovery_force_restart(context ? context : "Fatal error", severity);
        }
    }

    return ESP_OK;
}

esp_err_t error_recovery_register_strategy(const RecoveryStrategy_t* strategy) {
    if (!error_recovery_initialized || strategy == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (num_strategies >= MAX_RECOVERY_STRATEGIES) {
        ESP_LOGE(TAG, "Maximum number of recovery strategies reached");
        return ESP_ERR_NO_MEM;
    }

    // Check if strategy already exists
    for (uint32_t i = 0; i < num_strategies; i++) {
        if (strategies[i].error_code == strategy->error_code) {
            // Update existing strategy
            strategies[i] = *strategy;
            ESP_LOGI(TAG, "Updated recovery strategy for error %d", strategy->error_code);
            return ESP_OK;
        }
    }

    // Add new strategy
    strategies[num_strategies] = *strategy;
    num_strategies++;

    ESP_LOGI(TAG, "Registered recovery strategy for error %d", strategy->error_code);
    return ESP_OK;
}

esp_err_t error_recovery_attempt(ErrorCode_t error_code, const char* context) {
    if (!error_recovery_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Find recovery strategy
    RecoveryStrategy_t* strategy = NULL;
    for (uint32_t i = 0; i < num_strategies; i++) {
        if (strategies[i].error_code == error_code) {
            strategy = &strategies[i];
            break;
        }
    }

    if (strategy == NULL) {
        ESP_LOGW(TAG, "No recovery strategy for error %d", error_code);
        return ESP_ERR_NOT_FOUND;
    }

    // Check if max retries exceeded
    ErrorRecord_t* record = NULL;
    for (uint32_t i = 0; i < ERROR_HISTORY_SIZE; i++) {
        if (error_history[i].error_code == error_code) {
            record = &error_history[i];
            if (record->recovery_attempts >= strategy->max_retries) {
                ESP_LOGW(TAG, "Max retries exceeded for error %d", error_code);
                return ESP_ERR_INVALID_STATE;
            }
            break;
        }
    }

    if (record) {
        record->recovery_attempts++;
    }
    stats.recovery_attempts++;

    // Call custom callback if set
    if (recovery_callback != NULL) {
        esp_err_t ret = recovery_callback(error_code, strategy->action, context);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Custom recovery handled error %d", error_code);
            stats.successful_recoveries++;
            return ESP_OK;
        }
    }

    // Execute recovery action
    esp_err_t result = ESP_OK;
    switch (strategy->action) {
        case RECOVERY_ACTION_RETRY:
            if (strategy->retry_delay_ms > 0) {
                vTaskDelay(pdMS_TO_TICKS(strategy->retry_delay_ms));
            }
            ESP_LOGI(TAG, "Recovery: Retry for error %d", error_code);
            break;

        case RECOVERY_ACTION_RESET_DEVICE:
            // This would need to be implemented based on specific device
            ESP_LOGI(TAG, "Recovery: Reset device for error %d", error_code);
            // TODO: Implement device reset logic
            break;

        case RECOVERY_ACTION_REINIT:
            ESP_LOGI(TAG, "Recovery: Reinitialize subsystem for error %d", error_code);
            // TODO: Implement subsystem reinit logic
            break;

        case RECOVERY_ACTION_DISABLE:
            ESP_LOGW(TAG, "Recovery: Disabling component for error %d", error_code);
            // TODO: Implement component disable logic
            break;

        case RECOVERY_ACTION_RESTART_TASK:
            ESP_LOGW(TAG, "Recovery: Restart task for error %d", error_code);
            // TODO: Implement task restart logic
            break;

        case RECOVERY_ACTION_SOFT_RESTART:
            ESP_LOGE(TAG, "Recovery: Soft restart triggered for error %d", error_code);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Give time for logging
            esp_restart();
            break;

        case RECOVERY_ACTION_HARD_RESTART:
            ESP_LOGE(TAG, "Recovery: Hard restart triggered for error %d (stopping watchdog)", error_code);
            vTaskDelay(pdMS_TO_TICKS(1000));
            watchdog_disable();
            // Watchdog will trigger reset
            break;

        case RECOVERY_ACTION_NONE:
        default:
            ESP_LOGI(TAG, "Recovery: No action for error %d", error_code);
            break;
    }

    if (result == ESP_OK) {
        stats.successful_recoveries++;
    }

    return result;
}

bool error_recovery_threshold_exceeded(ErrorCode_t error_code) {
    // Find error record
    ErrorRecord_t* record = NULL;
    for (uint32_t i = 0; i < ERROR_HISTORY_SIZE; i++) {
        if (error_history[i].error_code == error_code) {
            record = &error_history[i];
            break;
        }
    }

    if (record == NULL) {
        return false;
    }

    // Find recovery strategy
    RecoveryStrategy_t* strategy = NULL;
    for (uint32_t i = 0; i < num_strategies; i++) {
        if (strategies[i].error_code == error_code) {
            strategy = &strategies[i];
            break;
        }
    }

    if (strategy == NULL) {
        return false;
    }

    return record->occurrence_count >= strategy->escalation_threshold;
}

esp_err_t error_recovery_get_last_error(ErrorCode_t error_code,
                                         ErrorRecord_t* record) {
    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint32_t i = 0; i < ERROR_HISTORY_SIZE; i++) {
        if (error_history[i].error_code == error_code) {
            *record = error_history[i];
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

uint32_t error_recovery_get_history(ErrorRecord_t* records,
                                      uint32_t max_records) {
    if (records == NULL || max_records == 0) {
        return 0;
    }

    uint32_t count = 0;
    uint32_t records_to_copy = (max_records < error_history_count) ? max_records : error_history_count;

    // Start from most recent and work backwards
    for (uint32_t i = 0; i < records_to_copy; i++) {
        uint32_t idx = (error_history_index - 1 - i + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
        if (error_history[idx].error_code != ERROR_NONE) {
            records[count++] = error_history[idx];
        }
    }

    return count;
}

void error_recovery_get_stats(ErrorRecoveryStats_t* stats_out) {
    if (stats_out != NULL) {
        *stats_out = stats;
    }
}

void error_recovery_reset_stats(void) {
    memset(&stats, 0, sizeof(ErrorRecoveryStats_t));
}

void error_recovery_clear_history(void) {
    memset(error_history, 0, sizeof(error_history));
    error_history_count = 0;
    error_history_index = 0;
}

SystemState_t error_recovery_get_system_state(void) {
    // Check for fatal errors
    if (stats.fatal_errors > 0) {
        return SYSTEM_STATE_SHUTDOWN;
    }

    // Check for critical errors
    if (stats.critical_errors > 0) {
        return SYSTEM_STATE_ERROR;
    }

    // Check if recovery is in progress
    if (stats.recovery_attempts > stats.successful_recoveries) {
        return SYSTEM_STATE_RECOVERY;
    }

    // System is running normally
    return SYSTEM_STATE_RUNNING;
}

void error_recovery_set_callback(
    esp_err_t (*callback)(ErrorCode_t, RecoveryAction_t, const char*)) {
    recovery_callback = callback;
}

void error_recovery_force_restart(const char* reason, ErrorSeverity_t severity) {
    ESP_LOGE(TAG, "FORCING RESTART: %s (severity: %d)", reason ? reason : "Unknown", severity);
    
    stats.system_restarts++;
    current_system_state = SYSTEM_STATE_SHUTDOWN;

    vTaskDelay(pdMS_TO_TICKS(1000)); // Give time for logging

    if (severity == ERROR_SEVERITY_FATAL) {
        // Hard restart via watchdog
        watchdog_disable();
        // Watchdog will trigger reset when it times out
        vTaskDelay(pdMS_TO_TICKS(10000)); // Wait for watchdog timeout
    } else {
        // Soft restart
        esp_restart();
    }
}

