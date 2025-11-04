/*
 * error_recovery.h - Error Recovery System Prototype
 *
 * This file defines the error recovery interface for the ESP32 sensor system.
 * The error recovery system provides automatic error detection, reporting,
 * and recovery mechanisms to ensure system resilience.
 *
 * Design Principles:
 * - Centralized error tracking and reporting
 * - Automatic recovery attempts based on error type
 * - Error history tracking for debugging
 * - Escalation levels (warn -> error -> critical -> restart)
 * - Integration with watchdog for critical failures
 * - Recovery strategies per error type
 */

#ifndef ERROR_RECOVERY_H
#define ERROR_RECOVERY_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdint.h>
#include "types.h"  // For ErrorCode_t and SystemState_t

// Error severity levels
typedef enum {
    ERROR_SEVERITY_INFO = 0,      // Informational (no action needed)
    ERROR_SEVERITY_WARNING,        // Warning (log, continue)
    ERROR_SEVERITY_ERROR,          // Error (attempt recovery)
    ERROR_SEVERITY_CRITICAL,       // Critical (restart required)
    ERROR_SEVERITY_FATAL          // Fatal (immediate restart)
} ErrorSeverity_t;

// Recovery action types
typedef enum {
    RECOVERY_ACTION_NONE = 0,     // No recovery action
    RECOVERY_ACTION_RETRY,         // Retry the operation
    RECOVERY_ACTION_RESET_DEVICE,  // Reset the I2C/SPI device
    RECOVERY_ACTION_REINIT,        // Reinitialize the subsystem
    RECOVERY_ACTION_DISABLE,       // Disable the failing component
    RECOVERY_ACTION_RESTART_TASK,  // Restart the failing task
    RECOVERY_ACTION_SOFT_RESTART,  // Software restart
    RECOVERY_ACTION_HARD_RESTART   // Hardware restart (watchdog)
} RecoveryAction_t;

// Recovery strategy for an error type
typedef struct {
    ErrorCode_t error_code;        // Error code this strategy handles
    ErrorSeverity_t severity;       // Severity level
    RecoveryAction_t action;         // Recovery action to take
    uint32_t max_retries;           // Maximum recovery attempts
    uint32_t retry_delay_ms;        // Delay between retries
    uint32_t escalation_threshold;  // Escalate after N occurrences
    bool auto_recover;              // Enable automatic recovery
} RecoveryStrategy_t;

// Error record for tracking error history
typedef struct {
    ErrorCode_t error_code;         // Error code
    ErrorSeverity_t severity;        // Severity level
    uint32_t timestamp;              // Timestamp (ms since boot)
    uint32_t occurrence_count;       // How many times this error occurred
    uint32_t recovery_attempts;      // Number of recovery attempts
    bool recovered;                  // Whether recovery was successful
    const char* context;             // Context string (task name, etc.)
    void* additional_data;           // Additional error-specific data
} ErrorRecord_t;

// Error recovery statistics
typedef struct {
    uint32_t total_errors;           // Total errors recorded
    uint32_t recovered_errors;       // Errors successfully recovered
    uint32_t critical_errors;         // Critical errors encountered
    uint32_t fatal_errors;           // Fatal errors encountered
    uint32_t recovery_attempts;      // Total recovery attempts
    uint32_t successful_recoveries; // Successful recovery operations
    uint32_t system_restarts;         // Number of system restarts triggered
} ErrorRecoveryStats_t;

// Maximum error history to keep
#define ERROR_HISTORY_SIZE           32
#define MAX_RECOVERY_STRATEGIES      16
#define ERROR_CONTEXT_MAX_LEN        32

/*
 * Initialize the error recovery system
 * Must be called before any error reporting
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t error_recovery_init(void);

/*
 * Start the error recovery monitoring task
 * This task monitors error rates and handles escalation
 * 
 * @return pdPASS on success, pdFAIL on failure
 */
BaseType_t error_recovery_start_task(void);

/*
 * Report an error to the recovery system
 * This is the main entry point for error reporting
 * 
 * @param error_code: Error code from ErrorCode_t
 * @param severity: Severity level
 * @param context: Context string (task name, function, etc.)
 * @param additional_data: Optional additional error data
 * @return ESP_OK if error was handled, error code otherwise
 */
esp_err_t error_report(ErrorCode_t error_code,
                        ErrorSeverity_t severity,
                        const char* context,
                        void* additional_data);

/*
 * Register a recovery strategy for a specific error code
 * 
 * @param strategy: Recovery strategy structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t error_recovery_register_strategy(const RecoveryStrategy_t* strategy);

/*
 * Attempt to recover from an error
 * Called automatically by error_report() or manually
 * 
 * @param error_code: Error code to recover from
 * @param context: Context string for logging
 * @return ESP_OK if recovery successful, error code if failed
 */
esp_err_t error_recovery_attempt(ErrorCode_t error_code, const char* context);

/*
 * Get the last error record for a specific error code
 * 
 * @param error_code: Error code to query
 * @param record: Pointer to structure to fill
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t error_recovery_get_last_error(ErrorCode_t error_code,
                                         ErrorRecord_t* record);

/*
 * Get error history (most recent errors)
 * 
 * @param records: Array of ErrorRecord_t to fill
 * @param max_records: Maximum number of records to retrieve
 * @return Number of records returned
 */
uint32_t error_recovery_get_history(ErrorRecord_t* records,
                                      uint32_t max_records);

/*
 * Check if error threshold has been exceeded
 * Used to determine if escalation is needed
 * 
 * @param error_code: Error code to check
 * @return true if threshold exceeded, false otherwise
 */
bool error_recovery_threshold_exceeded(ErrorCode_t error_code);

/*
 * Get error recovery statistics
 * 
 * @param stats: Pointer to structure to fill with statistics
 */
void error_recovery_get_stats(ErrorRecoveryStats_t* stats);

/*
 * Reset error recovery statistics
 */
void error_recovery_reset_stats(void);

/*
 * Clear error history
 */
void error_recovery_clear_history(void);

/*
 * Get current system state based on error conditions
 * 
 * @return Current SystemState_t
 */
SystemState_t error_recovery_get_system_state(void);

/*
 * Set recovery callback function
 * Called when recovery action is about to be taken
 * Allows application to perform custom recovery logic
 * 
 * @param callback: Function pointer for recovery callback
 *                  Parameters: (ErrorCode_t, RecoveryAction_t, void* context)
 *                  Returns: ESP_OK if handled, ESP_ERR_NOT_SUPPORTED to use default
 */
void error_recovery_set_callback(
    esp_err_t (*callback)(ErrorCode_t, RecoveryAction_t, const char*));

/*
 * Force system restart based on error severity
 * 
 * @param reason: Reason string for restart
 * @param severity: Severity level (determines restart type)
 */
void error_recovery_force_restart(const char* reason, ErrorSeverity_t severity);

// Error recovery task function (internal use)
void vErrorRecoveryTask(void *pvParameters);

#endif /* ERROR_RECOVERY_H */

