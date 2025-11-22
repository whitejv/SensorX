/*
 * task_monitor.h - Boot Button Task Monitor Header
 *
 * This file defines the boot button-triggered task monitor interface.
 * Provides FreeRTOS task statistics display via serial and MQTT JSON.
 *
 * Features:
 * - Boot button (GPIO9) triggered task statistics
 * - Serial output using vTaskList() and vTaskGetRunTimeStats()
 * - Non-intrusive (lowest priority task)
 * - Button debouncing
 */

#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <esp_err.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize task monitor
 * Configures boot button GPIO and initializes internal state
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t task_monitor_init(void);

/**
 * Start task monitor task
 * Creates the low-priority task that polls the boot button
 * 
 * @return pdPASS on success, pdFAIL on failure
 */
BaseType_t task_monitor_start_task(void);

/**
 * Print task information to serial
 * Uses vTaskList() and vTaskGetRunTimeStats() to display task statistics
 * Respects production gate (ENABLE_TASK_MONITOR_OUTPUT)
 */
void task_monitor_print_task_info(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MONITOR_H */

