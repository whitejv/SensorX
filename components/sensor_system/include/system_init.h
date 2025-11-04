/*
 * system_init.h - System Task Header
 *
 * This file contains task declarations for the ESP32 FreeRTOS
 * sensor system, including heartbeat, monitoring, and information tasks.
 */

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handles for heartbeat, information, and monitoring tasks
extern TaskHandle_t xHeartbeatTaskHandle;
extern TaskHandle_t xSystemInfoTaskHandle;
extern TaskHandle_t xSystemMonitorTaskHandle;

// Function declarations
void vHeartbeatTask(void *pvParameters);
void vSystemInfoTask(void *pvParameters);
void vSystemMonitorTask(void *pvParameters);
void vInfoResponseTask(void *pvParameters);

#endif /* SYSTEM_INIT_H */
