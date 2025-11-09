/*
 * pcnt_flow_manager.h - PCNT Flow Sensor Manager Header
 * 
 * This header defines the PCNT Flow Sensor Manager for ESP32-C6.
 * Uses hardware PCNT (Pulse Counter) peripherals to count pulses from
 * Hall effect flow sensors with zero CPU overhead.
 * 
 * Features:
 * - Supports 3 flow sensors (PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2)
 * - Hardware-based pulse counting (no CPU overhead)
 * - 2000ms accumulation window for optimal accuracy
 * - 1000ms check interval
 * - Updates genericSens_.flowData1, flowData2, flowData3
 */

#ifndef PCNT_FLOW_MANAGER_H
#define PCNT_FLOW_MANAGER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Task Handle
// ============================================================================

/**
 * PCNT Flow Manager Task Handle
 * 
 * Task handle for the PCNT flow manager task.
 * Used for task management and monitoring.
 */
extern TaskHandle_t xPcntFlowManagerTaskHandle;

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * Initialize PCNT Flow Manager
 * 
 * Initializes the PCNT hardware and creates the flow manager task.
 * Must be called before registering sensors.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t pcnt_flow_manager_init(void);

/**
 * Register Flow Sensor
 * 
 * Registers a flow sensor with the PCNT flow manager.
 * Configures GPIO pin and PCNT unit for pulse counting.
 * 
 * @param gpio_pin GPIO pin connected to flow sensor
 * @param name Sensor name for logging (e.g., "Flow1")
 * @return ESP_OK on success, error code on failure
 */
esp_err_t pcnt_flow_manager_register_sensor(gpio_num_t gpio_pin, const char* name);

/**
 * Get Number of Registered Sensors
 * 
 * Returns the number of flow sensors currently registered.
 * 
 * @return Number of registered sensors (0-3)
 */
uint8_t pcnt_flow_manager_get_sensor_count(void);

#ifdef __cplusplus
}
#endif

#endif /* PCNT_FLOW_MANAGER_H */

