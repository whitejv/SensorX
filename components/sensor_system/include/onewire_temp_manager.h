/*
 * onewire_temp_manager.h - One-Wire Temperature Sensor Manager Header
 * 
 * This header defines the One-Wire Temperature Manager for DS18B20 sensors.
 * Uses the official Espressif onewire_bus component (RMT-based).
 * 
 * Supports up to 4 DS18B20 sensors on a single One-Wire bus.
 * Reads all sensors every 5000ms with cooperative multitasking.
 */

#ifndef ONEWIRE_TEMP_MANAGER_H
#define ONEWIRE_TEMP_MANAGER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Task Handle
// ============================================================================

/**
 * One-Wire Temperature Manager Task Handle
 * 
 * Defined in onewire_temp_manager.c, declared here for external access.
 */
extern TaskHandle_t xOneWireTempManagerTaskHandle;

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * Initialize One-Wire Temperature Manager
 * 
 * Initializes the One-Wire bus, scans for DS18B20 devices, and creates
 * the temperature manager task.
 * 
 * Must be called after sensor_coordination_init().
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t onewire_temp_manager_init(void);

/**
 * Get number of discovered sensors
 * 
 * @return Number of DS18B20 sensors found (0-4)
 */
uint8_t onewire_temp_manager_get_sensor_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ONEWIRE_TEMP_MANAGER_H */

