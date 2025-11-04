/*
 * i2c_manager.h - I2C Bus Manager Header
 *
 * This file defines the I2C bus management interface for the ESP32 sensor system.
 * Provides I2C bus initialization and management for sensor communication.
 *
 * Design Principles:
 * - Simple initialization function
 * - ESP-IDF I2C master driver
 * - Error handling integrated with error recovery system
 * - Configuration from config.h (pins, speed, timeout)
 */

#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <esp_err.h>
#include <stdbool.h>
#include <driver/i2c_master.h>

// I2C bus handle (extern for access by sensor drivers)
extern i2c_master_bus_handle_t i2c_bus_handle;

/*
 * Initialize I2C bus manager
 * Configures I2C bus with pins, speed, and timeout from config.h
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_manager_init(void);

/*
 * Deinitialize I2C bus manager
 * Cleanup function (optional, for shutdown scenarios)
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_manager_deinit(void);

/*
 * Check if I2C manager is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool i2c_manager_is_initialized(void);

/*
 * Get I2C bus handle
 * Returns the bus handle for use by sensor drivers
 * 
 * @return I2C bus handle, or NULL if not initialized
 */
i2c_master_bus_handle_t i2c_manager_get_bus_handle(void);

/*
 * Scan I2C bus for devices
 * Scans addresses 0x08-0x77 and returns list of found addresses
 * 
 * @param addresses: Array to store found addresses (must be at least 16 elements)
 * @param max_count: Maximum number of addresses to return
 * @param found_count: Pointer to store number of devices found
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_manager_scan_devices(uint8_t* addresses, size_t max_count, size_t* found_count);

#endif /* I2C_MANAGER_H */

