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
#include <stdint.h>
#include <driver/i2c_master.h>

// I2C bus handle (extern for access by sensor drivers)
extern i2c_master_bus_handle_t i2c_bus_handle;

// I2C Error Statistics Structure
typedef struct {
    uint32_t total_errors;           // Total I2C errors
    uint32_t timeout_errors;         // I2C timeout errors
    uint32_t nack_errors;            // I2C NACK errors
    uint32_t bus_errors;             // I2C bus errors
    uint32_t device_not_found;       // Device probe failures
    uint32_t transaction_failures;   // Transaction failures
    uint32_t last_error_time_ms;     // Timestamp of last error (ms since boot)
} I2CStats_t;

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

/*
 * Record an I2C error for statistics tracking
 * Categorizes the error type and increments appropriate counters
 * 
 * @param error_code: ESP error code from I2C operation
 */
void i2c_manager_record_error(esp_err_t error_code);

/*
 * Get I2C error statistics
 * Returns current error counts and last error timestamp
 * 
 * @param stats: Pointer to I2CStats_t structure to fill
 */
void i2c_manager_get_stats(I2CStats_t* stats);

/*
 * Reset I2C error statistics
 * Clears all error counters (useful for testing or after recovery)
 */
void i2c_manager_reset_stats(void);

#endif /* I2C_MANAGER_H */

