/*
 * i2c_adc_manager.h - I2C ADC Manager Header
 *
 * This file defines the I2C ADC Manager interface for ADS1015 and ADS1115 devices.
 * Provides C-compatible interface for integration with C codebase.
 *
 * Features:
 * - Supports ADS1015 (12-bit) and ADS1115 (16-bit) ADCs
 * - Reads all 4 channels from each device
 * - Updates genericSens_.adc_x1 through adc_x8
 * - 1000ms read interval
 * - Thread-safe updates with mutex protection
 */

#ifndef I2C_ADC_MANAGER_H
#define I2C_ADC_MANAGER_H

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

#define ADS1015_I2C_ADDRESS     0x48  // 12-bit ADC address
#define ADS1115_I2C_ADDRESS     0x49  // 16-bit ADC address
#define MAX_ADC_DEVICES         2     // Maximum ADC devices (ADS1015 + ADS1115)
#define ADC_CHANNELS_PER_DEVICE 4     // Channels per ADC device

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * Initialize I2C ADC Manager
 * 
 * Creates the ADC manager task and initializes state.
 * Must be called after I2C manager is initialized.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_adc_manager_init(void);

/**
 * Register an ADC device
 * 
 * Registers an ADS1015 or ADS1115 device for reading.
 * 
 * @param address I2C address (ADS1015_I2C_ADDRESS or ADS1115_I2C_ADDRESS)
 * @param is_ads1115 true for ADS1115, false for ADS1015
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_adc_manager_register_device(uint8_t address, bool is_ads1115);

/**
 * Get number of registered ADC devices
 * 
 * @return Number of registered devices (0-2)
 */
uint8_t i2c_adc_manager_get_device_count(void);

/**
 * Check if device is present
 * 
 * @param address I2C address to check
 * @return true if device is present and enabled, false otherwise
 */
bool i2c_adc_manager_is_device_present(uint8_t address);

#ifdef __cplusplus
}
#endif

#endif /* I2C_ADC_MANAGER_H */

