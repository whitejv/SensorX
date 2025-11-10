/*
 * i2c_gpio_manager.h - I2C GPIO Manager Header
 *
 * This file defines the I2C GPIO Manager interface for MCP23017 GPIO expander.
 * Provides C-compatible interface for integration with C codebase.
 *
 * Features:
 * - Supports MCP23017 I2C GPIO expander
 * - Reads Port A and Port B (16 GPIOs total)
 * - Updates genericSens_.GPIO_x1 (Port A) and GPIO_x2 (Port B)
 * - 1000ms read interval
 * - Thread-safe updates with mutex protection
 */

#ifndef I2C_GPIO_MANAGER_H
#define I2C_GPIO_MANAGER_H

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

#define MCP23017_I2C_ADDRESS     0x20  // Default MCP23017 I2C address
#define MAX_GPIO_EXPANDERS       1     // Maximum GPIO expanders (typically 1)

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * Initialize I2C GPIO Manager
 * 
 * Creates the GPIO manager task and initializes state.
 * Must be called after I2C manager is initialized.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_gpio_manager_init(void);

/**
 * Register a GPIO expander device
 * 
 * Registers an MCP23017 GPIO expander for reading.
 * 
 * @param address I2C address (typically MCP23017_I2C_ADDRESS = 0x20)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_gpio_manager_register_expander(uint8_t address);

/**
 * Get number of registered GPIO expanders
 * 
 * @return Number of registered devices (0-1)
 */
uint8_t i2c_gpio_manager_get_device_count(void);

/**
 * Check if device is present
 * 
 * @param address I2C address to check
 * @return true if device is present and enabled, false otherwise
 */
bool i2c_gpio_manager_is_device_present(uint8_t address);

#ifdef __cplusplus
}
#endif

#endif /* I2C_GPIO_MANAGER_H */

