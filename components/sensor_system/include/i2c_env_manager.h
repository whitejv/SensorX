/*
 * i2c_env_manager.h - I2C Environmental Sensor Manager Header
 *
 * This file defines the I2C Environmental Sensor Manager interface for the ESP32 sensor system.
 * Manages BME280 environmental sensors (temperature, humidity, pressure).
 *
 * Features:
 * - Supports BME280 I2C environmental sensor
 * - Reads temperature, humidity, and pressure every 5000ms
 * - Updates genericSens_.tempx, humidity, pressurex
 * - Thread-safe updates with mutex protection
 * - Watchdog protection
 * - Error handling and I2C error tracking
 */

#ifndef I2C_ENV_MANAGER_H
#define I2C_ENV_MANAGER_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * Initialize I2C Environmental Manager
 * Creates the environmental sensor manager task.
 * Must be called before registering sensors.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_env_manager_init(void);

/*
 * Register BME280 Environmental Sensor
 * Registers a BME280 sensor with the I2C environmental manager.
 * Configures I2C device and initializes sensor.
 * 
 * @param address: I2C address of BME280 (typically 0x76 or 0x77)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_env_manager_register_bme280(uint8_t address);

/*
 * Get number of registered environmental sensors
 * 
 * @return Number of registered sensors (0 or 1 for BME280)
 */
uint8_t i2c_env_manager_get_sensor_count(void);

/*
 * Check if BME280 sensor is present and functioning
 * 
 * @return true if sensor is present and responding, false otherwise
 */
bool i2c_env_manager_is_bme280_present(void);

#endif /* I2C_ENV_MANAGER_H */

