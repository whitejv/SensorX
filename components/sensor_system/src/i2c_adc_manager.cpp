/*
 * i2c_adc_manager.cpp - I2C ADC Manager Implementation (C++)
 *
 * This file implements the I2C ADC Manager for ADS1015 and ADS1115 devices.
 * Uses the espp::Ads1x15 C++ component for ADC communication.
 *
 * Features:
 * - Supports ADS1015 (12-bit) and ADS1115 (16-bit) ADCs
 * - Reads all 4 channels from each device every 1000ms
 * - Updates genericSens_.adc_x1 through adc_x8
 * - Thread-safe updates with mutex protection
 * - Watchdog protection
 * - Error handling and retry logic
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <system_error>

#include <driver/i2c_master.h>

#include "ads1x15.hpp"              // espp component

// C headers must be wrapped in extern "C" for C++ linkage
extern "C" {
#include "i2c_adc_manager.h"        // C interface header
#include "i2c_manager.h"            // I2C bus manager
#include "sensor_coordination.h"    // Event group, mutex
#include "sensor.h"                 // genericSens_ structure
#include "config.h"                 // Configuration constants
#include "watchdog.h"               // Watchdog registration
}

static const char *TAG = "I2C_ADC";

// ============================================================================
// Task Handle
// ============================================================================

TaskHandle_t xI2cAdcManagerTaskHandle = NULL;

// ============================================================================
// ADC Device State Structure
// ============================================================================

namespace {
    struct AdcDeviceState {
        espp::Ads1x15* adc_device;              // C++ ADC object
        i2c_master_dev_handle_t i2c_device_handle; // I2C device handle
        uint8_t i2c_address;                     // I2C address (0x48 or 0x49)
        bool is_ads1115;                          // true = ADS1115, false = ADS1015
        bool present;                             // Device detected
        bool enabled;                             // Device enabled
    };
    
    // Manager State
    static AdcDeviceState adc_devices[MAX_ADC_DEVICES];
    static uint8_t num_devices = 0;
    static bool initialized = false;
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Create I2C device handle and ADC instance
 */
static esp_err_t create_adc_device(uint8_t address, bool is_ads1115) {
    // Get I2C bus handle from existing manager
    i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus_handle();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create I2C device handle for this ADC
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_CLOCK_SPEED,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };
    
    i2c_master_dev_handle_t i2c_dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s", address, esp_err_to_name(ret));
        return ret;
    }
    
    // Verify device is present by probing
    ret = i2c_master_probe(bus_handle, address, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Device at 0x%02X not responding to probe: %s", address, esp_err_to_name(ret));
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Small delay to ensure device is ready
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Create lambda functions for I2C read/write
    // These capture the device handle and wrap ESP-IDF I2C master driver
    auto write_fn = [i2c_dev_handle](uint8_t addr, const uint8_t *data, size_t len) -> bool {
        esp_err_t ret = i2c_master_transmit(i2c_dev_handle, data, len, I2C_TIMEOUT_MS);
        return (ret == ESP_OK);
    };
    
    auto read_fn = [i2c_dev_handle](uint8_t addr, uint8_t *data, size_t len) -> bool {
        esp_err_t ret = i2c_master_receive(i2c_dev_handle, data, len, I2C_TIMEOUT_MS);
        return (ret == ESP_OK);
    };
    
    // Create ADC instance based on device type
    espp::Ads1x15* adc = nullptr;
    
    if (is_ads1115) {
        // ADS1115 configuration
        espp::Ads1x15::Ads1115Config config = {
            .device_address = address,
            .write = write_fn,
            .read = read_fn,
            .gain = espp::Ads1x15::Gain::TWOTHIRDS,  // Default gain (matches Arduino)
            .sample_rate = espp::Ads1x15::Ads1115Rate::SPS128,  // Default rate
            .log_level = espp::Logger::Verbosity::WARN,
        };
        adc = new espp::Ads1x15(config);
    } else {
        // ADS1015 configuration
        espp::Ads1x15::Ads1015Config config = {
            .device_address = address,
            .write = write_fn,
            .read = read_fn,
            .gain = espp::Ads1x15::Gain::TWOTHIRDS,  // Default gain (matches Arduino)
            .sample_rate = espp::Ads1x15::Ads1015Rate::SPS1600,  // Default rate
            .log_level = espp::Logger::Verbosity::WARN,
        };
        adc = new espp::Ads1x15(config);
    }
    
    if (adc == nullptr) {
        ESP_LOGE(TAG, "Failed to create Ads1x15 instance");
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ESP_ERR_NO_MEM;
    }
    
    // Test device communication by attempting to read
    // The espp component doesn't have an explicit init() method, so we test by reading
    // Add a small delay before first communication attempt
    vTaskDelay(pdMS_TO_TICKS(50));
    
    std::error_code ec;
    (void)adc->sample_mv(0, ec);  // Test read (ignore result)
    if (ec) {
        // Safely get error message - store in variable to ensure string lifetime
        std::string error_msg = ec.message();
        ESP_LOGE(TAG, "Failed to communicate with ADC at 0x%02X: %s", address, error_msg.c_str());
        delete adc;
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Store device state
    adc_devices[num_devices].adc_device = adc;
    adc_devices[num_devices].i2c_device_handle = i2c_dev_handle;
    adc_devices[num_devices].i2c_address = address;
    adc_devices[num_devices].is_ads1115 = is_ads1115;
    adc_devices[num_devices].present = true;
    adc_devices[num_devices].enabled = true;
    
    num_devices++;
    
    ESP_LOGI(TAG, "%s initialized at address 0x%02X", 
             is_ads1115 ? "ADS1115" : "ADS1015", address);
    
    return ESP_OK;
}

// ============================================================================
// Task Function
// ============================================================================

static void vI2cAdcManagerTask(void *pvParameters) {
    ESP_LOGI(TAG, "I2C ADC Manager task started");
    
    // Register with watchdog
    watchdog_register_current_task("I2C_ADC_Mgr", I2C_ADC_TASK_INTERVAL_MS + 500);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_interval = pdMS_TO_TICKS(I2C_ADC_TASK_INTERVAL_MS);
    
    while (1) {
        // Local array to store ADC readings (outside mutex)
        // Index 0-3: ADS1015 channels (adc_x1 through adc_x4)
        // Index 4-7: ADS1115 channels (adc_x5 through adc_x8)
        float adc_readings[8] = {0.0f};
        
        // PHASE 1: Read all ADC channels (OUTSIDE mutex to minimize mutex hold time)
        // This is the time-consuming I2C operation that was previously blocking the mutex
        for (uint8_t i = 0; i < num_devices; i++) {
            if (!adc_devices[i].enabled || !adc_devices[i].present) {
                continue;
            }
            
            espp::Ads1x15* adc = adc_devices[i].adc_device;
            
            // Safety check: ensure ADC device pointer is valid
            if (adc == nullptr) {
                ESP_LOGW(TAG, "ADC device at 0x%02X has null pointer, skipping", 
                        adc_devices[i].i2c_address);
                continue;
            }
            
            uint8_t base_field_index;
            
            // Determine base field index (adc_x1 for ADS1015, adc_x5 for ADS1115)
            if (adc_devices[i].is_ads1115) {
                base_field_index = 5;  // adc_x5 through adc_x8 (array index 4-7)
            } else {
                base_field_index = 1;  // adc_x1 through adc_x4 (array index 0-3)
            }
            
            // Read all 4 channels for this device
            for (uint8_t channel = 0; channel < ADC_CHANNELS_PER_DEVICE; channel++) {
                float voltage = 0.0f;
                std::error_code ec;
                bool read_success = false;
                
                // Retry logic for I2C transaction failures
                const uint8_t max_retries = 3;
                for (uint8_t retry = 0; retry < max_retries; retry++) {
                    // Read voltage in millivolts, then convert to volts
                    float voltage_mv = adc->sample_mv(channel, ec);
                    
                    if (!ec) {
                        // Success - convert to volts
                        voltage = voltage_mv / 1000.0f;
                        read_success = true;
                        break;
                    }
                    
                    // Error occurred - log and retry if not last attempt
                    if (retry < (max_retries - 1)) {
                        // Safely get error message
                        std::string error_msg = ec.message();
                        ESP_LOGW(TAG, "Error reading %s channel %d at 0x%02X (retry %d/%d): %s",
                                adc_devices[i].is_ads1115 ? "ADS1115" : "ADS1015",
                                channel, adc_devices[i].i2c_address, 
                                retry + 1, max_retries, error_msg.c_str());
                        
                        // Exponential backoff: 5ms, 10ms, 20ms
                        vTaskDelay(pdMS_TO_TICKS(5 * (1 << retry)));
                        watchdog_task_heartbeat();
                    }
                }
                
                // If all retries failed, log final error and use 0.0
                if (!read_success) {
                    std::string error_msg = ec.message();
                    ESP_LOGW(TAG, "Failed to read %s channel %d at 0x%02X after %d retries: %s",
                            adc_devices[i].is_ads1115 ? "ADS1115" : "ADS1015",
                            channel, adc_devices[i].i2c_address, max_retries, error_msg.c_str());
                    
                    // Record I2C error for statistics
                    // Note: std::error_code doesn't map directly to ESP error codes,
                    // but we can infer timeout or transaction failure from context
                    // Most I2C errors from espp library are timeout or NACK related
                    i2c_manager_record_error(ESP_ERR_TIMEOUT);  // Most common I2C error
                    
                    voltage = 0.0f;  // Use 0.0 on error
                    
                    // Longer delay after failure to let device recover
                    vTaskDelay(pdMS_TO_TICKS(10));
                    watchdog_task_heartbeat();
                }
                
                // Store reading in local array (base_field_index is 1-based, array is 0-based)
                uint8_t array_index = base_field_index - 1 + channel;
                if (array_index < 8) {
                    adc_readings[array_index] = voltage;
                }
                
                // Small delay/yield between channel reads to reduce I2C bus stress
                // and improve reliability (especially for channel 3)
                // Increased delay to 10ms to allow device recovery time
                // Delay applies after all channels including channel 3
                vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay between channels
                // Feed watchdog during long operations
                watchdog_task_heartbeat();
            }
            
            // Feed watchdog after each device to prevent timeout during long operations
            watchdog_task_heartbeat();
        }
        
        // PHASE 2: Update genericSens_ structure (INSIDE mutex - quick operation)
        // This should be very fast (< 1ms) since we're just copying values
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            // Copy all readings from local array to genericSens_ structure
            genericSens_.generic.adc_x1 = adc_readings[0];
            genericSens_.generic.adc_x2 = adc_readings[1];
            genericSens_.generic.adc_x3 = adc_readings[2];
            genericSens_.generic.adc_x4 = adc_readings[3];
            genericSens_.generic.adc_x5 = adc_readings[4];
            genericSens_.generic.adc_x6 = adc_readings[5];
            genericSens_.generic.adc_x7 = adc_readings[6];
            genericSens_.generic.adc_x8 = adc_readings[7];
            
            // Release mutex immediately after update
            xSemaphoreGive(sensor_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take sensor data mutex");
        }
        
        // Signal completion for MQTT publisher coordination
        sensor_coordination_signal_completion(SENSOR_EVENT_ADC_COMPLETE);
        
        // Watchdog heartbeat
        watchdog_task_heartbeat();
        
        // Delay until next cycle
        vTaskDelayUntil(&last_wake_time, task_interval);
    }
}

// ============================================================================
// C Interface Functions
// ============================================================================

extern "C" esp_err_t i2c_adc_manager_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "I2C ADC Manager already initialized");
        return ESP_OK;
    }
    
    // Verify I2C manager is initialized
    if (!i2c_manager_is_initialized()) {
        ESP_LOGE(TAG, "I2C manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize device state
    memset(adc_devices, 0, sizeof(adc_devices));
    num_devices = 0;
    
    // Create task
    BaseType_t ret = xTaskCreate(
        vI2cAdcManagerTask,
        "I2C_ADC_Mgr",
        TASK_STACK_SIZE_FIXED_FREQ,
        NULL,
        TASK_PRIORITY_FIXED_FREQ,
        &xI2cAdcManagerTaskHandle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I2C ADC Manager task");
        return ESP_ERR_NO_MEM;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "I2C ADC Manager initialized");
    
    return ESP_OK;
}

extern "C" esp_err_t i2c_adc_manager_register_device(uint8_t address, bool is_ads1115) {
    if (!initialized) {
        ESP_LOGE(TAG, "I2C ADC Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (num_devices >= MAX_ADC_DEVICES) {
        ESP_LOGE(TAG, "Maximum ADC devices reached");
        return ESP_ERR_NO_MEM;
    }
    
    // Validate address
    if (address != ADS1015_I2C_ADDRESS && address != ADS1115_I2C_ADDRESS) {
        ESP_LOGE(TAG, "Invalid ADC address: 0x%02X", address);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate address matches device type
    if (is_ads1115 && address != ADS1115_I2C_ADDRESS) {
        ESP_LOGE(TAG, "ADS1115 must use address 0x%02X", ADS1115_I2C_ADDRESS);
        return ESP_ERR_INVALID_ARG;
    }
    if (!is_ads1115 && address != ADS1015_I2C_ADDRESS) {
        ESP_LOGE(TAG, "ADS1015 must use address 0x%02X", ADS1015_I2C_ADDRESS);
        return ESP_ERR_INVALID_ARG;
    }
    
    return create_adc_device(address, is_ads1115);
}

extern "C" uint8_t i2c_adc_manager_get_device_count(void) {
    return num_devices;
}

extern "C" bool i2c_adc_manager_is_device_present(uint8_t address) {
    for (uint8_t i = 0; i < num_devices; i++) {
        if (adc_devices[i].i2c_address == address && 
            adc_devices[i].present && 
            adc_devices[i].enabled) {
            return true;
        }
    }
    return false;
}

