/*
 * onewire_temp_manager.c - One-Wire Temperature Sensor Manager Implementation
 * 
 * This file implements the One-Wire Temperature Manager for DS18B20 sensors.
 * Uses the official Espressif onewire_bus component (RMT-based hardware timing).
 * 
 * Features:
 * - Supports up to 4 DS18B20 sensors on GPIO6
 * - Reads all sensors every 5000ms
 * - Cooperative multitasking with yields between sensors
 * - Updates genericSens_.temp1-temp4
 * - Watchdog protection
 * - Error handling and retry logic
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "onewire_bus.h"           // Official Espressif One-Wire component
#include "onewire_device.h"        // Device iterator
#include "onewire_cmd.h"            // Command definitions
#include "onewire_crc.h"            // CRC functions
#include "onewire_temp_manager.h"
#include "config.h"
#include "pins.h"
#include "sensor_coordination.h"
#include "sensor.h"
#include "watchdog.h"

static const char *TAG = "ONEWIRE_TEMP";

// ============================================================================
// Task Handle
// ============================================================================

TaskHandle_t xOneWireTempManagerTaskHandle = NULL;

// ============================================================================
// One-Wire Manager State
// ============================================================================

static onewire_bus_handle_t onewire_bus = NULL;
static onewire_device_t devices[ONEWIRE_MAX_SENSORS];
static uint8_t num_sensors = 0;
static bool initialized = false;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Convert Celsius to Fahrenheit
 */
static float celsius_to_fahrenheit(float celsius) {
    return (celsius * 9.0f / 5.0f) + 32.0f;
}

/**
 * Select a device by sending MATCH_ROM command followed by device address
 */
static esp_err_t onewire_select_device(onewire_bus_handle_t bus, onewire_device_t *device) {
    esp_err_t ret;
    uint8_t cmd = ONEWIRE_CMD_MATCH_ROM;  // 0x55
    
    // Send MATCH_ROM command
    ret = onewire_bus_write_bytes(bus, &cmd, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Send device address (8 bytes, LSB first)
    uint8_t addr_bytes[8];
    uint64_t addr = device->address;
    for (int i = 0; i < 8; i++) {
        addr_bytes[i] = (uint8_t)(addr & 0xFF);
        addr >>= 8;
    }
    
    ret = onewire_bus_write_bytes(bus, addr_bytes, 8);
    return ret;
}

/**
 * Read temperature from DS18B20 sensor
 * 
 * @param device One-Wire device handle
 * @param temp_fahrenheit Output temperature in Fahrenheit
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t ds18b20_read_temperature(onewire_device_t *device, float *temp_fahrenheit) {
    uint8_t scratchpad[9];
    esp_err_t ret;
    
    // Step 1: Reset bus
    ret = onewire_bus_reset(device->bus);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to reset bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Step 2: Select device and start conversion
    ret = onewire_select_device(device->bus, device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint8_t convert_cmd = 0x44;  // CONVERT_T command
    ret = onewire_bus_write_bytes(device->bus, &convert_cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send CONVERT_T: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Step 3: Wait for conversion (750ms for 12-bit resolution)
    vTaskDelay(pdMS_TO_TICKS(ONEWIRE_CONVERSION_DELAY_MS));
    
    // Step 4: Reset and select device again
    ret = onewire_bus_reset(device->bus);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to reset bus for read: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = onewire_select_device(device->bus, device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select device for read: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Step 5: Read scratchpad (9 bytes)
    uint8_t read_cmd = 0xBE;  // READ_SCRATCHPAD command
    ret = onewire_bus_write_bytes(device->bus, &read_cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send READ_SCRATCHPAD: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = onewire_bus_read_bytes(device->bus, scratchpad, 9);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read scratchpad: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Step 6: Verify CRC
    uint8_t crc = onewire_crc8(0, scratchpad, 8);
    if (crc != scratchpad[8]) {
        ESP_LOGW(TAG, "CRC error: calculated=0x%02X, received=0x%02X", crc, scratchpad[8]);
        return ESP_FAIL;
    }
    
    // Step 7: Convert to temperature
    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    float temp_celsius = raw_temp / 16.0f;
    *temp_fahrenheit = celsius_to_fahrenheit(temp_celsius);
    
    return ESP_OK;
}

// ============================================================================
// One-Wire Temperature Manager Task
// ============================================================================

/**
 * One-Wire Temperature Manager Task
 * 
 * Reads all discovered DS18B20 sensors every 5000ms.
 * Implements cooperative multitasking with yields between sensor reads.
 */
void vOneWireTempManagerTask(void *pvParameters) {
    (void)pvParameters;
    
    ESP_LOGI(TAG, "One-Wire Temperature Manager Task started");
    
    // Register with watchdog
    watchdog_register_current_task("OneWireTempMgr", ONEWIRE_TEMP_TASK_INTERVAL_MS + 1000);
    
    const TickType_t xTaskInterval = pdMS_TO_TICKS(ONEWIRE_TEMP_TASK_INTERVAL_MS);
    const TickType_t xYieldBetweenSensors = pdMS_TO_TICKS(ONEWIRE_YIELD_BETWEEN_MS);
    
    for (;;) {
        uint32_t cycle_start = xTaskGetTickCount();
        
        // Read all sensors sequentially within one cycle
        for (uint8_t sensor_index = 0; sensor_index < num_sensors; sensor_index++) {
            uint32_t sensor_start = xTaskGetTickCount();
            
            float temp_fahrenheit = 0.0f;
            esp_err_t ret = ds18b20_read_temperature(&devices[sensor_index], &temp_fahrenheit);
            
            if (ret == ESP_OK) {
                // Update genericSens_ structure (mutex-protected)
                if (sensor_data.mutex != NULL) {
                    if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        switch (sensor_index) {
                            case 0:
                                genericSens_.generic.temp1 = (int32_t)temp_fahrenheit;
                                // Store float value in temp1_f (as int32_t bit pattern)
                                memcpy(&genericSens_.generic.temp1_f, &temp_fahrenheit, sizeof(float));
                                break;
                            case 1:
                                genericSens_.generic.temp2 = temp_fahrenheit;
                                break;
                            case 2:
                                genericSens_.generic.temp3 = temp_fahrenheit;
                                break;
                            case 3:
                                genericSens_.generic.temp4 = temp_fahrenheit;
                                break;
                        }
                        xSemaphoreGive(sensor_data.mutex);
                        
                        ESP_LOGD(TAG, "Sensor %d: %.1f°F", sensor_index, temp_fahrenheit);
                    } else {
                        ESP_LOGW(TAG, "Mutex timeout updating sensor %d", sensor_index);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Failed to read sensor %d: %s", sensor_index, esp_err_to_name(ret));
            }
            
            // Monitor per-sensor read time
            uint32_t sensor_elapsed = (xTaskGetTickCount() - sensor_start) * portTICK_PERIOD_MS;
            if (sensor_elapsed > ONEWIRE_READ_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Sensor %d read took %lu ms (exceeded timeout)", sensor_index, sensor_elapsed);
            }
            
            // YIELD POINT: Small yield between sensors (except after last sensor)
            // Allows other same-priority tasks brief execution window
            if (sensor_index < num_sensors - 1) {
                vTaskDelay(xYieldBetweenSensors);
            }
            
            // Watchdog heartbeat after each sensor read
            watchdog_task_heartbeat();
        }
        
        // Calculate total cycle time
        uint32_t cycle_elapsed = (xTaskGetTickCount() - cycle_start) * portTICK_PERIOD_MS;
        if (cycle_elapsed > ONEWIRE_CYCLE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Complete cycle exceeded timeout: %lu ms", cycle_elapsed);
        }
        
        // Signal completion (optional, for 5000ms sensor coordination)
        sensor_coordination_signal_completion(SENSOR_EVENT_TEMP_COMPLETE);
        
        // YIELD POINT: Main task delay until next cycle
        // This is the primary yield - allows all other tasks to execute
        watchdog_task_heartbeat();
        vTaskDelay(xTaskInterval);
    }
}

// ============================================================================
// Initialization Functions
// ============================================================================

/**
 * Initialize One-Wire Temperature Manager
 * 
 * Initializes the One-Wire bus using RMT peripheral, scans for DS18B20
 * devices, and creates the temperature manager task.
 */
esp_err_t onewire_temp_manager_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "One-Wire Temperature Manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing One-Wire Temperature Manager...");
    
    // Initialize One-Wire bus with RMT peripheral
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = PIN_ONEWIRE_TEMP,  // GPIO6
        .flags = {
            .en_pull_up = true,  // Enable internal pull-up resistor (matches ESP-IDF example)
        },
    };
    
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,  // 1byte ROM command + 8byte ROM number + 1byte device command (matches ESP-IDF example)
    };
    
    esp_err_t ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &onewire_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create One-Wire bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "One-Wire bus initialized on GPIO%d", PIN_ONEWIRE_TEMP);
    
    // Scan for devices using iterator (matches ESP-IDF example pattern)
    onewire_device_iter_handle_t iter = NULL;
    ret = onewire_new_device_iter(onewire_bus, &iter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create device iterator: %s", esp_err_to_name(ret));
        onewire_bus_del(onewire_bus);
        onewire_bus = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    
    // Enumerate devices (matches ESP-IDF example pattern)
    num_sensors = 0;
    onewire_device_t device;
    esp_err_t search_result = ESP_OK;
    do {
        search_result = onewire_device_iter_get_next(iter, &device);
        if (search_result == ESP_OK) {
            // Found a device - store it
            devices[num_sensors].bus = device.bus;
            devices[num_sensors].address = device.address;
            ESP_LOGI(TAG, "Found device[%d], address: %016llX", num_sensors, (unsigned long long)device.address);
            num_sensors++;
            if (num_sensors >= ONEWIRE_MAX_SENSORS) {
                ESP_LOGI(TAG, "Max device number reached, stop searching...");
                break;
            }
        } else if (search_result == ESP_ERR_NOT_FOUND) {
            // No more devices - this is normal
            break;
        } else {
            ESP_LOGW(TAG, "Error enumerating device: %s", esp_err_to_name(search_result));
            break;
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    
    // Clean up iterator
    onewire_del_device_iter(iter);
    
    ESP_LOGI(TAG, "Found %d DS18B20 sensor(s)", num_sensors);
    
    if (num_sensors == 0) {
        ESP_LOGW(TAG, "No One-Wire devices found - task will still be created but won't read sensors");
    }
    
    // Update genericSens_ with sensor count
    if (sensor_data.mutex != NULL) {
        if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            genericSens_.generic.tempSensorcount = num_sensors;
            xSemaphoreGive(sensor_data.mutex);
        }
    }
    
    // Create temperature manager task
    BaseType_t xResult = xTaskCreate(
        vOneWireTempManagerTask,
        "OneWireTempMgr",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_BACKGROUND,  // Lowest priority
        &xOneWireTempManagerTaskHandle
    );
    
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create One-Wire Temperature Manager task");
        onewire_bus_del(onewire_bus);
        onewire_bus = NULL;
        return ESP_FAIL;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "One-Wire Temperature Manager initialized successfully");
    
    return ESP_OK;
}

/**
 * Get number of discovered sensors
 */
uint8_t onewire_temp_manager_get_sensor_count(void) {
    return num_sensors;
}
