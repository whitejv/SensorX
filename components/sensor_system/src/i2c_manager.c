/*
 * i2c_manager.c - I2C Bus Manager Implementation
 *
 * This file implements the I2C bus management system for the ESP32 sensor system.
 * Provides I2C bus initialization and management using ESP-IDF I2C master driver.
 */

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <string.h>

#include "i2c_manager.h"
#include "config.h"
#include "pins.h"
#include "error_recovery.h"
#include "watchdog.h"

static const char *TAG = "I2C_MANAGER";

// I2C bus handle (accessible by sensor drivers)
i2c_master_bus_handle_t i2c_bus_handle = NULL;

// Initialization status
static bool i2c_manager_initialized = false;

// I2C error statistics
static I2CStats_t i2c_stats = {0};

esp_err_t i2c_manager_init(void) {
    if (i2c_manager_initialized) {
        ESP_LOGW(TAG, "I2C manager already initialized");
        return ESP_OK;
    }

    // Enable power to STEMMA QT connector (GPIO20 must be HIGH)
    // This pin also controls NeoPixel power
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO20 for STEMMA QT power: %s", esp_err_to_name(ret));
        error_report(ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, "i2c_manager_init", NULL);
        return ret;
    }
    
    // Set GPIO20 HIGH to enable power to STEMMA QT connector
    ret = gpio_set_level(PIN_EN, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable STEMMA QT power: %s", esp_err_to_name(ret));
        error_report(ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, "i2c_manager_init", NULL);
        return ret;
    }
    ESP_LOGI(TAG, "STEMMA QT connector power enabled (GPIO20 HIGH)");
    
    // Give devices time to power up after enabling STEMMA QT power
    // Some devices need time to stabilize after power is applied
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms delay for power stabilization

    // I2C bus configuration
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,  // Enable internal pullups as backup (STEMMA QT has external, but this helps)
        }
    };

    // Initialize I2C master bus
    ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        error_report(ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, "i2c_manager_init", NULL);
        return ret;
    }

    // Note: Individual device configurations (speed, timeout) are set when adding devices
    // The bus itself is initialized with default settings
    // Device-specific configurations (speed, timeout) are handled per device

    i2c_manager_initialized = true;
    ESP_LOGI(TAG, "I2C manager initialized on port %d (SDA: GPIO%d, SCL: GPIO%d)",
             I2C_NUM_0, PIN_I2C_SDA, PIN_I2C_SCL);
    ESP_LOGI(TAG, "I2C configured for %d kHz, %d ms timeout",
             I2C_CLOCK_SPEED / 1000, I2C_TIMEOUT_MS);

    return ESP_OK;
}

esp_err_t i2c_manager_deinit(void) {
    if (!i2c_manager_initialized) {
        ESP_LOGW(TAG, "I2C manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (i2c_bus_handle != NULL) {
        esp_err_t ret = i2c_del_master_bus(i2c_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
            return ret;
        }
        i2c_bus_handle = NULL;
    }

    i2c_manager_initialized = false;
    ESP_LOGI(TAG, "I2C manager deinitialized");
    return ESP_OK;
}

bool i2c_manager_is_initialized(void) {
    return i2c_manager_initialized;
}

i2c_master_bus_handle_t i2c_manager_get_bus_handle(void) {
    return i2c_bus_handle;
}

esp_err_t i2c_manager_scan_devices(uint8_t* addresses, size_t max_count, size_t* found_count) {
    if (!i2c_manager_initialized || i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C manager not initialized or bus handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    if (addresses == NULL || max_count == 0 || found_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *found_count = 0;

    // Temporarily suppress ALL I2C driver messages during scanning
    // Errors are expected when probing addresses without devices - this is normal behavior
    esp_log_level_t old_level = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);  // Completely suppress all I2C driver logs during scan
    
    // Scan I2C addresses 0x08-0x77 (standard I2C address range)
    // Addresses 0x00-0x07 are reserved
    // Match ESP-IDF i2c_tools example implementation:
    // - Use 50ms timeout (I2C_TOOL_TIMEOUT_VALUE_MS) for reliable detection
    // - No delay between probes (just like i2c_tools example)
    // - Timeout passed directly in milliseconds (not pdMS_TO_TICKS)
    // - Simple error handling: ESP_OK = found, ESP_ERR_TIMEOUT/ESP_ERR_NOT_FOUND = no device
    const uint32_t scan_timeout_ms = 50;  // Match ESP-IDF i2c_tools example (50ms)
    
    for (uint8_t addr = 0x08; addr <= 0x77 && *found_count < max_count; addr++) {
        // Use i2c_master_probe - sends address and checks for ACK
        // Match ESP-IDF i2c_tools example: timeout passed directly in milliseconds
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, scan_timeout_ms);
        
        if (ret == ESP_OK) {
            // Device found!
            addresses[*found_count] = addr;
            (*found_count)++;
        }
        // ESP_ERR_TIMEOUT and ESP_ERR_NOT_FOUND mean no device - these are expected
        
        // Feed watchdog periodically during long scan to prevent timeout
        // Scan can take several seconds, so we need to keep watchdog happy
        if ((addr % 16) == 0) {
            watchdog_task_heartbeat();
        }
    }

    // Restore original log level
    esp_log_level_set("i2c.master", old_level);

    return ESP_OK;
}

// ============================================================================
// I2C Error Statistics Functions
// ============================================================================

void i2c_manager_record_error(esp_err_t error_code) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    i2c_stats.total_errors++;
    i2c_stats.last_error_time_ms = current_time;
    
    // Categorize error type
    // Note: ESP-IDF I2C driver doesn't expose NACK as a separate error code.
    // NACKs are typically reported as ESP_ERR_TIMEOUT or ESP_FAIL.
    switch (error_code) {
        case ESP_ERR_TIMEOUT:
            // Timeout errors (may include NACKs)
            i2c_stats.timeout_errors++;
            break;
        case ESP_ERR_NOT_FOUND:
            // Device probe failures
            i2c_stats.device_not_found++;
            break;
        case ESP_FAIL:
            // General failures (may include NACKs and transaction failures)
            // Count as transaction failure since we can't distinguish NACK from other failures
            i2c_stats.transaction_failures++;
            break;
        case ESP_ERR_INVALID_RESPONSE:
        case ESP_ERR_INVALID_STATE:
            // Transaction-level errors
            i2c_stats.transaction_failures++;
            break;
        case ESP_ERR_INVALID_ARG:
        case ESP_ERR_INVALID_SIZE:
            // Bus configuration errors
            i2c_stats.bus_errors++;
            break;
        default:
            // Check if it's a bus-level error (negative error codes)
            if (error_code < 0) {
                i2c_stats.bus_errors++;
            } else {
                // Unknown error type, count as transaction failure
                i2c_stats.transaction_failures++;
            }
            break;
    }
}

void i2c_manager_get_stats(I2CStats_t* stats) {
    if (stats == NULL) {
        return;
    }
    
    // Copy statistics (thread-safe copy)
    *stats = i2c_stats;
}

void i2c_manager_reset_stats(void) {
    memset(&i2c_stats, 0, sizeof(I2CStats_t));
}

