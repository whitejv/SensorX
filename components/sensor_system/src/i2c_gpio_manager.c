/*
 * i2c_gpio_manager.c - I2C GPIO Manager Implementation
 *
 * This file implements the I2C GPIO Manager for MCP23017 GPIO expander.
 * Uses direct register access via ESP-IDF I2C master driver (no library dependency).
 *
 * Features:
 * - Supports MCP23017 I2C GPIO expander
 * - Reads Port A and Port B (16 GPIOs total) every 1000ms
 * - Updates genericSens_.GPIO_x1 (Port A) and GPIO_x2 (Port B)
 * - Thread-safe updates with mutex protection
 * - Watchdog protection
 * - Error handling
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include <driver/i2c_master.h>

#include "i2c_gpio_manager.h"
#include "i2c_manager.h"
#include "sensor_coordination.h"
#include "sensor.h"
#include "config.h"
#include "watchdog.h"
#include "pins.h"

static const char *TAG = "I2C_GPIO";

// ============================================================================
// MCP23017 Register Definitions
// ============================================================================

// MCP23017 Register Addresses
#define MCP23017_REG_IODIRA    0x00  // Port A direction (1=input, 0=output)
#define MCP23017_REG_IODIRB    0x01  // Port B direction
#define MCP23017_REG_GPPUA     0x0C  // Port A pull-up enable
#define MCP23017_REG_GPPUB     0x0D  // Port B pull-up enable
#define MCP23017_REG_GPIOA     0x12  // Port A input register (read)
#define MCP23017_REG_GPIOB     0x13  // Port B input register (read)

// ============================================================================
// Task Handle
// ============================================================================

TaskHandle_t xI2cGpioManagerTaskHandle = NULL;

// ============================================================================
// GPIO Expander Device State Structure
// ============================================================================

typedef struct {
    i2c_master_dev_handle_t i2c_device_handle; // I2C device handle
    uint8_t i2c_address;                       // I2C address (0x20)
    bool present;                               // Device detected
    bool enabled;                               // Device enabled
} GpioExpanderState;

// Manager State
static GpioExpanderState gpio_expander = {0};
static bool initialized = false;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Write a register to MCP23017
 */
static esp_err_t mcp23017_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, 2, I2C_TIMEOUT_MS);
}

/**
 * Read a register from MCP23017
 */
static esp_err_t mcp23017_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *value) {
    // Write register address
    esp_err_t ret = i2c_master_transmit(dev_handle, &reg, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    // Read register value
    return i2c_master_receive(dev_handle, value, 1, I2C_TIMEOUT_MS);
}

/**
 * Create and initialize MCP23017 device
 */
static esp_err_t create_gpio_expander(uint8_t address) {
    // Get I2C bus handle from existing manager
    i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus_handle();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create I2C device handle for MCP23017
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
    
    // Configure Port A as inputs with pull-ups (matching Arduino code)
    ret = mcp23017_write_register(i2c_dev_handle, MCP23017_REG_IODIRA, 0xFF);  // All inputs
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Port A direction: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ret;
    }
    
    ret = mcp23017_write_register(i2c_dev_handle, MCP23017_REG_GPPUA, 0xFF);  // All pull-ups
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Port A pull-ups: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ret;
    }
    
    // Configure Port B as inputs with pull-ups
    ret = mcp23017_write_register(i2c_dev_handle, MCP23017_REG_IODIRB, 0xFF);  // All inputs
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Port B direction: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ret;
    }
    
    ret = mcp23017_write_register(i2c_dev_handle, MCP23017_REG_GPPUB, 0xFF);  // All pull-ups
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Port B pull-ups: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ret;
    }
    
    // Test device communication by reading Port A
    uint8_t test_val = 0;
    ret = mcp23017_read_register(i2c_dev_handle, MCP23017_REG_GPIOA, &test_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with MCP23017 at 0x%02X: %s", 
                address, esp_err_to_name(ret));
        i2c_master_bus_rm_device(i2c_dev_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Store device state
    gpio_expander.i2c_device_handle = i2c_dev_handle;
    gpio_expander.i2c_address = address;
    gpio_expander.present = true;
    gpio_expander.enabled = true;
    
    ESP_LOGI(TAG, "MCP23017 initialized at address 0x%02X", address);
    
    return ESP_OK;
}

// ============================================================================
// Task Function
// ============================================================================

static void vI2cGpioManagerTask(void *pvParameters) {
    ESP_LOGI(TAG, "I2C GPIO Manager task started");
    
    // Register with watchdog
    watchdog_register_current_task("I2C_GPIO_Mgr", I2C_GPIO_TASK_INTERVAL_MS + 500);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_interval = pdMS_TO_TICKS(I2C_GPIO_TASK_INTERVAL_MS);
    
    while (1) {
        // Local variables to store GPIO readings (outside mutex)
        int32_t port_a_value = 0;
        int32_t port_b_value = 0;
        
        // PHASE 1: Read GPIO expander ports (OUTSIDE mutex to minimize mutex hold time)
        if (gpio_expander.enabled && gpio_expander.present) {
            uint8_t port_a = 0;
            uint8_t port_b = 0;
            
            // Read Port A
            esp_err_t ret_a = mcp23017_read_register(gpio_expander.i2c_device_handle, 
                                                    MCP23017_REG_GPIOA, &port_a);
            // Read Port B
            esp_err_t ret_b = mcp23017_read_register(gpio_expander.i2c_device_handle, 
                                                    MCP23017_REG_GPIOB, &port_b);
            
            if (ret_a == ESP_OK && ret_b == ESP_OK) {
                port_a_value = (int32_t)port_a;
                port_b_value = (int32_t)port_b;
            } else {
                if (ret_a != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read MCP23017 Port A: %s", esp_err_to_name(ret_a));
                    i2c_manager_record_error(ret_a);
                }
                if (ret_b != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read MCP23017 Port B: %s", esp_err_to_name(ret_b));
                    i2c_manager_record_error(ret_b);
                }
                // Use 0 on error
                port_a_value = 0;
                port_b_value = 0;
            }
        } else {
            // Device not present or disabled - set to 0
            port_a_value = 0;
            port_b_value = 0;
        }
        
        // PHASE 2: Update genericSens_ structure (INSIDE mutex - quick operation)
        // Take mutex on genericSens_ with retry logic
        // Multiple sensor tasks may compete for the mutex, so retry if needed
        bool mutex_acquired = false;
        for (int retry = 0; retry < 3; retry++) {
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                mutex_acquired = true;
                break;
            }
            // Small delay before retry to allow other tasks to release mutex
            if (retry < 2) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        
        if (mutex_acquired) {
            // Copy readings to genericSens_ structure
            genericSens_.generic.GPIO_x1 = port_a_value;
            genericSens_.generic.GPIO_x2 = port_b_value;
            
            // Release mutex immediately after update
            xSemaphoreGive(sensor_data_mutex);
        } else {
            // Failed to acquire mutex after retries - log warning but continue
            ESP_LOGW(TAG, "Failed to take sensor data mutex after retries");
            // Still signal completion to avoid blocking MQTT publisher
        }
        
        // Signal completion for MQTT publisher coordination
        sensor_coordination_signal_completion(SENSOR_EVENT_GPIO_I2C_COMPLETE);
        
        // Watchdog heartbeat
        watchdog_task_heartbeat();
        
        // Delay until next cycle
        vTaskDelayUntil(&last_wake_time, task_interval);
    }
}

// ============================================================================
// C Interface Functions
// ============================================================================

esp_err_t i2c_gpio_manager_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "I2C GPIO Manager already initialized");
        return ESP_OK;
    }
    
    // Verify I2C manager is initialized
    if (!i2c_manager_is_initialized()) {
        ESP_LOGE(TAG, "I2C manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize device state
    memset(&gpio_expander, 0, sizeof(gpio_expander));
    
    // Create task
    BaseType_t ret = xTaskCreate(
        vI2cGpioManagerTask,
        "I2C_GPIO_Mgr",
        TASK_STACK_SIZE_FIXED_FREQ,
        NULL,
        TASK_PRIORITY_FIXED_FREQ,
        &xI2cGpioManagerTaskHandle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I2C GPIO Manager task");
        return ESP_ERR_NO_MEM;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "I2C GPIO Manager initialized");
    
    return ESP_OK;
}

esp_err_t i2c_gpio_manager_register_expander(uint8_t address) {
    if (!initialized) {
        ESP_LOGE(TAG, "I2C GPIO Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (gpio_expander.present) {
        ESP_LOGW(TAG, "GPIO expander already registered");
        return ESP_OK;
    }
    
    // Validate address
    if (address != MCP23017_I2C_ADDRESS) {
        ESP_LOGW(TAG, "Unexpected GPIO expander address: 0x%02X (expected 0x%02X)", 
                address, MCP23017_I2C_ADDRESS);
    }
    
    return create_gpio_expander(address);
}

uint8_t i2c_gpio_manager_get_device_count(void) {
    return gpio_expander.present ? 1 : 0;
}

bool i2c_gpio_manager_is_device_present(uint8_t address) {
    return (gpio_expander.i2c_address == address && 
            gpio_expander.present && 
            gpio_expander.enabled);
}

