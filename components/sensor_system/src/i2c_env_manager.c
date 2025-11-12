/*
 * i2c_env_manager.c - I2C Environmental Sensor Manager Implementation
 *
 * This file implements the I2C Environmental Sensor Manager for BME280 devices.
 * Uses direct register access via ESP-IDF I2C master driver (no library dependency).
 *
 * Features:
 * - Supports BME280 I2C environmental sensor
 * - Reads temperature, humidity, and pressure every 5000ms
 * - Updates genericSens_.tempx, humidity, pressurex
 * - Thread-safe updates with mutex protection
 * - Watchdog protection
 * - Error handling and I2C error tracking
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

#include <driver/i2c_master.h>

#include "i2c_env_manager.h"
#include "i2c_manager.h"
#include "sensor_coordination.h"
#include "sensor.h"
#include "config.h"
#include "watchdog.h"
#include "pins.h"

static const char *TAG = "I2C_ENV";

// ============================================================================
// Task Handle
// ============================================================================

TaskHandle_t xI2cEnvManagerTaskHandle = NULL;

// ============================================================================
// BME280 Register Definitions
// ============================================================================

// BME280 Register Addresses
#define BME280_REG_DIG_T1        0x88  // Temperature calibration (2 bytes)
#define BME280_REG_DIG_T2        0x8A  // Temperature calibration (2 bytes)
#define BME280_REG_DIG_T3        0x8C  // Temperature calibration (2 bytes)
#define BME280_REG_DIG_P1        0x8E  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P2        0x90  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P3        0x92  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P4        0x94  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P5        0x96  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P6        0x98  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P7        0x9A  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P8        0x9C  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_P9        0x9E  // Pressure calibration (2 bytes)
#define BME280_REG_DIG_H1        0xA1  // Humidity calibration (1 byte)
#define BME280_REG_DIG_H2        0xE1  // Humidity calibration (2 bytes)
#define BME280_REG_DIG_H3        0xE3  // Humidity calibration (1 byte)
#define BME280_REG_DIG_H4        0xE4  // Humidity calibration (2 bytes, special)
#define BME280_REG_DIG_H5        0xE5  // Humidity calibration (2 bytes, special)
#define BME280_REG_DIG_H6        0xE7  // Humidity calibration (1 byte)

#define BME280_REG_CHIPID        0xD0  // Chip ID (should be 0x60)
#define BME280_REG_VERSION       0xD1  // Version
#define BME280_REG_SOFTRESET     0xE0  // Soft reset
#define BME280_REG_CAL26         0xE1  // Calibration data continues

#define BME280_REG_CTRL_HUM      0xF2  // Humidity control
#define BME280_REG_STATUS         0xF3  // Status register
#define BME280_REG_CTRL_MEAS     0xF4  // Control measurement
#define BME280_REG_CONFIG        0xF5  // Configuration
#define BME280_REG_PRESS_MSB     0xF7  // Pressure data (MSB)
#define BME280_REG_PRESS_LSB     0xF8  // Pressure data (LSB)
#define BME280_REG_PRESS_XLSB    0xF9  // Pressure data (XLSB)
#define BME280_REG_TEMP_MSB      0xFA  // Temperature data (MSB)
#define BME280_REG_TEMP_LSB      0xFB  // Temperature data (LSB)
#define BME280_REG_TEMP_XLSB     0xFC  // Temperature data (XLSB)
#define BME280_REG_HUM_MSB       0xFD  // Humidity data (MSB)
#define BME280_REG_HUM_LSB       0xFE  // Humidity data (LSB)

// BME280 Control Register Values
#define BME280_MODE_SLEEP        0x00
#define BME280_MODE_FORCED       0x01
#define BME280_MODE_NORMAL       0x03

#define BME280_OSRS_T_SKIP       0x00
#define BME280_OSRS_T_1X         0x20
#define BME280_OSRS_P_SKIP       0x00
#define BME280_OSRS_P_1X         0x04
#define BME280_OSRS_H_SKIP       0x00
#define BME280_OSRS_H_1X         0x01

// ============================================================================
// BME280 Calibration Data Structure
// ============================================================================

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    bool calibration_loaded;
} BME280Calibration_t;

// ============================================================================
// BME280 Device State Structure
// ============================================================================

typedef struct {
    i2c_master_dev_handle_t i2c_device_handle; // I2C device handle
    uint8_t i2c_address;                      // I2C address (0x76 or 0x77)
    BME280Calibration_t calibration;          // Calibration data
    bool present;                              // Device detected
    bool enabled;                              // Device enabled
} BME280State;

// Manager State
static BME280State bme280_sensor = {0};
static bool initialized = false;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Read a register from BME280
 */
static esp_err_t bme280_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *value) {
    if (dev_handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // Write register address
    esp_err_t ret = i2c_master_transmit(dev_handle, &reg, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    // Read register value
    return i2c_master_receive(dev_handle, value, 1, I2C_TIMEOUT_MS);
}

/**
 * Read multiple registers from BME280
 */
static esp_err_t bme280_read_registers(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *data, size_t len) {
    if (dev_handle == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // Write register address
    esp_err_t ret = i2c_master_transmit(dev_handle, &reg, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    // Read register values
    return i2c_master_receive(dev_handle, data, len, I2C_TIMEOUT_MS);
}

/**
 * Write a register to BME280
 */
static esp_err_t bme280_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t value) {
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, 2, I2C_TIMEOUT_MS);
}

/**
 * Read calibration data from BME280
 */
static esp_err_t bme280_read_calibration(i2c_master_dev_handle_t dev_handle, BME280Calibration_t *cal) {
    if (dev_handle == NULL || cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t cal_data[26];
    esp_err_t ret;
    
    // Read temperature and pressure calibration (0x88-0x9F, 24 bytes)
    ret = bme280_read_registers(dev_handle, BME280_REG_DIG_T1, cal_data, 24);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse temperature calibration
    cal->dig_T1 = (uint16_t)(cal_data[0] | (cal_data[1] << 8));
    cal->dig_T2 = (int16_t)(cal_data[2] | (cal_data[3] << 8));
    cal->dig_T3 = (int16_t)(cal_data[4] | (cal_data[5] << 8));
    
    // Parse pressure calibration
    cal->dig_P1 = (uint16_t)(cal_data[6] | (cal_data[7] << 8));
    cal->dig_P2 = (int16_t)(cal_data[8] | (cal_data[9] << 8));
    cal->dig_P3 = (int16_t)(cal_data[10] | (cal_data[11] << 8));
    cal->dig_P4 = (int16_t)(cal_data[12] | (cal_data[13] << 8));
    cal->dig_P5 = (int16_t)(cal_data[14] | (cal_data[15] << 8));
    cal->dig_P6 = (int16_t)(cal_data[16] | (cal_data[17] << 8));
    cal->dig_P7 = (int16_t)(cal_data[18] | (cal_data[19] << 8));
    cal->dig_P8 = (int16_t)(cal_data[20] | (cal_data[21] << 8));
    cal->dig_P9 = (int16_t)(cal_data[22] | (cal_data[23] << 8));
    
    // Read humidity calibration H1 (0xA1)
    ret = bme280_read_register(dev_handle, BME280_REG_DIG_H1, &cal->dig_H1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read humidity calibration H2-H6 (0xE1-0xE7, 7 bytes)
    uint8_t hum_cal[7];
    ret = bme280_read_registers(dev_handle, BME280_REG_CAL26, hum_cal, 7);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse humidity calibration
    cal->dig_H2 = (int16_t)(hum_cal[0] | (hum_cal[1] << 8));
    cal->dig_H3 = hum_cal[2];
    cal->dig_H4 = (int16_t)(hum_cal[3] << 4) | (hum_cal[4] & 0x0F);
    cal->dig_H5 = (int16_t)((hum_cal[4] >> 4) | (hum_cal[5] << 4));
    cal->dig_H6 = (int8_t)hum_cal[6];
    
    cal->calibration_loaded = true;
    return ESP_OK;
}

/**
 * Compensate temperature (BME280 algorithm)
 * Returns temperature in Celsius
 */
static float bme280_compensate_temperature(int32_t adc_T, BME280Calibration_t *cal, int32_t *t_fine) {
    int32_t var1, var2;
    
    var1 = ((((adc_T >> 3) - ((int32_t)cal->dig_T1 << 1))) * ((int32_t)cal->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)cal->dig_T1)) * ((adc_T >> 4) - ((int32_t)cal->dig_T1))) >> 12) * ((int32_t)cal->dig_T3)) >> 14;
    
    *t_fine = var1 + var2;
    
    float T = (*t_fine * 5 + 128) >> 8;
    return T / 100.0f;  // Return in Celsius
}

/**
 * Compensate pressure (BME280 algorithm)
 * Returns pressure in Pascal
 */
static float bme280_compensate_pressure(int32_t adc_P, int32_t t_fine, BME280Calibration_t *cal) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal->dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal->dig_P5) << 17);
    var2 = var2 + (((int64_t)cal->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal->dig_P3) >> 8) + ((var1 * (int64_t)cal->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal->dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal->dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal->dig_P7) << 4);
    
    return (float)(p / 256.0);  // Return in Pascal
}

/**
 * Compensate humidity (BME280 algorithm)
 * Returns humidity in %
 */
static float bme280_compensate_humidity(int32_t adc_H, int32_t t_fine, BME280Calibration_t *cal) {
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal->dig_H4) << 20) - (((int32_t)cal->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)cal->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)cal->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)cal->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)cal->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    
    float h = (v_x1_u32r >> 12);
    return h / 1024.0f;  // Return in %
}

/**
 * Create and initialize BME280 device
 */
static esp_err_t create_bme280_device(uint8_t address) {
    // Clean up any existing device handle first
    if (bme280_sensor.i2c_device_handle != NULL) {
        i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus_handle();
        if (bus_handle != NULL) {
            i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        }
        bme280_sensor.i2c_device_handle = NULL;
    }
    
    // Reset calibration structure
    memset(&bme280_sensor.calibration, 0, sizeof(BME280Calibration_t));
    bme280_sensor.present = false;
    bme280_sensor.enabled = false;
    
    // Get I2C bus handle from existing manager
    i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus_handle();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create I2C device configuration
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_CLOCK_SPEED,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    // Add device to I2C bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &bme280_sensor.i2c_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s", address, esp_err_to_name(ret));
        bme280_sensor.i2c_device_handle = NULL;
        return ret;
    }

    // Probe device
    ret = i2c_master_probe(bus_handle, address, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Device at 0x%02X not responding to probe: %s", address, esp_err_to_name(ret));
        i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        bme280_sensor.i2c_device_handle = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // Small delay after probe
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify chip ID (should be 0x60 for BME280)
    uint8_t chip_id = 0;
    ret = bme280_read_register(bme280_sensor.i2c_device_handle, BME280_REG_CHIPID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        i2c_manager_record_error(ret);
        i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        bme280_sensor.i2c_device_handle = NULL;
        return ret;
    }
    
    if (chip_id != 0x60) {
        ESP_LOGW(TAG, "Unexpected chip ID: 0x%02X (expected 0x60 for BME280)", chip_id);
        i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        bme280_sensor.i2c_device_handle = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // Read calibration data
    ret = bme280_read_calibration(bme280_sensor.i2c_device_handle, &bme280_sensor.calibration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data: %s", esp_err_to_name(ret));
        i2c_manager_record_error(ret);
        i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        bme280_sensor.i2c_device_handle = NULL;
        memset(&bme280_sensor.calibration, 0, sizeof(BME280Calibration_t));
        return ret;
    }

    // Configure BME280 for normal mode
    // Set humidity oversampling (CTRL_HUM register)
    ret = bme280_write_register(bme280_sensor.i2c_device_handle, BME280_REG_CTRL_HUM, BME280_OSRS_H_1X);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set humidity control: %s", esp_err_to_name(ret));
        i2c_manager_record_error(ret);
        i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        bme280_sensor.i2c_device_handle = NULL;
        memset(&bme280_sensor.calibration, 0, sizeof(BME280Calibration_t));
        return ret;
    }

    // Set temperature and pressure oversampling, and mode (CTRL_MEAS register)
    uint8_t ctrl_meas = BME280_MODE_NORMAL | BME280_OSRS_T_1X | BME280_OSRS_P_1X;
    ret = bme280_write_register(bme280_sensor.i2c_device_handle, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set measurement control: %s", esp_err_to_name(ret));
        i2c_manager_record_error(ret);
        i2c_master_bus_rm_device(bme280_sensor.i2c_device_handle);
        bme280_sensor.i2c_device_handle = NULL;
        memset(&bme280_sensor.calibration, 0, sizeof(BME280Calibration_t));
        return ret;
    }

    // Small delay to allow sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));

    bme280_sensor.i2c_address = address;
    bme280_sensor.present = true;
    bme280_sensor.enabled = true;

    ESP_LOGI(TAG, "BME280 initialized at address 0x%02X", address);
    return ESP_OK;
}

// ============================================================================
// I2C Environmental Manager Task
// ============================================================================

/**
 * I2C Environmental Manager Task
 * 
 * Reads BME280 sensor every 5000ms.
 * Updates genericSens_.tempx, humidity, pressurex.
 */
void vI2cEnvManagerTask(void *pvParameters) {
    (void)pvParameters;
    
    ESP_LOGI(TAG, "I2C Environmental Manager Task started");
    
    // Register with watchdog
    watchdog_register_current_task("I2C_ENV_Mgr", I2C_ENV_TASK_INTERVAL_MS + 1000);
    
    const TickType_t xTaskInterval = pdMS_TO_TICKS(I2C_ENV_TASK_INTERVAL_MS);
    
    for (;;) {
        // PHASE 1: Read sensor data (OUTSIDE mutex to minimize mutex hold time)
        float temperature_c = 0.0f;
        float humidity_pct = 0.0f;
        float pressure_pa = 0.0f;
        bool read_success = false;
        
        if (bme280_sensor.enabled && bme280_sensor.present && 
            bme280_sensor.i2c_device_handle != NULL &&
            bme280_sensor.calibration.calibration_loaded) {
            
            // Read pressure, temperature, and humidity data (8 bytes starting at 0xF7)
            uint8_t sensor_data[8];
            esp_err_t ret = bme280_read_registers(bme280_sensor.i2c_device_handle, 
                                                   BME280_REG_PRESS_MSB, 
                                                   sensor_data, 8);
            
            if (ret == ESP_OK) {
                // Parse raw sensor data
                int32_t adc_P = (int32_t)((sensor_data[0] << 12) | (sensor_data[1] << 4) | (sensor_data[2] >> 4));
                int32_t adc_T = (int32_t)((sensor_data[3] << 12) | (sensor_data[4] << 4) | (sensor_data[5] >> 4));
                int32_t adc_H = (int32_t)((sensor_data[6] << 8) | sensor_data[7]);
                
                // Compensate temperature (also calculates t_fine needed for pressure/humidity)
                int32_t t_fine = 0;
                temperature_c = bme280_compensate_temperature(adc_T, &bme280_sensor.calibration, &t_fine);
                
                // Compensate pressure
                pressure_pa = bme280_compensate_pressure(adc_P, t_fine, &bme280_sensor.calibration);
                
                // Compensate humidity
                humidity_pct = bme280_compensate_humidity(adc_H, t_fine, &bme280_sensor.calibration);
                
                read_success = true;
            } else {
                ESP_LOGW(TAG, "Failed to read BME280 sensor data: %s", esp_err_to_name(ret));
                i2c_manager_record_error(ret);
            }
        }
        
        // PHASE 2: Update genericSens_ structure (INSIDE mutex - quick operation)
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            if (read_success) {
                // Convert units
                // Temperature: Celsius to Fahrenheit
                genericSens_.generic.tempx = (temperature_c * 9.0f / 5.0f) + 32.0f;
                
                // Humidity: Already in percentage
                genericSens_.generic.humidity = humidity_pct;
                
                // Pressure: Pascal to PSI (1 Pa = 0.000145038 PSI)
                genericSens_.generic.pressurex = pressure_pa * 0.000145038f;
            } else {
                // On error, set to 0 (allows fan control to detect invalid readings)
                genericSens_.generic.tempx = 0.0f;
                genericSens_.generic.humidity = 0.0f;
                genericSens_.generic.pressurex = 0.0f;
            }
            
            xSemaphoreGive(sensor_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take sensor data mutex");
        }
        
        // Send watchdog heartbeat
        watchdog_task_heartbeat();
        
        // Delay until next check interval
        vTaskDelay(xTaskInterval);
    }
}

// ============================================================================
// Initialization Functions
// ============================================================================

/**
 * Initialize I2C Environmental Manager
 * 
 * Initializes the environmental sensor manager and creates the task.
 * Must be called before registering sensors.
 */
esp_err_t i2c_env_manager_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "I2C Environmental Manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing I2C Environmental Manager...");
    
    // Initialize sensor state
    memset(&bme280_sensor, 0, sizeof(BME280State));
    
    // Create environmental manager task
    BaseType_t xResult = xTaskCreate(
        vI2cEnvManagerTask,
        "I2C_ENV_Mgr",
        TASK_STACK_SIZE_BACKGROUND,
        NULL,
        TASK_PRIORITY_BACKGROUND,  // Background priority (1)
        &xI2cEnvManagerTaskHandle
    );
    
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I2C Environmental Manager task");
        return ESP_FAIL;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "I2C Environmental Manager initialized successfully");
    
    return ESP_OK;
}

/**
 * Register BME280 Environmental Sensor
 * 
 * Registers a BME280 sensor with the I2C environmental manager.
 * Configures I2C device and initializes sensor.
 */
esp_err_t i2c_env_manager_register_bme280(uint8_t address) {
    if (!initialized) {
        ESP_LOGE(TAG, "I2C Environmental Manager not initialized - call i2c_env_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (bme280_sensor.present) {
        ESP_LOGW(TAG, "BME280 already registered at address 0x%02X", bme280_sensor.i2c_address);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Registering BME280 sensor at address 0x%02X", address);
    
    esp_err_t ret = create_bme280_device(address);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register BME280: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Registered BME280 sensor at address 0x%02X", address);
    
    return ESP_OK;
}

/**
 * Get number of registered environmental sensors
 */
uint8_t i2c_env_manager_get_sensor_count(void) {
    return bme280_sensor.present ? 1 : 0;
}

/**
 * Check if BME280 sensor is present and functioning
 */
bool i2c_env_manager_is_bme280_present(void) {
    return bme280_sensor.present && bme280_sensor.enabled;
}
