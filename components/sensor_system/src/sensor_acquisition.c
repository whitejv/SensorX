/*
 * sensor_acquisition.c - Sensor Acquisition Task Implementation
 *
 * This file implements the sensor acquisition task that reads sensors
 * in 100ms frames to distribute I2C operations and prevent bus conflicts.
 *
 * Frame Schedule (10 frames per second, 100ms each):
 * - Frame 0 (0-100ms): Temperature sensors (DS18B20)
 * - Frame 1: Skip (spacing)
 * - Frame 2 (200-300ms): Basic I/O (analog/digital)
 * - Frame 3: Skip (spacing)
 * - Frame 4 (400-500ms): ADC sensors (ADS1015/ADS1115)
 * - Frame 5: Skip (spacing)
 * - Frame 6 (600-700ms): GPIO expander (MCP23X17)
 * - Frame 7: Skip (spacing)
 * - Frame 8 (800-900ms): BME280 environmental sensor
 * - Frame 9: Skip (spacing)
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sensor_acquisition.h"
#include "config.h"
#include "watchdog.h"
#include "error_recovery.h"
#include "i2c_manager.h"

static const char *TAG = "SENSOR_ACQ";

// Task handle is defined in system_init.c, declared extern in sensor_acquisition.h

// Sensor device presence flags (will be populated by sensor drivers)
// These are weak symbols - if not defined by sensor drivers, default to false
bool ads1015_present __attribute__((weak)) = false;
bool ads1115_present __attribute__((weak)) = false;
bool gpio_expander_present __attribute__((weak)) = false;
bool BME280_present __attribute__((weak)) = false;
int tempSensorcount __attribute__((weak)) = 0;

// Forward declarations for sensor read functions (to be implemented by sensor drivers)
// Weak implementations provided here - sensor drivers will override them
void updateTemperatureData(void) __attribute__((weak));
void readAnalogInput(void) __attribute__((weak));
void readDigitalInput(void) __attribute__((weak));
void readADS1015(void) __attribute__((weak));
void readADS1115(void) __attribute__((weak));
void readGPIOExpander(void) __attribute__((weak));
void readBME280(void) __attribute__((weak));

// Weak implementations (empty) - sensor drivers will provide actual implementations
void updateTemperatureData(void) {
    // Empty - sensor driver will implement
}

void readAnalogInput(void) {
    // Empty - sensor driver will implement
}

void readDigitalInput(void) {
    // Empty - sensor driver will implement
}

void readADS1015(void) {
    // Empty - sensor driver will implement
}

void readADS1115(void) {
    // Empty - sensor driver will implement
}

void readGPIOExpander(void) {
    // Empty - sensor driver will implement
}

void readBME280(void) {
    // Empty - sensor driver will implement
}

void vSensorAcquisitionTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Sensor acquisition task started");

    // Register with watchdog
    watchdog_register_current_task("SensorAcq", SENSOR_ACQUISITION_INTERVAL + 200);

    int lastFrameIndex = -1;

    for (;;) {
        // Calculate current frame index (0-9, cycles every 1000ms)
        // Frame index = (milliseconds / 100) % 10
        uint32_t currentTimeMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
        int frameIndex = (currentTimeMs / SENSOR_ACQUISITION_INTERVAL) % 10;

        // Only execute if we're in a new frame (prevents multiple calls per 100ms frame)
        if (frameIndex != lastFrameIndex) {
            switch (frameIndex) {
                case 0:  // 0-100ms: Temperature sensors
                    if (tempSensorcount > 0) {
                        updateTemperatureData();
                    }
                    break;

                case 1:  // 100-200ms: Skip (spacing)
                    break;

                case 2:  // 200-300ms: Basic I/O (analog/digital)
                    readAnalogInput();
                    readDigitalInput();
                    break;

                case 3:  // 300-400ms: Skip (spacing)
                    break;

                case 4:  // 400-500ms: ADC sensors
                    if (ads1015_present) {
                        readADS1015();
                    }
                    if (ads1115_present) {
                        readADS1115();
                    }
                    break;

                case 5:  // 500-600ms: Skip (spacing)
                    break;

                case 6:  // 600-700ms: GPIO expander
                    if (gpio_expander_present) {
                        readGPIOExpander();
                    }
                    break;

                case 7:  // 700-800ms: Skip (spacing)
                    break;

                case 8:  // 800-900ms: BME280 environmental sensor
                    if (BME280_present) {
                        readBME280();
                    }
                    break;

                case 9:  // 900-1000ms: Skip (spacing)
                    break;

                default:
                    break;
            }

            lastFrameIndex = frameIndex;
        }

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        // Wait for next frame check (check frequently to catch frame transitions)
        vTaskDelay(pdMS_TO_TICKS(10));  // Check every 10ms
    }
}

