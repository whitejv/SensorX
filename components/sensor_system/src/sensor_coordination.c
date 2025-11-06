/*
 * sensor_coordination.c - Sensor Coordination System Implementation
 * 
 * This file implements the sensor coordination system using FreeRTOS Event Groups
 * to synchronize sensor manager completion with MQTT publishing.
 */

#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#include "sensor_coordination.h"
#include "sensor.h"
#include "config.h"

static const char *TAG = "SENSOR_COORD";

// ============================================================================
// Global Variables
// ============================================================================

// Event group for sensor completion coordination
EventGroupHandle_t sensor_event_group = NULL;

// Mutex for thread-safe access to genericSens_
SemaphoreHandle_t sensor_data_mutex = NULL;

// Sensor data wrapper structure
SensorDataWrapper_t sensor_data = {0};

// Global genericSens_ instance (defined here, declared in sensor.h)
union GENERICSENS_ genericSens_;

// ============================================================================
// Initialization Functions
// ============================================================================

esp_err_t sensor_coordination_init(void) {
    // Create event group for sensor coordination
    sensor_event_group = xEventGroupCreate();
    if (sensor_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor event group");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Sensor event group created successfully");

    // Create mutex for sensor data access
    sensor_data_mutex = xSemaphoreCreateMutex();
    if (sensor_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data mutex");
        // Clean up event group if mutex creation fails
        vEventGroupDelete(sensor_event_group);
        sensor_event_group = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Sensor data mutex created successfully");

    // Initialize sensor data wrapper
    sensor_data.mutex = sensor_data_mutex;

    // Initialize genericSens_ structure to zero
    memset(&genericSens_, 0, sizeof(union GENERICSENS_));

    // Initialize default values
    genericSens_.generic.tempSensorcount = 0;
    genericSens_.generic.cycle_count = 0;
    genericSens_.generic.fw_version = (FIRMWARE_VERSION_MAJOR << 16) | FIRMWARE_VERSION_MINOR;

    ESP_LOGI(TAG, "Sensor coordination system initialized");
    ESP_LOGI(TAG, "GenericSens structure size: %zu bytes (expected: %d bytes)",
             sizeof(union GENERICSENS_), GENERICSENS_SIZE_BYTES);

    // Verify structure size matches expected
    if (sizeof(union GENERICSENS_) != GENERICSENS_SIZE_BYTES) {
        ESP_LOGW(TAG, "WARNING: Structure size mismatch! Expected %d bytes, got %zu bytes",
                 GENERICSENS_SIZE_BYTES, sizeof(union GENERICSENS_));
    }

    return ESP_OK;
}

EventGroupHandle_t sensor_coordination_get_event_group(void) {
    return sensor_event_group;
}

SemaphoreHandle_t sensor_coordination_get_mutex(void) {
    return sensor_data_mutex;
}

// ============================================================================
// Event Group Operations
// ============================================================================

void sensor_coordination_signal_completion(EventBits_t event_bits) {
    if (sensor_event_group != NULL) {
        xEventGroupSetBits(sensor_event_group, event_bits);
    }
}

EventBits_t sensor_coordination_wait_for_completion(
    EventBits_t event_mask,
    bool clear_on_exit,
    bool wait_for_all,
    TickType_t timeout_ticks
) {
    if (sensor_event_group == NULL) {
        ESP_LOGW(TAG, "Event group not initialized");
        return 0;
    }

    return xEventGroupWaitBits(
        sensor_event_group,
        event_mask,
        clear_on_exit ? pdTRUE : pdFALSE,
        wait_for_all ? pdTRUE : pdFALSE,
        timeout_ticks
    );
}

void sensor_coordination_clear_bits(EventBits_t event_bits) {
    if (sensor_event_group != NULL) {
        xEventGroupClearBits(sensor_event_group, event_bits);
    }
}

EventBits_t sensor_coordination_get_bits(void) {
    if (sensor_event_group == NULL) {
        return 0;
    }
    return xEventGroupGetBits(sensor_event_group);
}

