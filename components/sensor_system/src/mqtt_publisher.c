/*
 * mqtt_publisher.c - MQTT Publisher Task Implementation
 *
 * This file implements the MQTT publishing task that publishes sensor data
 * at fixed intervals (1000ms). Publishes both binary and JSON formats.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>

#include "mqtt_publisher.h"
#include "config.h"
#include "watchdog.h"
#include "error_recovery.h"
#include "mqtt_manager.h"
#include "wifi_manager.h"

static const char *TAG = "MQTT_PUBLISHER";

// Task handle is defined in system_init.c, declared extern in mqtt_publisher.h

// TODO: Sensor data will be populated by sensor drivers when implemented
// Using SensorData_t from types.h when sensor data structure is ready

void vMqttPublisherTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "MQTT publisher task started");

    // Register with watchdog
    watchdog_register_current_task("MqttPub", MQTT_PUBLISH_INTERVAL + 1000);

    const TickType_t xPublishInterval = pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL);

    for (;;) {
        // Wait for WiFi and MQTT connection before publishing
        if (!wifi_manager_is_connected()) {
            ESP_LOGD(TAG, "WiFi not connected, skipping publish");
            watchdog_task_heartbeat();
            vTaskDelay(xPublishInterval);
            continue;
        }

        if (!mqtt_manager_is_connected()) {
            ESP_LOGD(TAG, "MQTT not connected, skipping publish");
            watchdog_task_heartbeat();
            vTaskDelay(xPublishInterval);
            continue;
        }

        // TODO: Publish binary payload
        // For now, placeholder - actual binary payload will be added when sensor data structure is defined
        // Example:
        // uint8_t binary_data[256];
        // size_t binary_len = pack_sensor_data_to_binary(&sensor_data, binary_data, sizeof(binary_data));
        // mqtt_manager_publish_binary(NULL, binary_data, binary_len, 0, false);

        // Publish JSON payload
        cJSON *json = cJSON_CreateObject();
        if (json != NULL) {
            // Add sensor data to JSON (placeholder - actual data will be added later)
            cJSON_AddNumberToObject(json, "cycle_count", 0);  // Placeholder
            cJSON_AddNumberToObject(json, "timestamp", xTaskGetTickCount() * portTICK_PERIOD_MS);
            
            // TODO: Add actual sensor data fields when sensor data structure is implemented
            // Example:
            // cJSON_AddNumberToObject(json, "temp1", sensor_data.temp1);
            // cJSON_AddNumberToObject(json, "flow_pulses", sensor_data.flow_pulses);
            // etc.

            // Serialize JSON
            char *json_string = cJSON_Print(json);
            if (json_string != NULL) {
                esp_err_t ret = mqtt_manager_publish_json(NULL, json_string, 0, false);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to publish JSON message");
                    error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "vMqttPublisherTask", NULL);
                } else {
                    ESP_LOGD(TAG, "JSON message published successfully");
                }
                free(json_string);
            } else {
                ESP_LOGE(TAG, "Failed to serialize JSON");
                error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "vMqttPublisherTask", NULL);
            }

            cJSON_Delete(json);
        } else {
            ESP_LOGE(TAG, "Failed to create JSON object");
            error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "vMqttPublisherTask", NULL);
        }

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        // Wait for next publishing interval
        vTaskDelay(xPublishInterval);
    }
}

