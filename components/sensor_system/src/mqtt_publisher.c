/*
 * mqtt_publisher.c - MQTT Publisher Task Implementation
 *
 * This file implements the MQTT publishing task that publishes sensor data
 * at fixed intervals (1000ms). Publishes both binary and JSON formats.
 * 
 * Architecture:
 * - Fixed Frequency rate group (Priority 3) to guarantee transmit
 * - Waits for sensor coordination before publishing
 * - Publishes binary first (QoS 2, critical)
 * - Publishes JSON second (QoS 0, viewing) with timeout/abort mechanism
 * - Uses same local_copy buffer for both binary and JSON (perfect synchronization)
 * - Increments cycle counter after successful binary publish
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <cJSON.h>
#include <string.h>

#include "mqtt_publisher.h"
#include "config.h"
#include "watchdog.h"
#include "error_recovery.h"
#include "mqtt_manager.h"
#include "wifi_manager.h"
#include "sensor.h"
#include "sensor_coordination.h"

static const char *TAG = "MQTT_PUBLISHER";

// Task handle is defined in system_init.c, declared extern in mqtt_publisher.h

void vMqttPublisherTask(void *pvParameters) {
    (void)pvParameters;

    // MQTT publisher task started - no log (only report errors)

    // Register with watchdog
    watchdog_register_current_task("MqttPub", MQTT_PUBLISH_INTERVAL + 1000);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL);
    const TickType_t json_timeout = pdMS_TO_TICKS(MQTT_JSON_TIMEOUT_MS);

    for (;;) {
        // Wait for WiFi and MQTT connection before publishing
        if (!wifi_manager_is_connected() || !mqtt_manager_is_connected()) {
            watchdog_task_heartbeat();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // Wait for sensor coordination (all 1000ms sensors complete)
        EventBits_t bits = xEventGroupWaitBits(
            sensor_event_group,
            SENSOR_EVENT_1000MS_MASK,
            pdTRUE,  // Clear bits on exit
            pdTRUE,  // Wait for ALL bits
            pdMS_TO_TICKS(MQTT_SENSOR_COORD_TIMEOUT_MS)  // 100ms timeout
        );

        if (bits & SENSOR_EVENT_1000MS_MASK) {
            // All sensors complete - proceed with publish
            
            // 1. Take mutex and copy data (minimize mutex hold time)
            union GENERICSENS_ local_copy;
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(MQTT_PUBLISH_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                memcpy(&local_copy, &genericSens_, sizeof(union GENERICSENS_));
                xSemaphoreGive(sensor_data_mutex);
                
                // 2. Publish binary FIRST (critical, outside mutex)
                esp_err_t binary_ret = mqtt_manager_publish_binary(
                    MQTT_TOPIC_BINARY_GENERICSENS,
                    (uint8_t*)local_copy.data_payload,
                    GENERICSENS_SIZE_BYTES,
                    MQTT_BINARY_QOS,  // QoS 2 - exactly-once delivery
                    false  // No retain
                );
                
                if (binary_ret == ESP_OK) {
                    // Binary message published successfully - no log (only report errors)
                    
                    // 3. Increment cycle counter (after successful binary publish)
                    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(MQTT_PUBLISH_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        genericSens_.generic.cycle_count++;
                        if (genericSens_.generic.cycle_count > 28800) {
                            genericSens_.generic.cycle_count = 0;  // Reset at max
                        }
                        xSemaphoreGive(sensor_data_mutex);
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to publish binary message: %s", esp_err_to_name(binary_ret));
                }
                
                // 4. Publish JSON (viewing, same data snapshot, with timeout protection)
                TickType_t json_start_time = xTaskGetTickCount();
                
                // Check if we have time remaining in cycle for JSON operations
                // Use more conservative check: need at least 2x timeout budget to attempt JSON
                TickType_t elapsed_since_wake = xTaskGetTickCount() - xLastWakeTime;
                TickType_t min_time_required = json_timeout * 2;  // Need 2x timeout budget
                if (elapsed_since_wake < (xFrequency - min_time_required)) {
                    // We have time for JSON operations
                    
                    cJSON *json = cJSON_CreateObject();
                    if (json == NULL) {
                        ESP_LOGE(TAG, "Failed to create JSON object");
                        // Continue - JSON failure doesn't affect binary
                    } else {
                        // Extract FlowData structures from local_copy (same as binary)
                        FlowData_t flow1, flow2, flow3;
                        GENERICSENS_EXTRACT_FLOWDATA(local_copy.generic.flowData1, flow1);
                        GENERICSENS_EXTRACT_FLOWDATA(local_copy.generic.flowData2, flow2);
                        GENERICSENS_EXTRACT_FLOWDATA(local_copy.generic.flowData3, flow3);
                        
                        // Flow sensors (bit-packed, extract individual fields)
                        cJSON_AddNumberToObject(json, "S005D:flow1_pulses", flow1.pulses);
                        cJSON_AddNumberToObject(json, "S005D:flow1_ms", flow1.milliseconds);
                        cJSON_AddNumberToObject(json, "S005D:flow1_newData", flow1.newData);
                        cJSON_AddNumberToObject(json, "S005D:flow2_pulses", flow2.pulses);
                        cJSON_AddNumberToObject(json, "S005D:flow2_ms", flow2.milliseconds);
                        cJSON_AddNumberToObject(json, "S005D:flow2_newData", flow2.newData);
                        cJSON_AddNumberToObject(json, "S005D:flow3_pulses", flow3.pulses);
                        cJSON_AddNumberToObject(json, "S005D:flow3_ms", flow3.milliseconds);
                        cJSON_AddNumberToObject(json, "S005D:flow3_newData", flow3.newData);
                        
                        // Integer fields (words 3-11) - use variable names array
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[3], local_copy.generic.adc_sensor);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[4], local_copy.generic.gpio_sensor);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[5], local_copy.generic.temp1);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[6], local_copy.generic.temp1_f);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[7], local_copy.generic.tempSensorcount);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[8], local_copy.generic.cycle_count);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[9], local_copy.generic.fw_version);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[10], local_copy.generic.GPIO_x1);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[11], local_copy.generic.GPIO_x2);
                        
                        // Float fields (words 12-25) - use variable names array
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[12], local_copy.generic.adc_x1);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[13], local_copy.generic.adc_x2);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[14], local_copy.generic.adc_x3);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[15], local_copy.generic.adc_x4);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[16], local_copy.generic.adc_x5);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[17], local_copy.generic.adc_x6);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[18], local_copy.generic.adc_x7);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[19], local_copy.generic.adc_x8);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[20], local_copy.generic.tempx);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[21], local_copy.generic.pressurex);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[22], local_copy.generic.humidity);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[23], local_copy.generic.temp2);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[24], local_copy.generic.temp3);
                        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[25], local_copy.generic.temp4);
                        
                        // Add timestamp
                        cJSON_AddNumberToObject(json, "timestamp", xTaskGetTickCount() * portTICK_PERIOD_MS);
                        
                        // Check timeout before serialization (most expensive operation)
                        TickType_t json_elapsed = xTaskGetTickCount() - json_start_time;
                        if (json_elapsed < json_timeout) {
                            // Send watchdog heartbeat before potentially long operation
                            watchdog_task_heartbeat();
                            
                            // Serialize and publish (within time budget)
                            // Note: cJSON_Print() is blocking and can't be interrupted
                            char *json_string = cJSON_Print(json);
                            
                            // Check timeout immediately after serialization
                            json_elapsed = xTaskGetTickCount() - json_start_time;
                            if (json_string != NULL) {
                                // Send heartbeat before publish
                                watchdog_task_heartbeat();
                                
                                // Check timeout again before publish
                                if (json_elapsed < json_timeout) {
                                    esp_err_t json_ret = mqtt_manager_publish_json(
                                        MQTT_TOPIC_JSON_GENERICSENS,
                                        json_string,
                                        MQTT_JSON_QOS,  // QoS 0 for viewing
                                        false
                                    );
                                    
                                    if (json_ret == ESP_OK) {
                                        // JSON message published successfully - no log (only report errors)
                                    } else {
                                        ESP_LOGW(TAG, "Failed to publish JSON message: %s", esp_err_to_name(json_ret));
                                    }
                                } else {
                                    ESP_LOGW(TAG, "JSON timeout before publish (took %lu ms) - aborting", 
                                             json_elapsed * portTICK_PERIOD_MS);
                                }
                                
                                free(json_string);
                            } else {
                                ESP_LOGE(TAG, "Failed to serialize JSON");
                            }
                        } else {
                            ESP_LOGW(TAG, "JSON timeout before serialization - aborting");
                        }
                        
                        cJSON_Delete(json);
                        
                        // Log timing for monitoring
                        json_elapsed = xTaskGetTickCount() - json_start_time;
                        if (json_elapsed > json_timeout) {
                            ESP_LOGW(TAG, "JSON operations took %lu ms (exceeded %lu ms timeout) - consider skipping JSON", 
                                     json_elapsed * portTICK_PERIOD_MS, 
                                     MQTT_JSON_TIMEOUT_MS);
                        } else {
                            ESP_LOGD(TAG, "JSON operations completed in %lu ms", json_elapsed * portTICK_PERIOD_MS);
                        }
                    }
                } else {
                    ESP_LOGI(TAG, "Skipping JSON - insufficient time remaining in cycle (elapsed: %lu ms, need: %lu ms)", 
                             elapsed_since_wake * portTICK_PERIOD_MS, min_time_required * portTICK_PERIOD_MS);
                }
            } else {
                ESP_LOGW(TAG, "Failed to acquire mutex for data copy");
            }
        } else {
            ESP_LOGW(TAG, "Sensor coordination timeout - sensors may not have completed");
            // Proceed anyway - better than missing publish cycle
        }
        
        watchdog_task_heartbeat();
        
        // Wait for next 1000ms interval (precise timing)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
