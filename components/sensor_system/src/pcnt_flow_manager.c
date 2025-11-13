/*
 * pcnt_flow_manager.c - PCNT Flow Sensor Manager Implementation
 * 
 * This file implements the PCNT Flow Sensor Manager for ESP32-C6.
 * Uses hardware PCNT (Pulse Counter) peripherals to count pulses from
 * Hall effect flow sensors with zero CPU overhead.
 * 
 * Features:
 * - Supports 3 flow sensors (PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2)
 * - Hardware-based pulse counting (no CPU overhead)
 * - 2000ms accumulation window for optimal accuracy
 * - 1000ms check interval
 * - Updates genericSens_.flowData1, flowData2, flowData3
 * - Watchdog protection
 * - Error handling and retry logic
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include <driver/pulse_cnt.h>
#include <driver/gpio.h>

#include "pcnt_flow_manager.h"
#include "config.h"
#include "pins.h"
#include "sensor_coordination.h"
#include "sensor.h"
#include "watchdog.h"

static const char *TAG = "PCNT_FLOW";

// ============================================================================
// Task Handle
// ============================================================================

TaskHandle_t xPcntFlowManagerTaskHandle = NULL;

// ============================================================================
// PCNT Flow Manager State
// ============================================================================

#define MAX_FLOW_SENSORS 3

typedef struct {
    pcnt_unit_handle_t pcnt_unit;  // PCNT unit handle (new API)
    gpio_num_t gpio_pin;
    const char* name;
    uint32_t last_reset_time;  // Timestamp of last reset (ms)
    bool enabled;
} FlowSensorConfig_t;

static FlowSensorConfig_t flow_sensors[MAX_FLOW_SENSORS];
static uint8_t num_sensors = 0;
static bool initialized = false;

// ============================================================================
// PCNT Flow Manager Task
// ============================================================================

/**
 * PCNT Flow Manager Task
 * 
 * Reads all registered flow sensors every 1000ms.
 * Pulse counts accumulate over 2000ms window for optimal accuracy.
 */
void vPcntFlowManagerTask(void *pvParameters) {
    (void)pvParameters;
    
    ESP_LOGI(TAG, "PCNT Flow Manager Task started");
    
    // Register with watchdog
    watchdog_register_current_task("PcntFlowMgr", PCNT_FLOW_TASK_INTERVAL_MS + 500);
    
    const TickType_t xTaskInterval = pdMS_TO_TICKS(PCNT_FLOW_TASK_INTERVAL_MS);
    
    for (;;) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Take mutex to update genericSens_ structure
        if (sensor_data.mutex != NULL) {
            if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                
                // Process all flow sensors
                for (uint8_t i = 0; i < num_sensors; i++) {
                    if (!flow_sensors[i].enabled) continue;
                    
                    FlowSensorConfig_t* sensor = &flow_sensors[i];
                    FlowData_t flow_data = {0};
                    esp_err_t ret;
                    
                    // Stop counter to prevent counting during read
                    ret = pcnt_unit_stop(sensor->pcnt_unit);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to stop counter for sensor %s: %s", 
                                sensor->name, esp_err_to_name(ret));
                        continue;
                    }
                    
                    // Read current pulse count (hardware counts only rising edges)
                    int pulse_count = 0;
                    ret = pcnt_unit_get_count(sensor->pcnt_unit, &pulse_count);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to read counter for sensor %s: %s", 
                                sensor->name, esp_err_to_name(ret));
                        pcnt_unit_start(sensor->pcnt_unit);  // Try to restart anyway
                        continue;
                    }
                    
                    // Ensure pulse_count is non-negative
                    if (pulse_count < 0) {
                        pulse_count = 0;
                    }
                    
                    // Calculate elapsed time since last reset
                    uint32_t elapsed_ms = current_time - sensor->last_reset_time;
                    
                    // Check for counter saturation (reached 12-bit limit)
                    bool counter_saturated = (pulse_count >= 4095);
                    
                    // Check if accumulation window complete AND pulses detected
                    if ((elapsed_ms >= FLOW_ACCUMULATION_WINDOW_MS && pulse_count > 0) || counter_saturated) {
                        // Window complete or saturated - store data and reset
                        if (counter_saturated) {
                            ESP_LOGW(TAG, "Sensor %s: Counter saturated at 4095 (high flow rate)", sensor->name);
                        }
                        
                        flow_data.pulses = (uint16_t)pulse_count;  // Cast to 12-bit
                        flow_data.milliseconds = (pulse_count > 0) ? elapsed_ms : 0;  // Zero milliseconds if no pulses
                        flow_data.newData = 1;
                        
                        // Clear counter (atomic operation)
                        ret = pcnt_unit_clear_count(sensor->pcnt_unit);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to clear counter for sensor %s: %s", 
                                    sensor->name, esp_err_to_name(ret));
                        }
                        
                        // Update reset timestamp
                        sensor->last_reset_time = current_time;
                    } else {
                        // Window not complete - store current data but continue accumulating
                        flow_data.pulses = (uint16_t)pulse_count;
                        flow_data.milliseconds = (pulse_count > 0) ? elapsed_ms : 0;  // Zero milliseconds if no pulses
                        flow_data.newData = 0;
                    }
                    
                    // Restart counter (continues counting from 0 if reset, or continues if not reset)
                    ret = pcnt_unit_start(sensor->pcnt_unit);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to start counter for sensor %s: %s", 
                                sensor->name, esp_err_to_name(ret));
                    }
                    
                    // Update genericSens_ structure based on sensor index
                    switch (i) {
                        case 0:
                            GENERICSENS_PACK_FLOWDATA(flow_data, genericSens_.generic.flowData1);
                            break;
                        case 1:
                            GENERICSENS_PACK_FLOWDATA(flow_data, genericSens_.generic.flowData2);
                            break;
                        case 2:
                            GENERICSENS_PACK_FLOWDATA(flow_data, genericSens_.generic.flowData3);
                            break;
                        default:
                            ESP_LOGW(TAG, "Invalid sensor index: %d", i);
                            break;
                    }
                }
                
                xSemaphoreGive(sensor_data.mutex);
                
                // Signal completion (for 1000ms sensor coordination)
                sensor_coordination_signal_completion(SENSOR_EVENT_FLOW_COMPLETE);
            } else {
                ESP_LOGW(TAG, "Failed to acquire mutex for flow sensor update");
            }
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
 * Initialize PCNT Flow Manager
 * 
 * Initializes the PCNT hardware and creates the flow manager task.
 * Must be called before registering sensors.
 */
esp_err_t pcnt_flow_manager_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "PCNT Flow Manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing PCNT Flow Manager...");
    
    // Initialize sensor array
    memset(flow_sensors, 0, sizeof(flow_sensors));
    num_sensors = 0;
    
    // Create flow manager task
    BaseType_t xResult = xTaskCreate(
        vPcntFlowManagerTask,
        "PcntFlowMgr",
        TASK_STACK_SIZE_CRITICAL,
        NULL,
        TASK_PRIORITY_CRITICAL,  // Highest priority
        &xPcntFlowManagerTaskHandle
    );
    
    if (xResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create PCNT Flow Manager task");
        return ESP_FAIL;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "PCNT Flow Manager initialized successfully");
    
    return ESP_OK;
}

/**
 * Register Flow Sensor
 * 
 * Registers a flow sensor with the PCNT flow manager.
 * Configures GPIO pin and PCNT unit for pulse counting.
 */
esp_err_t pcnt_flow_manager_register_sensor(gpio_num_t gpio_pin, const char* name) {
    if (!initialized) {
        ESP_LOGE(TAG, "PCNT Flow Manager not initialized - call pcnt_flow_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (num_sensors >= MAX_FLOW_SENSORS) {
        ESP_LOGE(TAG, "Maximum flow sensors (%d) already registered", MAX_FLOW_SENSORS);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Registering flow sensor: %s (GPIO%d)", name, gpio_pin);
    
    esp_err_t ret;
    
    // Create PCNT unit (new API)
    // Note: GPIO will be configured automatically by pcnt_new_channel()
    // IMPORTANT: low_limit must be negative (less than 0) for PCNT to work
    pcnt_unit_config_t unit_config = {
        .high_limit = 4095,  // Upper limit (12-bit max)
        .low_limit = -4095,  // Lower limit (must be negative!)
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ret = pcnt_new_unit(&unit_config, &pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit for sensor %s: %s", name, esp_err_to_name(ret));
        return ret;
    }
    
    // Configure glitch filter (reduces noise and false triggers)
    // Using 1000ns filter like the example (1μs)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,  // 1μs filter (matches example)
    };
    ret = pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set glitch filter for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Create PCNT channel (new API)
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = gpio_pin,
        .level_gpio_num = -1,  // No control pin needed
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ret = pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Configure channel to count only on rising edge (leading edge)
    // Count increases on rising edge, holds on falling edge
    ret = pcnt_channel_set_edge_action(pcnt_chan, 
                                        PCNT_CHANNEL_EDGE_ACTION_INCREASE, 
                                        PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set edge action for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Set level action (not used for simple pulse counting)
    ret = pcnt_channel_set_level_action(pcnt_chan, 
                                         PCNT_CHANNEL_LEVEL_ACTION_KEEP, 
                                         PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level action for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Enable PCNT unit
    ret = pcnt_unit_enable(pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Clear counter
    ret = pcnt_unit_clear_count(pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear counter for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Start counter
    ret = pcnt_unit_start(pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start counter for sensor %s: %s", name, esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        return ret;
    }
    
    // Store sensor configuration
    flow_sensors[num_sensors].pcnt_unit = pcnt_unit;
    flow_sensors[num_sensors].gpio_pin = gpio_pin;
    flow_sensors[num_sensors].name = name;
    flow_sensors[num_sensors].last_reset_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    flow_sensors[num_sensors].enabled = true;
    
    num_sensors++;
    
    ESP_LOGI(TAG, "Registered flow sensor: %s (GPIO%d)", name, gpio_pin);
    
    return ESP_OK;
}

/**
 * Get Number of Registered Sensors
 */
uint8_t pcnt_flow_manager_get_sensor_count(void) {
    return num_sensors;
}

