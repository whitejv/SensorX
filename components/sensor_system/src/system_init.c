/*
 * system_init.c - System Initialization Task Implementation
 *
 * This file implements the system initialization tasks for the ESP32 FreeRTOS
 * sensor system. The system init task performs basic system setup and
 * creates core application tasks.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_chip_info.h>
#include <esp_idf_version.h>
#include <string.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <cJSON.h>

#if CONFIG_SPIRAM
#include <esp_spiram.h>
#endif

#include "system_init.h"
#include "config.h"
#include "types.h"
#include "pins.h"
#include "watchdog.h"
#include "error_recovery.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "i2c_manager.h"
#include "sensor_coordination.h"
#include "sensor.h"
#include "onewire_temp_manager.h"
#include "panic_stats.h"
#include "json_constants.h"

static const char *TAG = "SYSTEM_INIT";

// Static JSON serialization buffer (reduces malloc/free overhead)
static char system_monitor_json_buffer[JSON_BUFFER_SIZE];

// Pre-built JSON templates for nested objects (reduces allocation overhead)
static cJSON* g_firmware_info_json = NULL;
static cJSON* g_wifi_json_template = NULL;
static cJSON* g_mqtt_json_template = NULL;
static cJSON* g_i2c_json_template = NULL;
static cJSON* g_onewire_json_template = NULL;
static cJSON* g_fan_json_template = NULL;
static cJSON* g_watchdog_json_template = NULL;
static cJSON* g_error_json_template = NULL;
static cJSON* g_i2c_errors_json_template = NULL;

// Task handles for heartbeat, information, and monitoring tasks
TaskHandle_t xHeartbeatTaskHandle = NULL;
TaskHandle_t xSystemInfoTaskHandle = NULL;
TaskHandle_t xSystemMonitorTaskHandle = NULL;

// Task handles for sensor acquisition and MQTT publishing
TaskHandle_t xSensorAcquisitionTaskHandle = NULL;
TaskHandle_t xMqttPublisherTaskHandle = NULL;

/*
 * Heartbeat Task
 * Simple task that periodically prints status to verify FreeRTOS is running
 * Also blinks onboard LED for visual indication
 */
void vHeartbeatTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // Register with watchdog
    watchdog_register_current_task("Heartbeat", 6000);

    // Initialize onboard LED
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << PIN_ONBOARD_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));

    // Start with LED off
    ESP_ERROR_CHECK(gpio_set_level(PIN_ONBOARD_LED, 0));

    const TickType_t xDelay = pdMS_TO_TICKS(5000);  // 5 second delay
    static bool led_state = false;  // Track LED state

    for (;;) {
        // Toggle LED state for visual heartbeat
        led_state = !led_state;
        ESP_ERROR_CHECK(gpio_set_level(PIN_ONBOARD_LED, led_state ? 1 : 0));

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        vTaskDelay(xDelay);
    }
}

/*
 * System Information Task
 * Displays comprehensive system information during startup
 */
void vSystemInfoTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // Small delay to ensure logging is ready
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "=== SensorX System Information ===");
    ESP_LOGI(TAG, "System Info Task: Starting information display...");
    ESP_LOGI(TAG, "Firmware Version: %s v%d.%d",
             FIRMWARE_NAME, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 Chip Revision: %d", chip_info.revision);

    // Flash size can be determined from partition table or chip capabilities
    ESP_LOGI(TAG, "ESP32 Flash: Available");

    ESP_LOGI(TAG, "FreeRTOS Kernel Version: %s", tskKERNEL_VERSION_NUMBER);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);

    // PSRAM information
#if CONFIG_SPIRAM
    size_t psram_size = esp_spiram_get_size();
    ESP_LOGI(TAG, "PSRAM Available: Yes, Size: %d MB", psram_size / (1024 * 1024));
#else
    ESP_LOGI(TAG, "PSRAM Available: No");
#endif

    ESP_LOGI(TAG, "=== System Information Complete ===");

    // Task completes and deletes itself
    vTaskDelete(NULL);
}

/*
 * System Monitoring Task
 * Continuously monitors system health and resource usage
 * Includes fan control based on temperature
 */

/**
 * Print verbose sensor data to serial output
 * 
 * This function displays all sensor readings in a human-readable format.
 * Intended for debugging and development until MQTT monitoring is available.
 * 
 * Should be called once per second from a task or timer.
 * Only compiled when VERBOSE=1.
 */
#if VERBOSE
static void print_sensor_data_verbose(void) {
    FlowData_t flow1, flow2, flow3;
    
    // Take mutex to safely read sensor data
    if (sensor_data.mutex == NULL) {
        ESP_LOGW("SENSOR_DATA", "Sensor data mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW("SENSOR_DATA", "Failed to acquire sensor data mutex");
        return;
    }
    
    // Extract flow data
    GENERICSENS_EXTRACT_FLOWDATA(genericSens_.generic.flowData1, flow1);
    GENERICSENS_EXTRACT_FLOWDATA(genericSens_.generic.flowData2, flow2);
    GENERICSENS_EXTRACT_FLOWDATA(genericSens_.generic.flowData3, flow3);
    
    // Print header
    ESP_LOGI("SENSOR_DATA", "========================================");
    ESP_LOGI("SENSOR_DATA", "Sensor Data Snapshot (Cycle: %ld)", genericSens_.generic.cycle_count);
    ESP_LOGI("SENSOR_DATA", "========================================");
    ESP_LOGI("SENSOR_DATA", "");
    
    // Flow Sensors
    ESP_LOGI("SENSOR_DATA", "FLOW SENSORS:");
    ESP_LOGI("SENSOR_DATA", "  Flow1: %u pulses | %ums | newData: %u", 
             flow1.pulses, flow1.milliseconds, flow1.newData);
    ESP_LOGI("SENSOR_DATA", "  Flow2: %u pulses | %ums | newData: %u", 
             flow2.pulses, flow2.milliseconds, flow2.newData);
    ESP_LOGI("SENSOR_DATA", "  Flow3: %u pulses | %ums | newData: %u", 
             flow3.pulses, flow3.milliseconds, flow3.newData);
    ESP_LOGI("SENSOR_DATA", "");
    
    // ADC Sensors
    ESP_LOGI("SENSOR_DATA", "ADC SENSORS:");
    ESP_LOGI("SENSOR_DATA", "  ADC Basic: %ld (raw)", genericSens_.generic.adc_sensor);
    ESP_LOGI("SENSOR_DATA", "  ADC x1 (Pump1): %.3f A", genericSens_.generic.adc_x1);
    ESP_LOGI("SENSOR_DATA", "  ADC x2 (Pump2): %.3f A", genericSens_.generic.adc_x2);
    ESP_LOGI("SENSOR_DATA", "  ADC x3 (Pump3): %.3f A", genericSens_.generic.adc_x3);
    ESP_LOGI("SENSOR_DATA", "  ADC x4 (Pump4): %.3f A", genericSens_.generic.adc_x4);
    ESP_LOGI("SENSOR_DATA", "  ADC x5 (Tank Pressure): %.3f PSI", genericSens_.generic.adc_x5);
    ESP_LOGI("SENSOR_DATA", "  ADC x6 (Water P1): %.3f PSI", genericSens_.generic.adc_x6);
    ESP_LOGI("SENSOR_DATA", "  ADC x7 (Water P2): %.3f PSI", genericSens_.generic.adc_x7);
    ESP_LOGI("SENSOR_DATA", "  ADC x8 (Water P3): %.3f PSI", genericSens_.generic.adc_x8);
    ESP_LOGI("SENSOR_DATA", "");
    
    // GPIO Sensors
    ESP_LOGI("SENSOR_DATA", "GPIO SENSORS:");
    ESP_LOGI("SENSOR_DATA", "  GPIO Basic: 0x%02lX (GPIO4:%ld GPIO5:%ld)", 
             (unsigned long)genericSens_.generic.gpio_sensor,
             (genericSens_.generic.gpio_sensor >> 1) & 1,
             (genericSens_.generic.gpio_sensor >> 2) & 1);
    ESP_LOGI("SENSOR_DATA", "  GPIO x1 (MCP PortA): 0x%02lX", 
             (unsigned long)genericSens_.generic.GPIO_x1);
    ESP_LOGI("SENSOR_DATA", "  GPIO x2 (MCP PortB): 0x%02lX", 
             (unsigned long)genericSens_.generic.GPIO_x2);
    ESP_LOGI("SENSOR_DATA", "");
    
    // Temperature Sensors
    ESP_LOGI("SENSOR_DATA", "TEMPERATURE SENSORS:");
    ESP_LOGI("SENSOR_DATA", "  Temp1 (int): %ld°F", genericSens_.generic.temp1);
    ESP_LOGI("SENSOR_DATA", "  Temp1 (float): %.1f°F", *(float*)&genericSens_.generic.temp1_f);
    ESP_LOGI("SENSOR_DATA", "  Temp2 (DS18B20-1): %.1f°F", genericSens_.generic.temp2);
    ESP_LOGI("SENSOR_DATA", "  Temp3 (DS18B20-2): %.1f°F", genericSens_.generic.temp3);
    ESP_LOGI("SENSOR_DATA", "  Temp4 (DS18B20-3): %.1f°F", genericSens_.generic.temp4);
    ESP_LOGI("SENSOR_DATA", "  TempX (Internal): %.1f°F", genericSens_.generic.tempx);
    ESP_LOGI("SENSOR_DATA", "  Sensors Detected: %ld", genericSens_.generic.tempSensorcount);
    ESP_LOGI("SENSOR_DATA", "");
    
    // Environmental Sensors
    ESP_LOGI("SENSOR_DATA", "ENVIRONMENTAL:");
    ESP_LOGI("SENSOR_DATA", "  Temperature (Internal): %.1f°F", genericSens_.generic.tempx);
    ESP_LOGI("SENSOR_DATA", "  Pressure: %.4f PSI (not available)", genericSens_.generic.pressurex);
    ESP_LOGI("SENSOR_DATA", "  Humidity: %.1f%% (not available)", genericSens_.generic.humidity);
    ESP_LOGI("SENSOR_DATA", "");
    
    // System Data
    ESP_LOGI("SENSOR_DATA", "SYSTEM:");
    ESP_LOGI("SENSOR_DATA", "  Cycle Count: %ld", genericSens_.generic.cycle_count);
    ESP_LOGI("SENSOR_DATA", "  FW Version: 0x%04lX", (unsigned long)genericSens_.generic.fw_version);
    ESP_LOGI("SENSOR_DATA", "========================================");
    
    // Release mutex
    xSemaphoreGive(sensor_data.mutex);
}
#endif // VERBOSE

/**
 * @brief Initialize JSON templates for system monitor messages.
 * Creates reusable JSON objects to reduce allocation overhead.
 */
static void init_json_templates(void) {
    // Firmware info template (static data, never changes)
    g_firmware_info_json = cJSON_CreateObject();
    if (g_firmware_info_json != NULL) {
        cJSON_AddStringToObject(g_firmware_info_json, JSON_KEY_FIRMWARE_VERSION, FIRMWARE_VERSION_STRING);
        cJSON_AddStringToObject(g_firmware_info_json, JSON_KEY_BUILD_DATE, __DATE__);
        cJSON_AddStringToObject(g_firmware_info_json, JSON_KEY_BUILD_TIME, __TIME__);
    }
    
    // WiFi template (minimal structure, values updated each cycle)
    g_wifi_json_template = cJSON_CreateObject();
    if (g_wifi_json_template != NULL) {
        cJSON_AddStringToObject(g_wifi_json_template, JSON_KEY_WIFI_STATUS, JSON_STATUS_DISCONNECTED);
    }
    
    // MQTT template
    g_mqtt_json_template = cJSON_CreateObject();
    if (g_mqtt_json_template != NULL) {
        cJSON_AddStringToObject(g_mqtt_json_template, JSON_KEY_MQTT_STATUS, JSON_STATUS_DISCONNECTED);
    }
    
    // I2C template
    g_i2c_json_template = cJSON_CreateObject();
    if (g_i2c_json_template != NULL) {
        cJSON_AddNumberToObject(g_i2c_json_template, JSON_KEY_I2C_DEVICE_COUNT, 0);
        cJSON_AddStringToObject(g_i2c_json_template, JSON_KEY_I2C_STATUS, JSON_STATUS_INITIALIZED);
    }
    
    // I2C errors template
    g_i2c_errors_json_template = cJSON_CreateObject();
    if (g_i2c_errors_json_template != NULL) {
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_TOTAL, 0);
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_TIMEOUTS, 0);
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_NACKS, 0);
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_BUS, 0);
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_DEVICE_NOT_FOUND, 0);
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_TX_FAILURES, 0);
        cJSON_AddNumberToObject(g_i2c_errors_json_template, JSON_KEY_I2C_ERRORS_LAST_TIME_MS, 0);
    }
    
    // One-Wire template
    g_onewire_json_template = cJSON_CreateObject();
    if (g_onewire_json_template != NULL) {
        cJSON_AddNumberToObject(g_onewire_json_template, JSON_KEY_ONEWIRE_SENSOR_COUNT, 0);
        cJSON_AddNumberToObject(g_onewire_json_template, JSON_KEY_ONEWIRE_SENSOR_COUNT_DATA, 0);
        cJSON_AddStringToObject(g_onewire_json_template, JSON_KEY_ONEWIRE_STATUS, JSON_STATUS_NO_SENSORS);
    }
    
    // Fan template
    g_fan_json_template = cJSON_CreateObject();
    if (g_fan_json_template != NULL) {
        cJSON_AddBoolToObject(g_fan_json_template, JSON_KEY_FAN_ON, false);
        cJSON_AddStringToObject(g_fan_json_template, JSON_KEY_FAN_TEMP_SOURCE, "none");
    }
    
    // Error recovery template
    g_error_json_template = cJSON_CreateObject();
    if (g_error_json_template != NULL) {
        cJSON_AddNumberToObject(g_error_json_template, JSON_KEY_ERRORS_TOTAL, 0);
        cJSON_AddNumberToObject(g_error_json_template, JSON_KEY_ERRORS_RECOVERED, 0);
        cJSON_AddNumberToObject(g_error_json_template, JSON_KEY_ERRORS_CRITICAL, 0);
    }
    
    // Watchdog template
    g_watchdog_json_template = cJSON_CreateObject();
    if (g_watchdog_json_template != NULL) {
        cJSON_AddNumberToObject(g_watchdog_json_template, JSON_KEY_WATCHDOG_TOTAL_FEEDS, 0);
        cJSON_AddNumberToObject(g_watchdog_json_template, JSON_KEY_WATCHDOG_TIMEOUTS, 0);
        cJSON_AddNumberToObject(g_watchdog_json_template, JSON_KEY_WATCHDOG_TASKS_MONITORED, 0);
    }
    
    ESP_LOGI(TAG, "JSON templates initialized");
}

void vSystemMonitorTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // ============================================================================
    // Initialize Fan Control GPIO
    // ============================================================================
    // First reset the pin to clear any previous configuration
    gpio_reset_pin(FAN_CONTROL_GPIO_PIN);
    
    gpio_config_t fan_config = {
        .pin_bit_mask = (1ULL << FAN_CONTROL_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&fan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure fan control GPIO%d: %s", FAN_CONTROL_GPIO_PIN, esp_err_to_name(ret));
        // Continue anyway - fan control will be disabled
    } else {
        // Start with fan OFF initially (will be set to ON if no temp source)
        gpio_set_level(FAN_CONTROL_GPIO_PIN, 0);
        ESP_LOGI(TAG, "Fan control GPIO%d initialized", FAN_CONTROL_GPIO_PIN);
    }


    // Register with watchdog
    watchdog_register_current_task("SysMonitor", MONITOR_INTERVAL_MS + 1000);

    ESP_LOGI(TAG, "System Monitor Task: Started successfully!");

    // Initialize monitoring variables on first run
    static uint32_t startTime = 0;
    static size_t lastHeapFree = 0;
    static size_t minHeapFree = 0;
    static UBaseType_t lastTaskCount = 0;
    static bool initialized = false;

    // Fan control state variables (shared between fan control and monitoring sections)
    static float current_temp = 0.0;
    static bool temp_valid = false;
    static bool using_fallback = false;
    static bool fan_state = false;  // Track fan state (ON=true, OFF=false)
    static uint32_t log_counter = 0;
    static bool no_temp_warning_logged = false;  // Track if "no temp source" warning was logged

    if (!initialized) {
        // Initialize JSON templates once
        init_json_templates();
        
        startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        lastHeapFree = esp_get_free_heap_size();
        minHeapFree = esp_get_free_heap_size();
        lastTaskCount = uxTaskGetNumberOfTasks();
        initialized = true;
    }

    const TickType_t xMonitorInterval = pdMS_TO_TICKS(MONITOR_INTERVAL_MS);

    for (;;) {
        // ============================================================================
        // Fan Control (every loop iteration = 1000ms)
        // ============================================================================
        // Try primary temperature source (One-Wire internal temp from genericSens_.tempx)
        float internal_temp = 0.0;
        bool internal_temp_valid = false;

        if (sensor_data.mutex != NULL) {
            if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(FAN_CONTROL_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                internal_temp = genericSens_.generic.tempx;
                xSemaphoreGive(sensor_data.mutex);

                // Validate internal temperature reading
                bool temp_in_range = (internal_temp > FAN_CONTROL_TEMP_VALID_MIN_F && 
                                     internal_temp < FAN_CONTROL_TEMP_VALID_MAX_F);
                bool temp_not_zero = (internal_temp != 0.0);
                
                if (temp_in_range && temp_not_zero) {
                    current_temp = internal_temp;
                    temp_valid = true;
                    using_fallback = false;
                    internal_temp_valid = true;
                    no_temp_warning_logged = false;  // Reset warning flag when valid temp found
                } else {
                    // Log why validation failed (only occasionally to avoid spam)
                    static uint32_t validation_log_counter = 0;
                    if ((validation_log_counter++ % 10) == 0) {  // Log every 10 seconds
                        ESP_LOGW(TAG, "Internal temp validation failed: temp=%.1f°F, in_range=%d, not_zero=%d (min=%.1f, max=%.1f)", 
                                 internal_temp, temp_in_range, temp_not_zero,
                                 FAN_CONTROL_TEMP_VALID_MIN_F, FAN_CONTROL_TEMP_VALID_MAX_F);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Failed to acquire mutex for temperature read");
            }
        } else {
            ESP_LOGW(TAG, "Sensor data mutex is NULL");
        }

        // Fallback: ESP32-C6 doesn't have a simple internal temperature sensor API
        // If One-Wire internal temp is unavailable, temp_valid will remain false and fan will default to ON
        if (!internal_temp_valid) {
            temp_valid = false;
        }

        // Apply fan control based on valid temperature source
        if (temp_valid) {
            float threshold = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;

            if (current_temp >= threshold) {
                gpio_set_level(FAN_CONTROL_GPIO_PIN, 1);  // Fan ON
                fan_state = true;
            } else {
                gpio_set_level(FAN_CONTROL_GPIO_PIN, 0);  // Fan OFF
                fan_state = false;
            }
        } else {
            // No valid temperature source - default to fan ON for safety
            gpio_set_level(FAN_CONTROL_GPIO_PIN, 1);  // Fan ON (safe default)
            fan_state = true;  // Track state
            if (!no_temp_warning_logged) {
                ESP_LOGW(TAG, "No valid temperature source - fan ON for safety");
                ESP_LOGW(TAG, "Internal temp read: %.1f°F (valid: %s, min: %.1f, max: %.1f)", 
                         internal_temp, 
                         (internal_temp > FAN_CONTROL_TEMP_VALID_MIN_F && 
                          internal_temp < FAN_CONTROL_TEMP_VALID_MAX_F && 
                          internal_temp != 0.0) ? "YES" : "NO",
                         FAN_CONTROL_TEMP_VALID_MIN_F,
                         FAN_CONTROL_TEMP_VALID_MAX_F);
                no_temp_warning_logged = true;  // Only log once
            }
        }

        // ============================================================================
        // Verbose Sensor Data Display (every SENSOR_DATA_VERBOSE_INTERVAL_MS)
        // ============================================================================
#if VERBOSE
        static uint32_t last_sensor_print = 0;  // Verbose sensor data display timing
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_sensor_print >= SENSOR_DATA_VERBOSE_INTERVAL_MS) {
            print_sensor_data_verbose();
            last_sensor_print = now;
        }
#endif

        // ============================================================================
        // System Monitoring (every 10th iteration = 10000ms)
        // ============================================================================
        log_counter++;
        if (log_counter >= (MONITOR_LOG_INTERVAL_MS / MONITOR_INTERVAL_MS)) {
            log_counter = 0;

            // Check production gate for system monitor output
            bool system_monitor_output_enabled = true;
#if PRODUCTION_MODE
            system_monitor_output_enabled = (ENABLE_SYSTEM_MONITOR_OUTPUT == 1);
#else
            system_monitor_output_enabled = true;  // Always enabled in development mode
#endif

            // Calculate uptime (always needed for MQTT)
            uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t uptimeMs = currentTime - startTime;
            uint32_t uptimeSeconds = uptimeMs / 1000;
            uint32_t uptimeHours = uptimeSeconds / 3600;
            uint32_t uptimeMinutes = (uptimeSeconds % 3600) / 60;
            uint32_t uptimeSecs = uptimeSeconds % 60;

            // Get current memory stats (always needed for MQTT)
            size_t currentHeapFree = esp_get_free_heap_size();

            // Update minimum heap tracking
            if (currentHeapFree < minHeapFree) {
                minHeapFree = currentHeapFree;
            }

            // Get task information (always needed for MQTT)
            UBaseType_t currentTaskCount = uxTaskGetNumberOfTasks();

            // Display monitoring information (gated by production mode)
            if (system_monitor_output_enabled) {
                ESP_LOGI(TAG, "--- System Monitor ---");
                ESP_LOGI(TAG, "Firmware: %s | Build: %s %s", 
                         FIRMWARE_VERSION_STRING, __DATE__, __TIME__);
                ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu (HH:MM:SS)",
                         uptimeHours, uptimeMinutes, uptimeSecs);
                ESP_LOGI(TAG, "Free Heap: %zu bytes (Min: %zu bytes)",
                         currentHeapFree, minHeapFree);
                ESP_LOGI(TAG, "Active Tasks: %d", currentTaskCount);
                
                // Get and display panic statistics
                PanicStats_t panic_stats;
                if (panic_stats_get(&panic_stats) == ESP_OK) {
                    ESP_LOGI(TAG, "Panics: %lu total, %lu since clean boot | Last Reset: %s",
                             panic_stats.total_panic_count,
                             panic_stats.panics_since_clean,
                             panic_stats_get_reset_reason_string(panic_stats.last_reset_reason));
                }
            }

            // Show task count changes
            if (system_monitor_output_enabled && currentTaskCount != lastTaskCount) {
                ESP_LOGW(TAG, "Task count changed from %d to %d",
                         lastTaskCount, currentTaskCount);
                lastTaskCount = currentTaskCount;
            }

            // Show heap usage trend
            if (system_monitor_output_enabled && currentHeapFree != lastHeapFree) {
                int32_t heapChange = (int32_t)currentHeapFree - (int32_t)lastHeapFree;
                ESP_LOGD(TAG, "Heap change: %ld bytes", heapChange);
                lastHeapFree = currentHeapFree;
            }

            // Get system state from error recovery
            SystemState_t state = error_recovery_get_system_state();
            if (system_monitor_output_enabled && state != SYSTEM_STATE_RUNNING) {
                ESP_LOGW(TAG, "System state: %d", state);
            }

            // Get error statistics
            ErrorRecoveryStats_t error_stats;
            error_recovery_get_stats(&error_stats);
            if (system_monitor_output_enabled && error_stats.total_errors > 0) {
                ESP_LOGI(TAG, "Errors: total=%lu, recovered=%lu, critical=%lu",
                         error_stats.total_errors,
                         error_stats.recovered_errors,
                         error_stats.critical_errors);
            }

            // Get watchdog statistics
            WatchdogStats_t wdt_stats;
            watchdog_get_stats(&wdt_stats);
            if (system_monitor_output_enabled) {
                ESP_LOGI(TAG, "Watchdog: feeds=%lu, timeouts=%lu, tasks=%lu",
                         wdt_stats.totalFeeds,
                         wdt_stats.timeoutCount,
                         wdt_stats.tasksMonitored);
            }

            // Get WiFi status
            WiFiStatus_t wifi_status = wifi_manager_get_status();
            bool wifi_connected = wifi_manager_is_connected();
            
            if (system_monitor_output_enabled) {
                if (wifi_connected) {
                    char ip_str[16];
                    int8_t rssi = wifi_manager_get_rssi();
                    WiFiStats_t wifi_stats;
                    wifi_manager_get_stats(&wifi_stats);
                    
                    if (wifi_manager_get_ip_address(ip_str, sizeof(ip_str)) == ESP_OK) {
                        ESP_LOGI(TAG, "WiFi: Connected | IP: %s | RSSI: %d dBm | Uptime: %lu sec",
                                 ip_str, rssi, wifi_stats.uptime);
                    } else {
                        ESP_LOGI(TAG, "WiFi: Connected | RSSI: %d dBm | Uptime: %lu sec",
                                 rssi, wifi_stats.uptime);
                    }
                } else {
                    const char* status_str = "Unknown";
                    switch (wifi_status) {
                        case WIFI_STATUS_DISCONNECTED:
                            status_str = "Disconnected";
                            break;
                        case WIFI_STATUS_CONNECTING:
                            status_str = "Connecting";
                            break;
                        case WIFI_STATUS_RECONNECTING:
                            status_str = "Reconnecting";
                            break;
                        case WIFI_STATUS_ERROR:
                            status_str = "Error";
                            break;
                        default:
                            status_str = "Unknown";
                            break;
                    }
                    ESP_LOGI(TAG, "WiFi: %s", status_str);
                }
            }

            // Get MQTT status
            MQTTStatus_t mqtt_status = mqtt_manager_get_status();
            bool mqtt_connected = mqtt_manager_is_connected();
            
            // MQTT status - only report errors (no log for normal operation)
            if (!mqtt_connected && mqtt_status == MQTT_STATUS_ERROR) {
                ESP_LOGE(TAG, "MQTT: Error state detected");
            }

            // Get I2C status (cache results to avoid scanning every cycle)
            static uint8_t cached_i2c_addresses[16] = {0};
            static size_t cached_device_count = 0;
            static uint32_t last_i2c_scan_time = 0;
            static bool i2c_scan_complete = false;
            
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            // Scan I2C devices every 60 seconds
            uint32_t scan_interval = 60000;  // 60 seconds
            bool should_scan = !i2c_scan_complete || 
                              (current_time - last_i2c_scan_time > scan_interval);
            
            if (i2c_manager_is_initialized()) {
                if (should_scan) {
                    esp_err_t scan_result = i2c_manager_scan_devices(cached_i2c_addresses, 
                                                                      sizeof(cached_i2c_addresses), 
                                                                      &cached_device_count);
                    if (scan_result == ESP_OK) {
                        last_i2c_scan_time = current_time;
                        i2c_scan_complete = true;
                    }
                }
                
                // Display cached results
                if (cached_device_count > 0) {
                    // Build address string more efficiently to reduce stack usage
                    char addr_str[64] = "";
                    char *ptr = addr_str;
                    size_t remaining = sizeof(addr_str) - 1;
                    
                    for (size_t i = 0; i < cached_device_count && remaining > 8; i++) {
                        if (i > 0) {
                            int written = snprintf(ptr, remaining, ", ");
                            if (written > 0 && written < (int)remaining) {
                                ptr += written;
                                remaining -= written;
                            }
                        }
                        int written = snprintf(ptr, remaining, "0x%02X", cached_i2c_addresses[i]);
                        if (written > 0 && written < (int)remaining) {
                            ptr += written;
                            remaining -= written;
                        } else {
                            break;  // Not enough space
                        }
                    }
                    
                    // Get I2C error statistics
                    I2CStats_t i2c_stats;
                    i2c_manager_get_stats(&i2c_stats);
                    
                    if (system_monitor_output_enabled) {
                        if (i2c_stats.total_errors > 0) {
                            ESP_LOGI(TAG, "I2C: Initialized | Devices: %zu [%s] | Errors: %lu (timeouts: %lu, bus: %lu, not_found: %lu, tx_fail: %lu)",
                                     cached_device_count, addr_str,
                                     i2c_stats.total_errors,
                                     i2c_stats.timeout_errors,
                                     i2c_stats.bus_errors,
                                     i2c_stats.device_not_found,
                                     i2c_stats.transaction_failures);
                        } else {
                            ESP_LOGI(TAG, "I2C: Initialized | Devices: %zu [%s] | Errors: 0", cached_device_count, addr_str);
                        }
                    }
                } else if (i2c_scan_complete) {
                    // Get I2C error statistics
                    I2CStats_t i2c_stats;
                    i2c_manager_get_stats(&i2c_stats);
                    
                    if (system_monitor_output_enabled) {
                        if (i2c_stats.total_errors > 0) {
                            ESP_LOGI(TAG, "I2C: Initialized | No devices found | Errors: %lu (timeouts: %lu, bus: %lu, not_found: %lu, tx_fail: %lu)",
                                     i2c_stats.total_errors,
                                     i2c_stats.timeout_errors,
                                     i2c_stats.bus_errors,
                                     i2c_stats.device_not_found,
                                     i2c_stats.transaction_failures);
                        } else {
                            ESP_LOGI(TAG, "I2C: Initialized | No devices found | Errors: 0");
                        }
                    }
                } else {
                    if (system_monitor_output_enabled) {
                        ESP_LOGI(TAG, "I2C: Initialized | Scanning...");
                    }
                }
            } else {
                if (system_monitor_output_enabled) {
                    ESP_LOGI(TAG, "I2C: Not initialized");
                }
                // Reset cache when I2C is not initialized
                cached_device_count = 0;
                i2c_scan_complete = false;
            }

            // Get One-Wire sensor status
            uint8_t onewire_sensor_count = onewire_temp_manager_get_sensor_count();
            uint8_t onewire_sensor_count_from_data = 0;
            if (sensor_data.mutex != NULL) {
                if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    onewire_sensor_count_from_data = (uint8_t)genericSens_.generic.tempSensorcount;
                    xSemaphoreGive(sensor_data.mutex);
                }
            }
            
            if (system_monitor_output_enabled) {
                if (onewire_sensor_count > 0) {
                    if (onewire_sensor_count == onewire_sensor_count_from_data) {
                        ESP_LOGI(TAG, "One-Wire: Initialized | Sensors: %d", onewire_sensor_count);
                    } else {
                        ESP_LOGW(TAG, "One-Wire: Initialized | Sensors: %d (mismatch: manager=%d, data=%d)", 
                                 onewire_sensor_count, onewire_sensor_count, onewire_sensor_count_from_data);
                    }
                } else {
                    ESP_LOGI(TAG, "One-Wire: Initialized | Sensors: 0 (no sensors detected)");
                }
            }

            // Display fan control status and temperature
            // Note: fan_state is tracked in a variable since gpio_get_level() doesn't work reliably for output pins
            if (system_monitor_output_enabled) {
                if (temp_valid) {
                    const char* temp_source = using_fallback ? "Die" : "Ambient";
                    float threshold_display = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;
                    ESP_LOGI(TAG, "Fan: %s | %s Temp: %.1f°F | Threshold: %.1f°F",
                             fan_state ? "ON" : "OFF",
                             temp_source,
                             current_temp,
                             threshold_display);
                } else {
                    ESP_LOGI(TAG, "Fan: %s | Temp: N/A | Threshold: N/A",
                             fan_state ? "ON" : "OFF");
                }
            }

            // Publish system monitor message to MQTT (every 10 seconds)
            if (mqtt_connected && wifi_connected) {
                cJSON *json = cJSON_CreateObject();
                if (json != NULL) {
                    // Firmware and build information (matches serial output) - use pre-built template
                    if (g_firmware_info_json != NULL) {
                        cJSON_AddItemReferenceToObject(json, JSON_KEY_FIRMWARE_VERSION, 
                                                       cJSON_GetObjectItem(g_firmware_info_json, JSON_KEY_FIRMWARE_VERSION));
                        cJSON_AddItemReferenceToObject(json, JSON_KEY_BUILD_DATE, 
                                                       cJSON_GetObjectItem(g_firmware_info_json, JSON_KEY_BUILD_DATE));
                        cJSON_AddItemReferenceToObject(json, JSON_KEY_BUILD_TIME, 
                                                       cJSON_GetObjectItem(g_firmware_info_json, JSON_KEY_BUILD_TIME));
                    } else {
                        // Fallback if template not initialized
                        cJSON_AddStringToObject(json, JSON_KEY_FIRMWARE_VERSION, FIRMWARE_VERSION_STRING);
                        cJSON_AddStringToObject(json, JSON_KEY_BUILD_DATE, __DATE__);
                        cJSON_AddStringToObject(json, JSON_KEY_BUILD_TIME, __TIME__);
                    }
                    
                    // System status - using static strings
                    char uptime_str[16];
                    snprintf(uptime_str, sizeof(uptime_str), "%02lu:%02lu:%02lu", 
                             uptimeHours, uptimeMinutes, uptimeSecs);
                    cJSON_AddStringToObject(json, JSON_KEY_UPTIME, uptime_str);
                    cJSON_AddNumberToObject(json, JSON_KEY_UPTIME_SEC, uptimeSeconds);
                    cJSON_AddNumberToObject(json, JSON_KEY_FREE_HEAP, currentHeapFree);
                    cJSON_AddNumberToObject(json, JSON_KEY_MIN_FREE_HEAP, minHeapFree);
                    cJSON_AddNumberToObject(json, JSON_KEY_ACTIVE_TASKS, currentTaskCount);
                    
                    // Panic statistics (matches serial output) - using static strings
                    PanicStats_t panic_stats;
                    if (panic_stats_get(&panic_stats) == ESP_OK) {
                        cJSON_AddNumberToObject(json, JSON_KEY_PANIC_COUNT, panic_stats.total_panic_count);
                        cJSON_AddNumberToObject(json, JSON_KEY_PANIC_COUNT_CLEAN, panic_stats.panics_since_clean);
                        cJSON_AddStringToObject(json, JSON_KEY_LAST_RESET_REASON, 
                                               panic_stats_get_reset_reason_string(panic_stats.last_reset_reason));
                        if (panic_stats.last_panic_timestamp > 0) {
                            cJSON_AddNumberToObject(json, JSON_KEY_LAST_PANIC_TIMESTAMP_MS, panic_stats.last_panic_timestamp);
                        }
                    }
                    
                    // Calculate idle time percentage (matches serial output)
                    float idle_percentage = 0.0f;
                    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
                    if (num_tasks > 0) {
                        // Use static buffer to avoid stack overflow (max reasonable task count: 32)
                        static TaskStatus_t task_status_array[32];
                        uint32_t total_runtime = 0;
                        UBaseType_t num_tasks_found = uxTaskGetSystemState(
                            task_status_array,
                            (num_tasks < 32) ? num_tasks : 32,
                            &total_runtime
                        );
                        
                        if (total_runtime > 0 && num_tasks_found > 0) {
                            // Find idle task(s) - ESP32-C6 has single core, so typically "IDLE" or "IDLE0"
                            uint32_t idle_runtime = 0;
                            for (UBaseType_t i = 0; i < num_tasks_found; i++) {
                                const char* task_name = task_status_array[i].pcTaskName;
                                if (task_name != NULL && 
                                    (strcmp(task_name, "IDLE") == 0 || 
                                     strcmp(task_name, "IDLE0") == 0 ||
                                     strncmp(task_name, "IDLE", 4) == 0)) {
                                    idle_runtime += task_status_array[i].ulRunTimeCounter;
                                }
                            }
                            
                            // Calculate idle percentage
                            idle_percentage = (idle_runtime * 100.0f) / total_runtime;
                        }
                    }
                    cJSON_AddNumberToObject(json, JSON_KEY_IDLE_TIME_PERCENT, idle_percentage);
                    
                    // WiFi status - using template
                    cJSON *wifi_json = NULL;
                    if (g_wifi_json_template != NULL) {
                        wifi_json = cJSON_Duplicate(g_wifi_json_template, 1);
                    } else {
                        wifi_json = cJSON_CreateObject();
                    }
                    if (wifi_json != NULL) {
                        if (wifi_connected) {
                            char ip_str[16];
                            int8_t rssi = wifi_manager_get_rssi();
                            WiFiStats_t wifi_stats;
                            wifi_manager_get_stats(&wifi_stats);
                            
                            if (wifi_manager_get_ip_address(ip_str, sizeof(ip_str)) == ESP_OK) {
                                cJSON_AddStringToObject(wifi_json, JSON_KEY_WIFI_IP, ip_str);
                            }
                            cJSON_AddNumberToObject(wifi_json, JSON_KEY_WIFI_RSSI, rssi);
                            cJSON_AddNumberToObject(wifi_json, JSON_KEY_WIFI_UPTIME_SEC, wifi_stats.uptime);
                            cJSON_SetValuestring(cJSON_GetObjectItem(wifi_json, JSON_KEY_WIFI_STATUS), JSON_STATUS_CONNECTED);
                        } else {
                            cJSON_SetValuestring(cJSON_GetObjectItem(wifi_json, JSON_KEY_WIFI_STATUS), JSON_STATUS_DISCONNECTED);
                        }
                        cJSON_AddItemToObject(json, "wifi", wifi_json);
                    }
                    
                    // MQTT status - using template
                    cJSON *mqtt_json = NULL;
                    if (g_mqtt_json_template != NULL) {
                        mqtt_json = cJSON_Duplicate(g_mqtt_json_template, 1);
                    } else {
                        mqtt_json = cJSON_CreateObject();
                    }
                    if (mqtt_json != NULL) {
                        if (mqtt_connected) {
                            char mqtt_ip[16];
                            MQTTStats_t mqtt_stats;
                            mqtt_manager_get_stats(&mqtt_stats);
                            
                            if (mqtt_manager_get_broker_ip(mqtt_ip, sizeof(mqtt_ip)) == ESP_OK) {
                                cJSON_AddStringToObject(mqtt_json, JSON_KEY_MQTT_BROKER_IP, mqtt_ip);
                            }
                            cJSON_AddBoolToObject(mqtt_json, JSON_KEY_MQTT_CONNECTED_TO_PROD, mqtt_stats.connectedToProd);
                            cJSON_AddNumberToObject(mqtt_json, JSON_KEY_MQTT_PUBLISH_SUCCESS, mqtt_stats.publishSuccess);
                            cJSON_AddNumberToObject(mqtt_json, JSON_KEY_MQTT_PUBLISH_FAILURES, mqtt_stats.publishFailures);
                            // Update status string
                            cJSON_DeleteItemFromObject(mqtt_json, JSON_KEY_MQTT_STATUS);
                            cJSON_AddStringToObject(mqtt_json, JSON_KEY_MQTT_STATUS, JSON_STATUS_CONNECTED);
                        } else {
                            // Update status string
                            cJSON_DeleteItemFromObject(mqtt_json, JSON_KEY_MQTT_STATUS);
                            cJSON_AddStringToObject(mqtt_json, JSON_KEY_MQTT_STATUS, JSON_STATUS_DISCONNECTED);
                        }
                        cJSON_AddItemToObject(json, "mqtt", mqtt_json);
                    }
                    
                    // I2C devices - using template
                    if (i2c_manager_is_initialized()) {
                        cJSON *i2c_json = NULL;
                        if (g_i2c_json_template != NULL) {
                            i2c_json = cJSON_Duplicate(g_i2c_json_template, 1);
                        } else {
                            i2c_json = cJSON_CreateObject();
                        }
                        if (i2c_json != NULL) {
                            if (cached_device_count > 0) {
                                cJSON *devices_array = cJSON_CreateArray();
                                if (devices_array != NULL) {
                                    for (size_t i = 0; i < cached_device_count; i++) {
                                        cJSON_AddItemToArray(devices_array, cJSON_CreateNumber(cached_i2c_addresses[i]));
                                    }
                                    // Update device count
                                    cJSON_DeleteItemFromObject(i2c_json, JSON_KEY_I2C_DEVICE_COUNT);
                                    cJSON_AddNumberToObject(i2c_json, JSON_KEY_I2C_DEVICE_COUNT, cached_device_count);
                                    cJSON_AddItemToObject(i2c_json, JSON_KEY_I2C_DEVICES, devices_array);
                                }
                            } else {
                                // Update device count
                                cJSON_DeleteItemFromObject(i2c_json, JSON_KEY_I2C_DEVICE_COUNT);
                                cJSON_AddNumberToObject(i2c_json, JSON_KEY_I2C_DEVICE_COUNT, 0);
                            }
                            // Update status string
                            cJSON_DeleteItemFromObject(i2c_json, JSON_KEY_I2C_STATUS);
                            cJSON_AddStringToObject(i2c_json, JSON_KEY_I2C_STATUS, JSON_STATUS_INITIALIZED);
                            
                            // Add I2C error statistics - using template
                            I2CStats_t i2c_stats;
                            i2c_manager_get_stats(&i2c_stats);
                            if (i2c_stats.total_errors > 0) {
                                cJSON *i2c_errors_json = NULL;
                                if (g_i2c_errors_json_template != NULL) {
                                    i2c_errors_json = cJSON_Duplicate(g_i2c_errors_json_template, 1);
                                } else {
                                    i2c_errors_json = cJSON_CreateObject();
                                }
                                if (i2c_errors_json != NULL) {
                                    // Update error values (delete and re-add to ensure correct values)
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_TOTAL);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_TOTAL, i2c_stats.total_errors);
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_TIMEOUTS);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_TIMEOUTS, i2c_stats.timeout_errors);
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_NACKS);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_NACKS, i2c_stats.nack_errors);
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_BUS);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_BUS, i2c_stats.bus_errors);
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_DEVICE_NOT_FOUND);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_DEVICE_NOT_FOUND, i2c_stats.device_not_found);
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_TX_FAILURES);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_TX_FAILURES, i2c_stats.transaction_failures);
                                    cJSON_DeleteItemFromObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_LAST_TIME_MS);
                                    cJSON_AddNumberToObject(i2c_errors_json, JSON_KEY_I2C_ERRORS_LAST_TIME_MS, i2c_stats.last_error_time_ms);
                                    cJSON_AddItemToObject(i2c_json, "errors", i2c_errors_json);
                                }
                            }
                            
                            cJSON_AddItemToObject(json, "i2c", i2c_json);
                        }
                    }
                    
                    // One-Wire sensors - using template
                    uint8_t onewire_sensor_count = onewire_temp_manager_get_sensor_count();
                    uint8_t onewire_sensor_count_from_data = 0;
                    if (sensor_data.mutex != NULL) {
                        if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            onewire_sensor_count_from_data = (uint8_t)genericSens_.generic.tempSensorcount;
                            xSemaphoreGive(sensor_data.mutex);
                        }
                    }
                    
                    cJSON *onewire_json = NULL;
                    if (g_onewire_json_template != NULL) {
                        onewire_json = cJSON_Duplicate(g_onewire_json_template, 1);
                    } else {
                        onewire_json = cJSON_CreateObject();
                    }
                    if (onewire_json != NULL) {
                        // Update sensor counts
                        cJSON_DeleteItemFromObject(onewire_json, JSON_KEY_ONEWIRE_SENSOR_COUNT);
                        cJSON_AddNumberToObject(onewire_json, JSON_KEY_ONEWIRE_SENSOR_COUNT, onewire_sensor_count);
                        cJSON_DeleteItemFromObject(onewire_json, JSON_KEY_ONEWIRE_SENSOR_COUNT_DATA);
                        cJSON_AddNumberToObject(onewire_json, JSON_KEY_ONEWIRE_SENSOR_COUNT_DATA, onewire_sensor_count_from_data);
                        // Update status string
                        cJSON_DeleteItemFromObject(onewire_json, JSON_KEY_ONEWIRE_STATUS);
                        if (onewire_sensor_count > 0) {
                            cJSON_AddStringToObject(onewire_json, JSON_KEY_ONEWIRE_STATUS, JSON_STATUS_INITIALIZED);
                        } else {
                            cJSON_AddStringToObject(onewire_json, JSON_KEY_ONEWIRE_STATUS, JSON_STATUS_NO_SENSORS);
                        }
                        cJSON_AddItemToObject(json, "onewire", onewire_json);
                    }
                    
                    // Fan control status - using template
                    cJSON *fan_json = NULL;
                    if (g_fan_json_template != NULL) {
                        fan_json = cJSON_Duplicate(g_fan_json_template, 1);
                    } else {
                        fan_json = cJSON_CreateObject();
                    }
                    if (fan_json != NULL) {
                        // Update fan state (bool) - delete and re-add to ensure correct value
                        cJSON_DeleteItemFromObject(fan_json, JSON_KEY_FAN_ON);
                        cJSON_AddBoolToObject(fan_json, JSON_KEY_FAN_ON, fan_state);
                        if (temp_valid) {
                            cJSON_AddNumberToObject(fan_json, JSON_KEY_FAN_TEMP_F, current_temp);
                            // Update temp source string
                            cJSON_DeleteItemFromObject(fan_json, JSON_KEY_FAN_TEMP_SOURCE);
                            cJSON_AddStringToObject(fan_json, JSON_KEY_FAN_TEMP_SOURCE, 
                                                   using_fallback ? "die" : "ambient");
                            float threshold_display = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;
                            cJSON_AddNumberToObject(fan_json, JSON_KEY_FAN_THRESHOLD_F, threshold_display);
                        } else {
                            cJSON_AddNullToObject(fan_json, JSON_KEY_FAN_TEMP_F);
                            // Update temp source string
                            cJSON_DeleteItemFromObject(fan_json, JSON_KEY_FAN_TEMP_SOURCE);
                            cJSON_AddStringToObject(fan_json, JSON_KEY_FAN_TEMP_SOURCE, "none");
                        }
                        cJSON_AddItemToObject(json, "fan", fan_json);
                    }
                    
                    // Error statistics - using template
                    ErrorRecoveryStats_t error_stats;
                    error_recovery_get_stats(&error_stats);
                    if (error_stats.total_errors > 0) {
                        cJSON *error_json = NULL;
                        if (g_error_json_template != NULL) {
                            error_json = cJSON_Duplicate(g_error_json_template, 1);
                        } else {
                            error_json = cJSON_CreateObject();
                        }
                        if (error_json != NULL) {
                            // Update error values
                            cJSON_DeleteItemFromObject(error_json, JSON_KEY_ERRORS_TOTAL);
                            cJSON_AddNumberToObject(error_json, JSON_KEY_ERRORS_TOTAL, error_stats.total_errors);
                            cJSON_DeleteItemFromObject(error_json, JSON_KEY_ERRORS_RECOVERED);
                            cJSON_AddNumberToObject(error_json, JSON_KEY_ERRORS_RECOVERED, error_stats.recovered_errors);
                            cJSON_DeleteItemFromObject(error_json, JSON_KEY_ERRORS_CRITICAL);
                            cJSON_AddNumberToObject(error_json, JSON_KEY_ERRORS_CRITICAL, error_stats.critical_errors);
                            cJSON_AddItemToObject(json, "errors", error_json);
                        }
                    }
                    
                    // Watchdog statistics - using template
                    WatchdogStats_t wdt_stats;
                    watchdog_get_stats(&wdt_stats);
                    cJSON *wdt_json = NULL;
                    if (g_watchdog_json_template != NULL) {
                        wdt_json = cJSON_Duplicate(g_watchdog_json_template, 1);
                    } else {
                        wdt_json = cJSON_CreateObject();
                    }
                    if (wdt_json != NULL) {
                        // Update watchdog values
                        cJSON_DeleteItemFromObject(wdt_json, JSON_KEY_WATCHDOG_TOTAL_FEEDS);
                        cJSON_AddNumberToObject(wdt_json, JSON_KEY_WATCHDOG_TOTAL_FEEDS, wdt_stats.totalFeeds);
                        cJSON_DeleteItemFromObject(wdt_json, JSON_KEY_WATCHDOG_TIMEOUTS);
                        cJSON_AddNumberToObject(wdt_json, JSON_KEY_WATCHDOG_TIMEOUTS, wdt_stats.timeoutCount);
                        cJSON_DeleteItemFromObject(wdt_json, JSON_KEY_WATCHDOG_TASKS_MONITORED);
                        cJSON_AddNumberToObject(wdt_json, JSON_KEY_WATCHDOG_TASKS_MONITORED, wdt_stats.tasksMonitored);
                        cJSON_AddItemToObject(json, "watchdog", wdt_json);
                    }
                    
                    // Add timestamp - using static string
                    cJSON_AddNumberToObject(json, JSON_KEY_TIMESTAMP, currentTime);
                    
                    // Serialize and publish - using static buffer
                    int json_len = cJSON_PrintPreallocated(json, system_monitor_json_buffer, JSON_BUFFER_SIZE, false);
                    if (json_len > 0 && json_len < JSON_BUFFER_SIZE) {
                        esp_err_t ret = mqtt_manager_publish_json(
                            MQTT_TOPIC_SYSTEM_MONITOR,
                            system_monitor_json_buffer,
                            MQTT_SYSTEM_MONITOR_QOS,
                            false
                        );
                        if (ret == ESP_OK) {
                            // System monitor message published successfully - no log (only report errors)
                        } else {
                            ESP_LOGW(TAG, "Failed to publish system monitor message: %s", esp_err_to_name(ret));
                        }
                    } else if (json_len >= JSON_BUFFER_SIZE) {
                        ESP_LOGE(TAG, "System monitor JSON buffer overflow (needed %d bytes, buffer is %d bytes)", 
                                 json_len, JSON_BUFFER_SIZE);
                    } else {
                        ESP_LOGE(TAG, "Failed to serialize system monitor JSON");
                    }
                    
                    cJSON_Delete(json);
                } else {
                    ESP_LOGE(TAG, "Failed to create system monitor JSON object");
                }
            } else {
                ESP_LOGD(TAG, "Skipping system monitor MQTT publish - mqtt_connected=%d, wifi_connected=%d", 
                         mqtt_connected, wifi_connected);
            }
        }

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        // Calculate and display idle time percentage
        if (log_counter == 0) {
            // Check production gate for system monitor output
            bool system_monitor_output_enabled = true;
#if PRODUCTION_MODE
            system_monitor_output_enabled = (ENABLE_SYSTEM_MONITOR_OUTPUT == 1);
#else
            system_monitor_output_enabled = true;  // Always enabled in development mode
#endif

            if (system_monitor_output_enabled) {
                // Get task system state to calculate idle time percentage
                UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
                if (num_tasks > 0) {
                    // Use static buffer to avoid stack overflow (max reasonable task count: 32)
                    static TaskStatus_t task_status_array[32];
                    uint32_t total_runtime = 0;
                    UBaseType_t num_tasks_found = uxTaskGetSystemState(
                        task_status_array,
                        (num_tasks < 32) ? num_tasks : 32,
                        &total_runtime
                    );
                    
                    if (total_runtime > 0 && num_tasks_found > 0) {
                        // Find idle task(s) - ESP32-C6 has single core, so typically "IDLE" or "IDLE0"
                        uint32_t idle_runtime = 0;
                        for (UBaseType_t i = 0; i < num_tasks_found; i++) {
                            const char* task_name = task_status_array[i].pcTaskName;
                            if (task_name != NULL && 
                                (strcmp(task_name, "IDLE") == 0 || 
                                 strcmp(task_name, "IDLE0") == 0 ||
                                 strncmp(task_name, "IDLE", 4) == 0)) {
                                idle_runtime += task_status_array[i].ulRunTimeCounter;
                            }
                        }
                        
                        // Calculate idle percentage
                        float idle_percentage = (idle_runtime * 100.0f) / total_runtime;
                        ESP_LOGI(TAG, "Idle Time: %.1f%%", idle_percentage);
                    }
                }
                
                ESP_LOGI(TAG, "--- Monitor Complete ---");
            }
        }

        // Wait for next monitoring interval
        vTaskDelay(xMonitorInterval);
    }
}

/*
 * Info Response Task
 * Provides comprehensive system information
 */
void vInfoResponseTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "=== System Information ===");

    // Basic system info
    ESP_LOGI(TAG, "Firmware: %s v%d.%d", FIRMWARE_NAME,
             FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);

    // ESP32 chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 Chip: Revision %d", chip_info.revision);

    // Flash size can be determined from partition table or chip capabilities
    ESP_LOGI(TAG, "Flash: Available");

    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());

    // Task count and states
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    ESP_LOGI(TAG, "Active Tasks: %d", taskCount);

    // Memory information
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "Free Heap: %zu bytes", free_heap);
    ESP_LOGI(TAG, "Min Free Heap: %zu bytes", min_free_heap);

    // PSRAM information
#if CONFIG_SPIRAM
    size_t psram_size = esp_spiram_get_size();
    size_t psram_free = esp_spiram_get_free_size();
    ESP_LOGI(TAG, "PSRAM: %zu bytes total, %zu bytes free", psram_size, psram_free);
#endif

    // Uptime calculation
    uint32_t uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t uptime_sec = uptime_ms / 1000;
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    uint32_t seconds = uptime_sec % 60;
    ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu", hours, minutes, seconds);

    ESP_LOGI(TAG, "=== End System Information ===");

    // Task completes and deletes itself
    vTaskDelete(NULL);
}
