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

static const char *TAG = "SYSTEM_INIT";

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
 */
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
    ESP_LOGI("SENSOR_DATA", "  TempX (BME280): %.1f°F", genericSens_.generic.tempx);
    ESP_LOGI("SENSOR_DATA", "  Sensors Detected: %ld", genericSens_.generic.tempSensorcount);
    ESP_LOGI("SENSOR_DATA", "");
    
    // Environmental Sensors
    ESP_LOGI("SENSOR_DATA", "ENVIRONMENTAL (BME280):");
    ESP_LOGI("SENSOR_DATA", "  Temperature: %.1f°F", genericSens_.generic.tempx);
    ESP_LOGI("SENSOR_DATA", "  Pressure: %.4f PSI", genericSens_.generic.pressurex);
    ESP_LOGI("SENSOR_DATA", "  Humidity: %.1f%%", genericSens_.generic.humidity);
    ESP_LOGI("SENSOR_DATA", "");
    
    // System Data
    ESP_LOGI("SENSOR_DATA", "SYSTEM:");
    ESP_LOGI("SENSOR_DATA", "  Cycle Count: %ld", genericSens_.generic.cycle_count);
    ESP_LOGI("SENSOR_DATA", "  FW Version: 0x%04lX", (unsigned long)genericSens_.generic.fw_version);
    ESP_LOGI("SENSOR_DATA", "========================================");
    
    // Release mutex
    xSemaphoreGive(sensor_data.mutex);
}

void vSystemMonitorTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    // ============================================================================
    // Initialize Fan Control GPIO
    // ============================================================================
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
    static uint32_t log_counter = 0;
    static bool no_temp_warning_logged = false;  // Track if "no temp source" warning was logged
    
    // Verbose sensor data display timing
    static uint32_t last_sensor_print = 0;

    if (!initialized) {
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
        // Try primary temperature source (BME280 from genericSens_.tempx)
        float bme280_temp = 0.0;
        bool bme280_valid = false;

        if (sensor_data.mutex != NULL) {
            if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(FAN_CONTROL_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                bme280_temp = genericSens_.generic.tempx;
                xSemaphoreGive(sensor_data.mutex);

                // Validate BME280 reading
                if (bme280_temp > FAN_CONTROL_TEMP_VALID_MIN_F && 
                    bme280_temp < FAN_CONTROL_TEMP_VALID_MAX_F && 
                    bme280_temp != 0.0) {
                    current_temp = bme280_temp;
                    temp_valid = true;
                    using_fallback = false;
                    bme280_valid = true;
                    no_temp_warning_logged = false;  // Reset warning flag when valid temp found
                }
            }
        }

        // Fallback: ESP32-C6 doesn't have a simple internal temperature sensor API
        // If BME280 is unavailable, temp_valid will remain false and fan will default to ON
        if (!bme280_valid) {
            temp_valid = false;
        }

        // Apply fan control based on valid temperature source
        if (temp_valid) {
            float threshold = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;

            if (current_temp >= threshold) {
                gpio_set_level(FAN_CONTROL_GPIO_PIN, 1);  // Fan ON
            } else {
                gpio_set_level(FAN_CONTROL_GPIO_PIN, 0);  // Fan OFF
            }
        } else {
            // No valid temperature source - default to fan ON for safety
            gpio_set_level(FAN_CONTROL_GPIO_PIN, 1);  // Fan ON (safe default)
            if (!no_temp_warning_logged) {
                ESP_LOGW(TAG, "No valid temperature source - fan ON for safety");
                no_temp_warning_logged = true;  // Only log once
            }
        }

        // ============================================================================
        // Verbose Sensor Data Display (every SENSOR_DATA_VERBOSE_INTERVAL_MS)
        // ============================================================================
#if SENSOR_DATA_VERBOSE_ENABLED
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_sensor_print >= SENSOR_DATA_VERBOSE_INTERVAL_MS) {
            print_sensor_data_verbose();
            last_sensor_print = now;
        }
#endif

        // ============================================================================
        // System Monitoring (every 5th iteration = 5000ms)
        // ============================================================================
        log_counter++;
        if (log_counter >= (MONITOR_LOG_INTERVAL_MS / MONITOR_INTERVAL_MS)) {
            log_counter = 0;

        // Calculate uptime
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t uptimeMs = currentTime - startTime;
        uint32_t uptimeSeconds = uptimeMs / 1000;
        uint32_t uptimeHours = uptimeSeconds / 3600;
        uint32_t uptimeMinutes = (uptimeSeconds % 3600) / 60;
        uint32_t uptimeSecs = uptimeSeconds % 60;

        // Get current memory stats
        size_t currentHeapFree = esp_get_free_heap_size();

        // Update minimum heap tracking
        if (currentHeapFree < minHeapFree) {
            minHeapFree = currentHeapFree;
        }

        // Get task information
        UBaseType_t currentTaskCount = uxTaskGetNumberOfTasks();

        // Display monitoring information
        ESP_LOGI(TAG, "--- System Monitor ---");
        ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu (HH:MM:SS)",
                 uptimeHours, uptimeMinutes, uptimeSecs);
        ESP_LOGI(TAG, "Free Heap: %zu bytes (Min: %zu bytes)",
                 currentHeapFree, minHeapFree);
        ESP_LOGI(TAG, "Active Tasks: %d", currentTaskCount);

        // Show task count changes
        if (currentTaskCount != lastTaskCount) {
            ESP_LOGW(TAG, "Task count changed from %d to %d",
                     lastTaskCount, currentTaskCount);
            lastTaskCount = currentTaskCount;
        }

        // Show heap usage trend
        if (currentHeapFree != lastHeapFree) {
            int32_t heapChange = (int32_t)currentHeapFree - (int32_t)lastHeapFree;
            ESP_LOGD(TAG, "Heap change: %ld bytes", heapChange);
            lastHeapFree = currentHeapFree;
        }

        // Get system state from error recovery
        SystemState_t state = error_recovery_get_system_state();
        if (state != SYSTEM_STATE_RUNNING) {
            ESP_LOGW(TAG, "System state: %d", state);
        }

        // Get error statistics
        ErrorRecoveryStats_t error_stats;
        error_recovery_get_stats(&error_stats);
        if (error_stats.total_errors > 0) {
            ESP_LOGI(TAG, "Errors: total=%lu, recovered=%lu, critical=%lu",
                     error_stats.total_errors,
                     error_stats.recovered_errors,
                     error_stats.critical_errors);
        }

        // Get watchdog statistics
        WatchdogStats_t wdt_stats;
        watchdog_get_stats(&wdt_stats);
        ESP_LOGI(TAG, "Watchdog: feeds=%lu, timeouts=%lu, tasks=%lu",
                 wdt_stats.totalFeeds,
                 wdt_stats.timeoutCount,
                 wdt_stats.tasksMonitored);

        // Get WiFi status
        WiFiStatus_t wifi_status = wifi_manager_get_status();
        bool wifi_connected = wifi_manager_is_connected();
        
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

        // Get MQTT status
        MQTTStatus_t mqtt_status = mqtt_manager_get_status();
        bool mqtt_connected = mqtt_manager_is_connected();
        
        if (mqtt_connected) {
            char mqtt_ip[16];
            MQTTStats_t mqtt_stats;
            mqtt_manager_get_stats(&mqtt_stats);
            
            if (mqtt_manager_get_broker_ip(mqtt_ip, sizeof(mqtt_ip)) == ESP_OK) {
                ESP_LOGI(TAG, "MQTT: Connected | Broker: %s (%s) | Pub: %lu/%lu (ok/fail)",
                         mqtt_ip, mqtt_stats.connectedToProd ? "PROD" : "DEV",
                         mqtt_stats.publishSuccess, mqtt_stats.publishFailures);
            } else {
                ESP_LOGI(TAG, "MQTT: Connected | Pub: %lu/%lu (ok/fail)",
                         mqtt_stats.publishSuccess, mqtt_stats.publishFailures);
            }
        } else {
            const char* mqtt_status_str = "Unknown";
            switch (mqtt_status) {
                case MQTT_STATUS_DISCONNECTED: mqtt_status_str = "Disconnected"; break;
                case MQTT_STATUS_CONNECTING: mqtt_status_str = "Connecting"; break;
                case MQTT_STATUS_RECONNECTING: mqtt_status_str = "Reconnecting"; break;
                case MQTT_STATUS_ERROR: mqtt_status_str = "Error"; break;
                default: mqtt_status_str = "Unknown"; break;
            }
            ESP_LOGI(TAG, "MQTT: %s", mqtt_status_str);
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
                    ESP_LOGI(TAG, "I2C: Initialized | Devices: %zu [%s]", cached_device_count, addr_str);
                } else if (i2c_scan_complete) {
                    ESP_LOGI(TAG, "I2C: Initialized | No devices found");
                } else {
                    ESP_LOGI(TAG, "I2C: Initialized | Scanning...");
                }
            } else {
                ESP_LOGI(TAG, "I2C: Not initialized");
                // Reset cache when I2C is not initialized
                cached_device_count = 0;
                i2c_scan_complete = false;
            }

            // Display fan control status and temperature
            bool fan_state = gpio_get_level(FAN_CONTROL_GPIO_PIN);
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

        // Send heartbeat to watchdog
        watchdog_task_heartbeat();

        if (log_counter == 0) {
            ESP_LOGI(TAG, "--- Monitor Complete ---");
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
