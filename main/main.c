// main/main.c - ESP-IDF Application Entry Point

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"         // From sensor_system component
#include "types.h"          // From sensor_system component
#include "system_init.h"    // From sensor_system component
#include "watchdog.h"       // From sensor_system component
#include "error_recovery.h" // From sensor_system component
#include "wifi_manager.h"   // From sensor_system component
#include "i2c_manager.h"    // From sensor_system component
#include "mqtt_manager.h"   // From sensor_system component
#include "sensor_acquisition.h" // From sensor_system component
#include "mqtt_publisher.h"     // From sensor_system component
#include "sensor_coordination.h" // From sensor_system component
#include "sensor.h"          // From sensor_system component
#include "onewire_temp_manager.h" // From sensor_system component
#include "pcnt_flow_manager.h" // From sensor_system component
#include "i2c_adc_manager.h" // From sensor_system component
#include "i2c_gpio_manager.h" // From sensor_system component
#include "i2c_env_manager.h" // From sensor_system component
#include "pins.h"            // From sensor_system component - GPIO pin definitions

static const char *TAG = "MAIN";

void app_main(void) {
    // Initialize ESP_LOG system
    esp_log_level_set("*", ESP_LOG_INFO);  // Set global log level
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Set specific tag level

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "===  Sensor Base ESP-IDF Starting  ===");
    ESP_LOGI(TAG, "======================================");

    // 1. Initialize error recovery FIRST (needed for error handling)
    esp_err_t ret = error_recovery_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize error recovery: %s", esp_err_to_name(ret));
        abort();
    }

    ret = error_recovery_start_task();
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to start error recovery task");
        abort();
    }

    // 2. Initialize watchdog timer
    ret = watchdog_init(WATCHDOG_TIMEOUT_SEC);
    if (ret != ESP_OK) {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_CRITICAL, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to initialize watchdog: %s", esp_err_to_name(ret));
        abort();
    }

    ret = watchdog_start_task();
    if (ret != pdPASS) {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_CRITICAL, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to start watchdog task");
        abort();
    }

    // ============================================================================
    // HARDWARE DEBUGGER SUPPORT: Uncomment the following to disable watchdog
    // when using a hardware debugger. The watchdog will trigger if execution
    // is paused (e.g., breakpoints), so disable it during debugging sessions.
    // ============================================================================
    // watchdog_disable();
    // ESP_LOGW(TAG, "WATCHDOG DISABLED FOR DEBUGGING - DO NOT USE IN PRODUCTION!");
    // ============================================================================

    // 3. Initialize WiFi Manager (non-critical - continue if fails)
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "Failed to initialize WiFi manager: %s", esp_err_to_name(ret));
        // Continue anyway - WiFi is not critical for basic operation
    } else {
        ret = wifi_manager_start_task();
        if (ret != pdPASS) {
            error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to start WiFi manager task");
            // Continue anyway - WiFi is not critical for basic operation
        } else {
            ESP_LOGI(TAG, "WiFi manager initialized and task started");
            
#if WIFI_AUTO_CONNECT
            // Auto-connect to WiFi on startup
            ret = wifi_manager_connect(WIFI_SSID, WIFI_PASSWORD);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initiate WiFi connection: %s", esp_err_to_name(ret));
                error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            } else {
                ESP_LOGI(TAG, "WiFi connection initiated to: %s", WIFI_SSID);
            }
#endif
        }
    }

    // 4. Initialize Sensor Coordination System (Phase 0 - Foundation)
    // This must be initialized before any sensor managers
    ret = sensor_coordination_init();
    if (ret != ESP_OK) {
        error_report(ERROR_MEMORY_ALLOCATION, ERROR_SEVERITY_CRITICAL, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to initialize sensor coordination: %s", esp_err_to_name(ret));
        abort();
    }
    ESP_LOGI(TAG, "Sensor coordination system initialized successfully");

    // 5. Initialize One-Wire Temperature Manager (Phase 2)
    ret = onewire_temp_manager_init();
    if (ret != ESP_OK) {
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "Failed to initialize One-Wire Temperature Manager: %s", esp_err_to_name(ret));
        // Continue anyway - One-Wire sensors are not critical for basic operation
    } else {
        ESP_LOGI(TAG, "One-Wire Temperature Manager initialized successfully");
    }

    // 6. Initialize PCNT Flow Manager (Phase 3)
    ret = pcnt_flow_manager_init();
    if (ret != ESP_OK) {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_CRITICAL, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to initialize PCNT Flow Manager: %s", esp_err_to_name(ret));
        abort();
    }
    ESP_LOGI(TAG, "PCNT Flow Manager initialized successfully");
    
    // Register flow sensors
    ret = pcnt_flow_manager_register_sensor(PIN_FLOW_SENSOR_1, "Flow1");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register Flow Sensor 1: %s", esp_err_to_name(ret));
    }
    
    ret = pcnt_flow_manager_register_sensor(PIN_FLOW_SENSOR_2, "Flow2");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register Flow Sensor 2: %s", esp_err_to_name(ret));
    }
    
    ret = pcnt_flow_manager_register_sensor(PIN_FLOW_SENSOR_3, "Flow3");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register Flow Sensor 3: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Registered %d flow sensor(s)", pcnt_flow_manager_get_sensor_count());

    // 7. Initialize I2C Manager (non-critical - continue if fails)
    ret = i2c_manager_init();
    if (ret != ESP_OK) {
        error_report(ERROR_I2C_TIMEOUT, ERROR_SEVERITY_ERROR, "app_main", NULL);
        ESP_LOGE(TAG, "Failed to initialize I2C manager: %s", esp_err_to_name(ret));
        // Continue anyway - some sensors may not be critical
    } else {
        ESP_LOGI(TAG, "I2C manager initialized successfully");
        
        // Initialize I2C ADC Manager (Phase 5)
        ret = i2c_adc_manager_init();
        if (ret != ESP_OK) {
            error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to initialize I2C ADC Manager: %s", esp_err_to_name(ret));
            // Continue anyway - ADC devices are optional
        } else {
            ESP_LOGI(TAG, "I2C ADC Manager initialized successfully");
            
            // Register ADS1015 (12-bit ADC at 0x48)
            ret = i2c_adc_manager_register_device(ADS1015_I2C_ADDRESS, false);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to register ADS1015: %s", esp_err_to_name(ret));
                // Continue - device may not be present
            }
            
            // Register ADS1115 (16-bit ADC at 0x49)
            ret = i2c_adc_manager_register_device(ADS1115_I2C_ADDRESS, true);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to register ADS1115: %s", esp_err_to_name(ret));
                // Continue - device may not be present
            }
            
            ESP_LOGI(TAG, "Registered %d ADC device(s)", i2c_adc_manager_get_device_count());
        }
        
        // Initialize I2C GPIO Manager (Phase 4)
        ret = i2c_gpio_manager_init();
        if (ret != ESP_OK) {
            error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to initialize I2C GPIO Manager: %s", esp_err_to_name(ret));
            // Continue anyway - GPIO expander is optional
        } else {
            ESP_LOGI(TAG, "I2C GPIO Manager initialized successfully");
            
            // Register MCP23017 GPIO expander (at 0x20)
            ret = i2c_gpio_manager_register_expander(MCP23017_I2C_ADDRESS);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to register MCP23017 GPIO expander: %s", esp_err_to_name(ret));
                // Continue - device may not be present
            } else {
                ESP_LOGI(TAG, "Registered MCP23017 GPIO expander");
            }
            
            ESP_LOGI(TAG, "Registered %d GPIO expander(s)", i2c_gpio_manager_get_device_count());
        }
        
        // Initialize I2C Environmental Manager (BME280)
        ret = i2c_env_manager_init();
        if (ret != ESP_OK) {
            error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to initialize I2C Environmental Manager: %s", esp_err_to_name(ret));
            // Continue anyway - BME280 is optional
        } else {
            ESP_LOGI(TAG, "I2C Environmental Manager initialized successfully");
            
            // Register BME280 sensor (try both common addresses)
            // Try 0x77 first (most common), then 0x76
            ret = i2c_env_manager_register_bme280(0x77);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "BME280 not found at 0x77, trying 0x76...");
                ret = i2c_env_manager_register_bme280(0x76);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "BME280 not found at either address (0x76 or 0x77): %s", esp_err_to_name(ret));
                    // Continue - device may not be present
                } else {
                    ESP_LOGI(TAG, "Registered BME280 sensor at address 0x76");
                }
            } else {
                ESP_LOGI(TAG, "Registered BME280 sensor at address 0x77");
            }
            
            ESP_LOGI(TAG, "Registered %d environmental sensor(s)", i2c_env_manager_get_sensor_count());
        }
    }

    // 8. Initialize MQTT Manager (after WiFi, non-critical - continue if fails)
    // Note: MQTT init requires WiFi to be connected, so we'll initialize it
    // after WiFi connection is established
    if (wifi_manager_is_connected()) {
        ret = mqtt_manager_init();
        if (ret != ESP_OK) {
            error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to initialize MQTT manager: %s", esp_err_to_name(ret));
            // Continue anyway - MQTT is not critical for basic operation
        } else {
            ESP_LOGI(TAG, "MQTT manager initialized successfully");
            // Try to connect to MQTT broker
            ret = mqtt_manager_connect();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to connect to MQTT broker: %s", esp_err_to_name(ret));
                error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            } else {
                ESP_LOGI(TAG, "MQTT connection initiated");
            }
        }
    } else {
        ESP_LOGI(TAG, "WiFi not connected yet, MQTT initialization deferred");
    }

    // 8. Create the system information task
    BaseType_t xResult = xTaskCreate(
        vSystemInfoTask,              // Task function
        "SystemInfo",                 // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_FIXED_FREQ,     // Higher priority (3) to run first
        &xSystemInfoTaskHandle        // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System information task created successfully");
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_ERROR, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create system information task");
    }

    // 9. Create the system monitoring task
    xResult = xTaskCreate(
        vSystemMonitorTask,           // Task function
        "SysMonitor",                 // Task name
        TASK_STACK_SIZE_FIXED_FREQ,   // Increased stack size for I2C scanning and string operations
        NULL,                         // Parameters
        TASK_PRIORITY_BACKGROUND,     // Priority from config.h
        &xSystemMonitorTaskHandle     // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System monitoring task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(xSystemMonitorTaskHandle, "SysMonitor", MONITOR_INTERVAL_MS + 1000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_FATAL, "app_main", NULL);
        ESP_LOGE(TAG, "ERROR: Failed to create system monitoring task!");
        abort();
    }

    // 10. Create the heartbeat task
    xResult = xTaskCreate(
        vHeartbeatTask,               // Task function
        "Heartbeat",                  // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_BACKGROUND,     // Priority from config.h
        &xHeartbeatTaskHandle         // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "Heartbeat task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(xHeartbeatTaskHandle, "Heartbeat", 6000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create heartbeat task");
    }

    // 11. Create the sensor acquisition task (critical priority)
    xResult = xTaskCreate(
        vSensorAcquisitionTask,         // Task function
        "SensorAcq",                     // Task name
        TASK_STACK_SIZE_CRITICAL,        // Stack size from config.h
        NULL,                            // Parameters
        TASK_PRIORITY_CRITICAL,          // Priority from config.h (highest)
        &xSensorAcquisitionTaskHandle    // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "Sensor acquisition task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(xSensorAcquisitionTaskHandle, "SensorAcq", SENSOR_ACQUISITION_INTERVAL + 200);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_FATAL, "app_main", NULL);
        ESP_LOGE(TAG, "ERROR: Failed to create sensor acquisition task!");
        abort();
    }

    // 12. Create the MQTT publisher task
    xResult = xTaskCreate(
        vMqttPublisherTask,              // Task function
        "MqttPub",                       // Task name
        TASK_STACK_SIZE_FIXED_FREQ,      // Stack size from config.h
        NULL,                            // Parameters
        TASK_PRIORITY_FIXED_FREQ,        // Priority from config.h
        &xMqttPublisherTaskHandle        // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "MQTT publisher task created successfully");
        // Register with watchdog after creation
        watchdog_register_task(xMqttPublisherTaskHandle, "MqttPub", MQTT_PUBLISH_INTERVAL + 1000);
    } else {
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
        ESP_LOGW(TAG, "WARNING: Failed to create MQTT publisher task");
        // Continue anyway - MQTT is not critical for basic operation
    }

    // FreeRTOS scheduler is already running in ESP-IDF
    // Tasks will start executing immediately
    ESP_LOGI(TAG, "All tasks created successfully. System is now running...");
}