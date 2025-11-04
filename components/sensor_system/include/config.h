/*
 * config.h - System Configuration Constants
 *
 * This file contains all system-wide configuration constants for the
 * ESP32 FreeRTOS sensor system. These values control timing, buffer
 * sizes, and other system parameters.
 */

#ifndef CONFIG_H
#define CONFIG_H

// System identification
#define FIRMWARE_VERSION_MAJOR    1
#define FIRMWARE_VERSION_MINOR    0
#define FIRMWARE_NAME             "SensorX ESP32"

// FreeRTOS task priorities (higher number = higher priority in ESP-IDF)
#define TASK_PRIORITY_CRITICAL    5    // Sensor acquisition, watchdog
#define TASK_PRIORITY_FIXED_FREQ  3    // MQTT publisher, data processor
#define TASK_PRIORITY_BACKGROUND  1    // SD logger, system monitor
#define TASK_PRIORITY_IDLE        0    // WiFi manager, OTA handler

// FreeRTOS task stack sizes (in bytes)
#define TASK_STACK_SIZE_CRITICAL     4096  // Sensor acquisition
#define TASK_STACK_SIZE_FIXED_FREQ   3072  // MQTT publisher
#define TASK_STACK_SIZE_BACKGROUND   2048  // SD logger, system monitor
#define TASK_STACK_SIZE_IDLE         2048  // WiFi manager

// System timing constants (in milliseconds)
#define SYSTEM_LOOP_INTERVAL         1000  // Main loop interval
#define SENSOR_ACQUISITION_INTERVAL  100   // Sensor reading interval
#define WATCHDOG_FEED_INTERVAL       100   // Watchdog feeding interval (reduced CPU load)
#define MQTT_PUBLISH_INTERVAL        1000  // MQTT publishing interval
#define SD_LOG_FLUSH_INTERVAL        2000  // SD card flush interval (2-5 sec range)
#define SYSTEM_STATUS_INTERVAL       30000 // System status reporting
#define MONITOR_INTERVAL_MS          5000  // System monitor task interval

// Buffer sizes
#define SERIAL_BUFFER_SIZE           256   // Serial input buffer
#define LOG_MESSAGE_BUFFER_SIZE      512   // Log message buffer
#define MQTT_BUFFER_SIZE             1024  // MQTT message buffer

// Sensor timing constants
#define FLOW_SENSOR_TIMEOUT          2000  // Flow sensor measurement timeout
#define TEMPERATURE_READ_TIMEOUT     1000  // Temperature sensor timeout
#define ADC_READ_TIMEOUT             500   // ADC conversion timeout

// I2C configuration
#define I2C_CLOCK_SPEED              100000 // 100kHz (I2C Standard-mode, optimal default)
#define I2C_TIMEOUT_MS               10    // 10ms timeout

// System limits
#define MAX_TEMPERATURE_SENSORS      4     // Maximum DS18B20 sensors
#define MAX_ADC_CHANNELS             8     // Maximum ADC channels
#define MAX_RETRY_ATTEMPTS           3     // Maximum retry attempts

// Debug configuration
#define DEBUG_MODE_DEFAULT           0     // Default debug level
#define DEBUG_SERIAL_TIMEOUT         5000  // Debug status interval

// Verbose and debug mode configuration
#define VERBOSE                      0     // Enable verbose output (0=off, 1=on) - DISABLED for stability
#define DEBUG                        0     // Debug level (0=off, 1-3=levels) - DISABLED for stability

// Memory management
#define HEAP_CHECK_INTERVAL          10000 // Heap monitoring interval
#define STACK_WATERMARK_CHECK        1000  // Stack watermark check interval

// WiFi configuration
#define WIFI_SSID                     "ATT9LCV8fL_2.4"      // Replace with your WiFi network name
#define WIFI_PASSWORD                 "6jhz7ai7pqy5"        // Replace with your WiFi password
#define WIFI_AUTO_CONNECT              1                    // Set to 1 to auto-connect on startup, 0 to connect manually
#define WIFI_HOSTNAME                  "SensorX-ESP32"      // Hostname for WiFi (max ~32 chars, no spaces/special chars)

// MQTT configuration
#define MQTT_PROD_SERVER_IP            "192.168.1.250"     // Production MQTT broker IP
#define MQTT_DEV_SERVER_IP              "192.168.1.249"     // Development MQTT broker IP
#define MQTT_PORT                      1883                 // MQTT broker port
#define MQTT_CLIENT_ID                 WIFI_HOSTNAME       // Use hostname as client ID
#define MQTT_TOPIC_BINARY               "sensor/binary"     // Base topic for binary messages
#define MQTT_TOPIC_JSON                 "sensor/json"       // Base topic for JSON messages
#define MQTT_AUTO_RECONNECT             1                   // Auto-reconnect on disconnect (1=enabled, 0=disabled)
#define MQTT_CONNECT_TIMEOUT_MS         5000                // Connection timeout in milliseconds

#endif /* CONFIG_H */
