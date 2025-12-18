/*
 * json_constants.h - Static JSON Key Strings and Constants
 *
 * This file defines static string constants for all JSON keys used in
 * MQTT messages. Using static strings reduces memory allocations and
 * improves performance by avoiding repeated string literal allocations.
 */

#ifndef JSON_CONSTANTS_H
#define JSON_CONSTANTS_H

// System and firmware keys
extern const char* JSON_KEY_FIRMWARE_VERSION;
extern const char* JSON_KEY_BUILD_DATE;
extern const char* JSON_KEY_BUILD_TIME;
extern const char* JSON_KEY_UPTIME;
extern const char* JSON_KEY_UPTIME_SEC;
extern const char* JSON_KEY_FREE_HEAP;
extern const char* JSON_KEY_MIN_FREE_HEAP;
extern const char* JSON_KEY_ACTIVE_TASKS;
extern const char* JSON_KEY_IDLE_TIME_PERCENT;
extern const char* JSON_KEY_TIMESTAMP;

// Panic statistics keys
extern const char* JSON_KEY_PANIC_COUNT;
extern const char* JSON_KEY_PANIC_COUNT_CLEAN;
extern const char* JSON_KEY_LAST_RESET_REASON;
extern const char* JSON_KEY_LAST_PANIC_TIMESTAMP_MS;

// Status strings
extern const char* JSON_STATUS_CONNECTED;
extern const char* JSON_STATUS_DISCONNECTED;
extern const char* JSON_STATUS_INITIALIZED;
extern const char* JSON_STATUS_NO_SENSORS;

// Flow sensor keys
extern const char* JSON_KEY_FLOW1_PULSES;
extern const char* JSON_KEY_FLOW1_MS;
extern const char* JSON_KEY_FLOW1_NEWDATA;
extern const char* JSON_KEY_FLOW2_PULSES;
extern const char* JSON_KEY_FLOW2_MS;
extern const char* JSON_KEY_FLOW2_NEWDATA;
extern const char* JSON_KEY_FLOW3_PULSES;
extern const char* JSON_KEY_FLOW3_MS;
extern const char* JSON_KEY_FLOW3_NEWDATA;

// WiFi keys
extern const char* JSON_KEY_WIFI_IP;
extern const char* JSON_KEY_WIFI_RSSI;
extern const char* JSON_KEY_WIFI_UPTIME_SEC;
extern const char* JSON_KEY_WIFI_STATUS;

// MQTT keys
extern const char* JSON_KEY_MQTT_BROKER_IP;
extern const char* JSON_KEY_MQTT_CONNECTED_TO_PROD;
extern const char* JSON_KEY_MQTT_PUBLISH_SUCCESS;
extern const char* JSON_KEY_MQTT_PUBLISH_FAILURES;
extern const char* JSON_KEY_MQTT_STATUS;

// I2C keys
extern const char* JSON_KEY_I2C_DEVICE_COUNT;
extern const char* JSON_KEY_I2C_DEVICES;
extern const char* JSON_KEY_I2C_STATUS;
extern const char* JSON_KEY_I2C_ERRORS_TOTAL;
extern const char* JSON_KEY_I2C_ERRORS_TIMEOUTS;
extern const char* JSON_KEY_I2C_ERRORS_NACKS;
extern const char* JSON_KEY_I2C_ERRORS_BUS;
extern const char* JSON_KEY_I2C_ERRORS_DEVICE_NOT_FOUND;
extern const char* JSON_KEY_I2C_ERRORS_TX_FAILURES;
extern const char* JSON_KEY_I2C_ERRORS_LAST_TIME_MS;

// One-Wire keys
extern const char* JSON_KEY_ONEWIRE_SENSOR_COUNT;
extern const char* JSON_KEY_ONEWIRE_SENSOR_COUNT_DATA;
extern const char* JSON_KEY_ONEWIRE_STATUS;

// Fan keys
extern const char* JSON_KEY_FAN_ON;
extern const char* JSON_KEY_FAN_TEMP_F;
extern const char* JSON_KEY_FAN_TEMP_SOURCE;
extern const char* JSON_KEY_FAN_THRESHOLD_F;

// Error recovery keys
extern const char* JSON_KEY_ERRORS_TOTAL;
extern const char* JSON_KEY_ERRORS_RECOVERED;
extern const char* JSON_KEY_ERRORS_CRITICAL;

// Watchdog keys
extern const char* JSON_KEY_WATCHDOG_TOTAL_FEEDS;
extern const char* JSON_KEY_WATCHDOG_TIMEOUTS;
extern const char* JSON_KEY_WATCHDOG_TASKS_MONITORED;

#endif /* JSON_CONSTANTS_H */

