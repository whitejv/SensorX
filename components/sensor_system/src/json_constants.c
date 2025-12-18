/*
 * json_constants.c - Static JSON Key Strings Implementation
 *
 * Defines all JSON key strings as static constants to reduce memory allocations.
 */

#include "json_constants.h"

// System and firmware keys
const char* JSON_KEY_FIRMWARE_VERSION = "firmware_version";
const char* JSON_KEY_BUILD_DATE = "build_date";
const char* JSON_KEY_BUILD_TIME = "build_time";
const char* JSON_KEY_UPTIME = "uptime";
const char* JSON_KEY_UPTIME_SEC = "uptime_sec";
const char* JSON_KEY_FREE_HEAP = "free_heap";
const char* JSON_KEY_MIN_FREE_HEAP = "min_free_heap";
const char* JSON_KEY_ACTIVE_TASKS = "active_tasks";
const char* JSON_KEY_IDLE_TIME_PERCENT = "idle_time_percent";
const char* JSON_KEY_TIMESTAMP = "timestamp";

// Panic statistics keys
const char* JSON_KEY_PANIC_COUNT = "panic_count";
const char* JSON_KEY_PANIC_COUNT_CLEAN = "panic_count_since_clean";
const char* JSON_KEY_LAST_RESET_REASON = "last_reset_reason";
const char* JSON_KEY_LAST_PANIC_TIMESTAMP_MS = "last_panic_timestamp_ms";

// Status strings
const char* JSON_STATUS_CONNECTED = "connected";
const char* JSON_STATUS_DISCONNECTED = "disconnected";
const char* JSON_STATUS_INITIALIZED = "initialized";
const char* JSON_STATUS_NO_SENSORS = "no_sensors";

// Flow sensor keys
const char* JSON_KEY_FLOW1_PULSES = "S005D:flow1_pulses";
const char* JSON_KEY_FLOW1_MS = "S005D:flow1_ms";
const char* JSON_KEY_FLOW1_NEWDATA = "S005D:flow1_newData";
const char* JSON_KEY_FLOW2_PULSES = "S005D:flow2_pulses";
const char* JSON_KEY_FLOW2_MS = "S005D:flow2_ms";
const char* JSON_KEY_FLOW2_NEWDATA = "S005D:flow2_newData";
const char* JSON_KEY_FLOW3_PULSES = "S005D:flow3_pulses";
const char* JSON_KEY_FLOW3_MS = "S005D:flow3_ms";
const char* JSON_KEY_FLOW3_NEWDATA = "S005D:flow3_newData";

// WiFi keys
const char* JSON_KEY_WIFI_IP = "ip";
const char* JSON_KEY_WIFI_RSSI = "rssi";
const char* JSON_KEY_WIFI_UPTIME_SEC = "uptime_sec";
const char* JSON_KEY_WIFI_STATUS = "status";

// MQTT keys
const char* JSON_KEY_MQTT_BROKER_IP = "broker_ip";
const char* JSON_KEY_MQTT_CONNECTED_TO_PROD = "connected_to_prod";
const char* JSON_KEY_MQTT_PUBLISH_SUCCESS = "publish_success";
const char* JSON_KEY_MQTT_PUBLISH_FAILURES = "publish_failures";
const char* JSON_KEY_MQTT_STATUS = "status";

// I2C keys
const char* JSON_KEY_I2C_DEVICE_COUNT = "device_count";
const char* JSON_KEY_I2C_DEVICES = "devices";
const char* JSON_KEY_I2C_STATUS = "status";
const char* JSON_KEY_I2C_ERRORS_TOTAL = "total";
const char* JSON_KEY_I2C_ERRORS_TIMEOUTS = "timeouts";
const char* JSON_KEY_I2C_ERRORS_NACKS = "nacks";
const char* JSON_KEY_I2C_ERRORS_BUS = "bus_errors";
const char* JSON_KEY_I2C_ERRORS_DEVICE_NOT_FOUND = "device_not_found";
const char* JSON_KEY_I2C_ERRORS_TX_FAILURES = "transaction_failures";
const char* JSON_KEY_I2C_ERRORS_LAST_TIME_MS = "last_error_time_ms";

// One-Wire keys
const char* JSON_KEY_ONEWIRE_SENSOR_COUNT = "sensor_count";
const char* JSON_KEY_ONEWIRE_SENSOR_COUNT_DATA = "sensor_count_from_data";
const char* JSON_KEY_ONEWIRE_STATUS = "status";

// Fan keys
const char* JSON_KEY_FAN_ON = "on";
const char* JSON_KEY_FAN_TEMP_F = "temp_f";
const char* JSON_KEY_FAN_TEMP_SOURCE = "temp_source";
const char* JSON_KEY_FAN_THRESHOLD_F = "threshold_f";

// Error recovery keys
const char* JSON_KEY_ERRORS_TOTAL = "total";
const char* JSON_KEY_ERRORS_RECOVERED = "recovered";
const char* JSON_KEY_ERRORS_CRITICAL = "critical";

// Watchdog keys
const char* JSON_KEY_WATCHDOG_TOTAL_FEEDS = "total_feeds";
const char* JSON_KEY_WATCHDOG_TIMEOUTS = "timeouts";
const char* JSON_KEY_WATCHDOG_TASKS_MONITORED = "tasks_monitored";

