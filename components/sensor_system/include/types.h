/*
 * types.h - Data Structures and Type Definitions
 *
 * This file contains all common data structures, enums, and type
 * definitions used throughout the ESP32 FreeRTOS sensor system.
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Forward declarations for data structures
typedef struct FlowData FlowData_t;
typedef struct SensorData SensorData_t;
typedef struct SystemStatus SystemStatus_t;

// Sensor types enumeration
typedef enum {
    SENSOR_TYPE_NONE = 0,
    SENSOR_TYPE_FLOW,
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_PRESSURE,
    SENSOR_TYPE_CURRENT,
    SENSOR_TYPE_HUMIDITY,
    SENSOR_TYPE_ENVIRONMENTAL,
    SENSOR_TYPE_GPIO,
    SENSOR_TYPE_ADC
} SensorType_t;

// Sensor priority levels
typedef enum {
    SENSOR_PRIORITY_LOW = 0,      // GPIO, status sensors
    SENSOR_PRIORITY_MEDIUM,       // Environmental sensors
    SENSOR_PRIORITY_HIGH,         // ADC/PWM critical sensors
    SENSOR_PRIORITY_CRITICAL      // Real-time critical sensors
} SensorPriority_t;

// System state enumeration
typedef enum {
    SYSTEM_STATE_INIT = 0,
    SYSTEM_STATE_STARTUP,
    SYSTEM_STATE_RUNNING,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_RECOVERY,
    SYSTEM_STATE_SHUTDOWN
} SystemState_t;

// Error codes
typedef enum {
    ERROR_NONE = 0,
    ERROR_I2C_TIMEOUT,
    ERROR_I2C_NACK,
    ERROR_SENSOR_NOT_FOUND,
    ERROR_BUFFER_OVERFLOW,
    ERROR_INVALID_DATA,
    ERROR_MEMORY_ALLOCATION,
    ERROR_TASK_CREATION,
    ERROR_QUEUE_FULL,
    ERROR_CHECKSUM
} ErrorCode_t;

// Flow sensor data structure (bit-packed for efficiency)
struct FlowData {
    uint32_t pulses : 12;       // 12 bits for pulse count (0-4095)
    uint32_t milliseconds : 19; // 19 bits for milliseconds (0-524287)
    uint32_t newData : 1;       // 1 bit for new data flag
};

// Basic sensor data structure
struct SensorData {
    SensorType_t type;          // Sensor type
    SensorPriority_t priority;  // Sensor priority level
    uint32_t timestamp;         // Timestamp (milliseconds)
    float value;                // Sensor reading value
    ErrorCode_t error;          // Error status
    bool valid;                 // Data validity flag
};

// System status structure
struct SystemStatus {
    SystemState_t state;        // Current system state
    uint32_t uptime;            // System uptime in seconds
    uint32_t freeHeap;          // Free heap memory in bytes
    uint32_t totalTasks;        // Number of active FreeRTOS tasks
    bool wifiConnected;         // WiFi connection status
    bool mqttConnected;         // MQTT connection status
    bool sdCardReady;           // SD card status
    ErrorCode_t lastError;      // Last system error
};

// System task status enumeration (avoiding FreeRTOS TaskStatus_t conflict)
typedef enum {
    SYS_TASK_STATUS_RUNNING = 0,
    SYS_TASK_STATUS_BLOCKED,
    SYS_TASK_STATUS_SUSPENDED,
    SYS_TASK_STATUS_DELETED,
    SYS_TASK_STATUS_ERROR
} SystemTaskStatus_t;

// Task information structure
typedef struct {
    const char *name;           // Task name
    SystemTaskStatus_t status;  // Task status
    uint32_t stackHighWaterMark; // Stack usage watermark
    uint32_t runtimeCounter;    // Task runtime counter
} TaskInfo_t;

// Buffer status structure
typedef struct {
    uint32_t size;              // Buffer size
    uint32_t used;              // Bytes currently used
    uint32_t overflowCount;     // Number of overflow events
    bool full;                  // Buffer full flag
} BufferStatus_t;

// I2C device status
typedef struct {
    uint8_t address;            // I2C address
    bool present;               // Device presence flag
    uint32_t lastSeen;          // Last successful communication
    ErrorCode_t lastError;      // Last communication error
} I2CDeviceStatus_t;

// System configuration structure
typedef struct {
    uint16_t firmwareVersion;   // Firmware version (major.minor)
    uint32_t systemStartTime;   // System start timestamp
    uint32_t configChecksum;    // Configuration checksum
} SystemConfig_t;

// Function pointer types for callbacks
typedef void (*SensorCallback_t)(SensorData_t *data);
typedef ErrorCode_t (*InitFunction_t)(void);
typedef void (*TaskFunction_t)(void *parameters);

// Message types for inter-task communication
typedef enum {
    MSG_TYPE_SENSOR_DATA = 0,
    MSG_TYPE_SYSTEM_STATUS,
    MSG_TYPE_ERROR_REPORT,
    MSG_TYPE_COMMAND,
    MSG_TYPE_LOG_MESSAGE
} MessageType_t;

// Generic message structure
typedef struct {
    MessageType_t type;         // Message type
    uint32_t timestamp;         // Message timestamp
    void *data;                 // Message data pointer
    uint16_t dataLength;        // Data length
} Message_t;

#endif /* TYPES_H */
