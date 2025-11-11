/*
 * sensor_coordination.h - Sensor Coordination System Header
 * 
 * This header defines the sensor coordination system using FreeRTOS Event Groups
 * to synchronize sensor manager completion with MQTT publishing.
 * 
 * The coordination system ensures that all 1000ms sensors complete their reads
 * before the MQTT publisher publishes data, ensuring data consistency.
 */

#ifndef SENSOR_COORDINATION_H
#define SENSOR_COORDINATION_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Event Group Bit Definitions
// ============================================================================

/**
 * Sensor Completion Event Bits
 * 
 * Each sensor manager sets its completion bit after updating genericSens_.
 * The MQTT publisher waits for all 1000ms sensors to complete before publishing.
 */
#define SENSOR_EVENT_FLOW_COMPLETE       BIT0  // PCNT flow sensors (1000ms)
#define SENSOR_EVENT_ADC_COMPLETE        BIT1  // I2C ADC sensors (1000ms)
#define SENSOR_EVENT_GPIO_I2C_COMPLETE  BIT2  // I2C GPIO expander (1000ms)
// SENSOR_EVENT_GPIO_DISC_COMPLETE removed - GPIO discrete manager deprecated (no longer needed)
#define SENSOR_EVENT_ENV_COMPLETE       BIT4  // Environmental sensors (5000ms) - optional
#define SENSOR_EVENT_TEMP_COMPLETE      BIT5  // Temperature sensors (5000ms) - optional

/**
 * Event Group Mask for 1000ms Sensors
 * 
 * All sensors that must complete before MQTT publisher publishes at 1000ms intervals.
 * The publisher waits for ALL bits in this mask to be set.
 */
#define SENSOR_EVENT_1000MS_MASK        (SENSOR_EVENT_FLOW_COMPLETE | \
                                         SENSOR_EVENT_ADC_COMPLETE | \
                                         SENSOR_EVENT_GPIO_I2C_COMPLETE)

/**
 * Event Group Mask for 5000ms Sensors (Optional)
 * 
 * These sensors read at 5000ms intervals and don't participate in the
 * 1000ms coordination, but their completion can be tracked if needed.
 */
#define SENSOR_EVENT_5000MS_MASK        (SENSOR_EVENT_ENV_COMPLETE | \
                                         SENSOR_EVENT_TEMP_COMPLETE)

// ============================================================================
// External Declarations
// ============================================================================

/**
 * Sensor Event Group
 * 
 * Event group used for coordinating sensor completion with MQTT publishing.
 * Created in sensor_coordination.c, initialized in main.c.
 */
extern EventGroupHandle_t sensor_event_group;

/**
 * Sensor Data Mutex
 * 
 * Mutex used to protect concurrent access to genericSens_ structure.
 * Created in sensor_coordination.c, initialized in main.c.
 */
extern SemaphoreHandle_t sensor_data_mutex;

// ============================================================================
// Sensor Data Structure
// ============================================================================

/**
 * Sensor Data Wrapper Structure
 * 
 * Contains the genericSens_ union and mutex for thread-safe access.
 * This structure is initialized in sensor_coordination.c.
 */
typedef struct {
    SemaphoreHandle_t mutex;  // Mutex for thread-safe access to genericSens_
    // Note: genericSens_ is declared in sensor.h and defined in sensor_coordination.c
} SensorDataWrapper_t;

/**
 * Global Sensor Data Wrapper
 * 
 * Provides access to sensor data structure and synchronization primitives.
 */
extern SensorDataWrapper_t sensor_data;

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * Initialize Sensor Coordination System
 * 
 * Creates the event group and mutex for sensor coordination.
 * Must be called before any sensor managers start.
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t sensor_coordination_init(void);

/**
 * Get Sensor Event Group Handle
 * 
 * @return Event group handle, or NULL if not initialized
 */
EventGroupHandle_t sensor_coordination_get_event_group(void);

/**
 * Get Sensor Data Mutex Handle
 * 
 * @return Mutex handle, or NULL if not initialized
 */
SemaphoreHandle_t sensor_coordination_get_mutex(void);

/**
 * Signal Sensor Completion
 * 
 * Convenience function to signal sensor completion.
 * Used by sensor managers after updating genericSens_.
 * 
 * @param event_bits Event bits to set (e.g., SENSOR_EVENT_FLOW_COMPLETE)
 */
void sensor_coordination_signal_completion(EventBits_t event_bits);

/**
 * Wait for Sensor Completion
 * 
 * Convenience function to wait for sensor completion.
 * Used by MQTT publisher to wait for all 1000ms sensors.
 * 
 * @param event_mask Bits to wait for (e.g., SENSOR_EVENT_1000MS_MASK)
 * @param clear_on_exit Clear bits after reading if true
 * @param wait_for_all Wait for all bits if true, any bit if false
 * @param timeout_ticks Timeout in ticks
 * @return Event bits that were set, or 0 on timeout
 */
EventBits_t sensor_coordination_wait_for_completion(
    EventBits_t event_mask,
    bool clear_on_exit,
    bool wait_for_all,
    TickType_t timeout_ticks
);

/**
 * Clear Sensor Event Bits
 * 
 * Clear specified event bits in the event group.
 * 
 * @param event_bits Bits to clear
 */
void sensor_coordination_clear_bits(EventBits_t event_bits);

/**
 * Get Current Event Bits
 * 
 * Get current event bits without waiting.
 * 
 * @return Current event bits
 */
EventBits_t sensor_coordination_get_bits(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_COORDINATION_H */

