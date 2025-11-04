/*
 * mqtt_manager.h - MQTT Manager Header
 *
 * This file defines the MQTT client management interface for the ESP32 sensor system.
 * Provides MQTT client initialization, connection management, and publishing functions.
 *
 * Design Principles:
 * - Dual server fallback (production → development)
 * - Auto-reconnect on disconnect
 * - Binary and JSON publishing support
 * - Integration with error recovery system
 * - Non-blocking connection management
 */

#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// MQTT connection status
typedef enum {
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTING,
    MQTT_STATUS_CONNECTED,
    MQTT_STATUS_RECONNECTING,
    MQTT_STATUS_ERROR
} MQTTStatus_t;

// MQTT statistics
typedef struct {
    uint32_t connectAttempts;      // Total connection attempts
    uint32_t successfulConnections; // Successful connections
    uint32_t disconnections;       // Disconnection events
    uint32_t reconnectAttempts;     // Reconnection attempts
    uint32_t publishSuccess;        // Successful publishes
    uint32_t publishFailures;       // Failed publishes
    uint32_t lastErrorCode;         // Last error code
    bool connectedToProd;           // Currently connected to production server
} MQTTStats_t;

/*
 * Initialize MQTT manager
 * Must be called before any other MQTT functions
 * Note: WiFi must be connected before calling this
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_manager_init(void);

/*
 * Connect to MQTT broker
 * Tries production server first, then development server
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_manager_connect(void);

/*
 * Disconnect from MQTT broker
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_manager_disconnect(void);

/*
 * Check if MQTT is connected
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_manager_is_connected(void);

/*
 * Get current MQTT connection status
 * 
 * @return MQTTStatus_t status
 */
MQTTStatus_t mqtt_manager_get_status(void);

/*
 * Publish binary payload to MQTT
 * 
 * @param topic: MQTT topic (if NULL, uses default binary topic)
 * @param payload: Binary data payload
 * @param payload_len: Length of payload in bytes
 * @param qos: Quality of Service (0, 1, or 2)
 * @param retain: Retain flag (true to retain message)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_manager_publish_binary(const char* topic, 
                                       const uint8_t* payload, 
                                       size_t payload_len,
                                       int qos,
                                       bool retain);

/*
 * Publish JSON payload to MQTT
 * 
 * @param topic: MQTT topic (if NULL, uses default JSON topic)
 * @param json_str: JSON string (cJSON formatted)
 * @param qos: Quality of Service (0, 1, or 2)
 * @param retain: Retain flag (true to retain message)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_manager_publish_json(const char* topic,
                                     const char* json_str,
                                     int qos,
                                     bool retain);

/*
 * Get MQTT statistics
 * 
 * @param stats: Pointer to structure to fill with statistics
 */
void mqtt_manager_get_stats(MQTTStats_t* stats);

/*
 * Get MQTT broker IP address
 * 
 * @param ip_str: Buffer to store IP address string
 * @param len: Length of buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_manager_get_broker_ip(char* ip_str, size_t len);

/*
 * Reset MQTT statistics
 */
void mqtt_manager_reset_stats(void);

#endif /* MQTT_MANAGER_H */

