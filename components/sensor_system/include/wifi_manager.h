/*
 * wifi_manager.h - WiFi Manager Header
 *
 * This file defines the WiFi management interface for the ESP32 sensor system.
 * Provides WiFi station mode connection management, automatic reconnection,
 * and status monitoring.
 *
 * Design Principles:
 * - Non-blocking WiFi connection management
 * - Automatic reconnection with exponential backoff
 * - Integration with error recovery system
 * - Connection status monitoring
 * - Event-driven architecture
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// WiFi configuration constants
#define WIFI_MAX_SSID_LEN         32
#define WIFI_MAX_PASSWORD_LEN      64
#define WIFI_MAX_RETRY_COUNT      5
#define WIFI_CONNECT_TIMEOUT_MS    10000  // 10 seconds
#define WIFI_RECONNECT_DELAY_MS   5000   // 5 seconds initial delay
#define WIFI_MAX_RECONNECT_DELAY_MS 60000  // 60 seconds max delay

// WiFi connection status
typedef enum {
    WIFI_STATUS_DISCONNECTED = 0,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_RECONNECTING,
    WIFI_STATUS_ERROR
} WiFiStatus_t;

// WiFi statistics
typedef struct {
    uint32_t connectAttempts;      // Total connection attempts
    uint32_t successfulConnections; // Successful connections
    uint32_t disconnections;       // Disconnection events
    uint32_t reconnectAttempts;     // Reconnection attempts
    uint32_t lastErrorCode;         // Last error code
    uint32_t uptime;                // Connection uptime in seconds
    int8_t rssi;                    // Current RSSI (signal strength)
} WiFiStats_t;

// WiFi configuration structure
typedef struct {
    char ssid[WIFI_MAX_SSID_LEN];
    char password[WIFI_MAX_PASSWORD_LEN];
    uint32_t timeout_ms;
    uint32_t max_retry;
} WiFiConfig_t;

/*
 * Initialize WiFi manager
 * Must be called before any other WiFi functions
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_init(void);

/*
 * Start WiFi connection task
 * This task manages WiFi connection and reconnection
 * 
 * @return pdPASS on success, pdFAIL on failure
 */
BaseType_t wifi_manager_start_task(void);

/*
 * Connect to WiFi network
 * 
 * @param ssid: WiFi SSID (network name)
 * @param password: WiFi password (can be NULL for open networks)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_connect(const char* ssid, const char* password);

/*
 * Disconnect from WiFi network
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_disconnect(void);

/*
 * Get current WiFi connection status
 * 
 * @return WiFiStatus_t status
 */
WiFiStatus_t wifi_manager_get_status(void);

/*
 * Check if WiFi is connected
 * 
 * @return true if connected, false otherwise
 */
bool wifi_manager_is_connected(void);

/*
 * Get WiFi statistics
 * 
 * @param stats: Pointer to structure to fill with statistics
 */
void wifi_manager_get_stats(WiFiStats_t* stats);

/*
 * Reset WiFi statistics
 */
void wifi_manager_reset_stats(void);

/*
 * Get current RSSI (signal strength)
 * 
 * @return RSSI value in dBm, or 0 if not connected
 */
int8_t wifi_manager_get_rssi(void);

/*
 * Get IP address as string
 * 
 * @param ip_str: Buffer to store IP address string (must be at least 16 bytes)
 * @param len: Length of buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_get_ip_address(char* ip_str, size_t len);

// WiFi manager task function (internal use)
void vWiFiManagerTask(void *pvParameters);

#endif /* WIFI_MANAGER_H */

