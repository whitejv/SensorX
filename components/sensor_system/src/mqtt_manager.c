/*
 * mqtt_manager.c - MQTT Manager Implementation
 *
 * This file implements the MQTT client management system for the ESP32 sensor system.
 * Provides MQTT client initialization, connection management with dual server fallback,
 * and publishing functions for binary and JSON data.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mqtt_client.h>
#include <esp_netif.h>
#include <string.h>
#include <stdbool.h>
#include <lwip/inet.h>

#include "mqtt_manager.h"
#include "config.h"
#include "wifi_manager.h"
#include "error_recovery.h"

static const char *TAG = "MQTT_MANAGER";

// Static variables
static bool mqtt_manager_initialized = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static MQTTStatus_t current_status = MQTT_STATUS_DISCONNECTED;
static MQTTStats_t stats = {0};
static bool connected_to_prod = false;
static char current_broker_ip[16] = "";  // Store current broker IP

// Forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data);

esp_err_t mqtt_manager_init(void) {
    if (mqtt_manager_initialized) {
        ESP_LOGW(TAG, "MQTT manager already initialized");
        return ESP_OK;
    }

    // Check WiFi connection
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected, MQTT initialization deferred");
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize statistics
    memset(&stats, 0, sizeof(MQTTStats_t));
    current_status = MQTT_STATUS_DISCONNECTED;
    connected_to_prod = false;

    mqtt_manager_initialized = true;
    // MQTT manager initialized - no log (only report errors)
    return ESP_OK;
}

// Forward declaration
static esp_err_t mqtt_manager_connect_to_server(const char* server_ip, bool is_prod);

esp_err_t mqtt_manager_connect(void) {
    if (!mqtt_manager_initialized) {
        ESP_LOGE(TAG, "MQTT manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!wifi_manager_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected, cannot connect to MQTT");
        return ESP_ERR_INVALID_STATE;
    }

    // Clean up existing client if any
    if (mqtt_client != NULL) {
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }

    // Try production server first
    esp_err_t ret = mqtt_manager_connect_to_server(MQTT_PROD_SERVER_IP, true);
    if (ret == ESP_OK) {
        return ESP_OK;
    }

    // Try development server if production failed
    ESP_LOGW(TAG, "Production server connection failed, trying development server");
    ret = mqtt_manager_connect_to_server(MQTT_DEV_SERVER_IP, false);
    if (ret == ESP_OK) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to connect to any MQTT server");
    error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "mqtt_manager_connect", NULL);
    current_status = MQTT_STATUS_ERROR;
    return ret;
}

static esp_err_t mqtt_manager_connect_to_server(const char* server_ip, bool is_prod) {
    esp_err_t ret;

    // MQTT broker URI
    char broker_uri[64];
    snprintf(broker_uri, sizeof(broker_uri), "mqtt://%s:%d", server_ip, MQTT_PORT);

    // MQTT client configuration
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials.client_id = MQTT_CLIENT_ID,
        .session.keepalive = 60,
        .session.disable_clean_session = false,
    };

    // Create MQTT client
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_ERR_NO_MEM;
    }

    // Register event handler
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                    mqtt_event_handler, NULL);

    // Start MQTT client
    ret = esp_mqtt_client_start(mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return ret;
    }

    current_status = MQTT_STATUS_CONNECTING;
    stats.connectAttempts++;
    connected_to_prod = is_prod;
    strncpy(current_broker_ip, server_ip, sizeof(current_broker_ip) - 1);
    current_broker_ip[sizeof(current_broker_ip) - 1] = '\0';

    // Connecting to MQTT server - no log (only report errors)

    // Wait for connection (with timeout)
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (current_status == MQTT_STATUS_CONNECTING && 
           mqtt_client != NULL) {
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) > MQTT_CONNECT_TIMEOUT_MS) {
            ESP_LOGE(TAG, "MQTT connection timeout");
            esp_mqtt_client_destroy(mqtt_client);
            mqtt_client = NULL;
            current_status = MQTT_STATUS_ERROR;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (current_status == MQTT_STATUS_CONNECTED) {
        stats.successfulConnections++;
        // MQTT connected - no log (only report errors)
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t mqtt_manager_disconnect(void) {
    if (!mqtt_manager_initialized || mqtt_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_mqtt_client_stop(mqtt_client);
    if (ret == ESP_OK) {
        stats.disconnections++;
        current_status = MQTT_STATUS_DISCONNECTED;
        // MQTT disconnected - no log (only report errors)
    }
    return ret;
}

bool mqtt_manager_is_connected(void) {
    return (current_status == MQTT_STATUS_CONNECTED);
}

MQTTStatus_t mqtt_manager_get_status(void) {
    return current_status;
}

esp_err_t mqtt_manager_publish_binary(const char* topic,
                                       const uint8_t* payload,
                                       size_t payload_len,
                                       int qos,
                                       bool retain) {
    if (!mqtt_manager_is_connected() || mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT not connected, cannot publish");
        return ESP_ERR_INVALID_STATE;
    }

    const char* publish_topic = (topic != NULL) ? topic : MQTT_TOPIC_BINARY;

    int msg_id = esp_mqtt_client_publish(mqtt_client, publish_topic,
                                          (const char*)payload, payload_len,
                                          qos, retain ? 1 : 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish binary message: topic=%s, len=%zu, qos=%d", 
                 publish_topic, payload_len, qos);
        stats.publishFailures++;
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "mqtt_manager_publish_binary", NULL);
        return ESP_FAIL;
    }
    // Binary message published successfully - no log (only report errors)
    stats.publishSuccess++;
    return ESP_OK;
}

esp_err_t mqtt_manager_publish_json(const char* topic,
                                     const char* json_str,
                                     int qos,
                                     bool retain) {
    if (!mqtt_manager_is_connected() || mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT not connected, cannot publish");
        return ESP_ERR_INVALID_STATE;
    }

    const char* publish_topic = (topic != NULL) ? topic : MQTT_TOPIC_JSON;

    int msg_id = esp_mqtt_client_publish(mqtt_client, publish_topic,
                                          json_str, strlen(json_str),
                                          qos, retain ? 1 : 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish JSON message");
        stats.publishFailures++;
        error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "mqtt_manager_publish_json", NULL);
        return ESP_FAIL;
    }

    stats.publishSuccess++;
    return ESP_OK;
}

void mqtt_manager_get_stats(MQTTStats_t* stats_out) {
    if (stats_out != NULL) {
        *stats_out = stats;
        stats_out->connectedToProd = connected_to_prod;
    }
}

void mqtt_manager_reset_stats(void) {
    memset(&stats, 0, sizeof(MQTTStats_t));
    stats.connectedToProd = connected_to_prod;
}

esp_err_t mqtt_manager_get_broker_ip(char* ip_str, size_t len) {
    if (ip_str == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current_status != MQTT_STATUS_CONNECTED || strlen(current_broker_ip) == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    strncpy(ip_str, current_broker_ip, len - 1);
    ip_str[len - 1] = '\0';
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    (void)handler_args;  // Unused parameter
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            // MQTT connected - no log (only report errors)
            current_status = MQTT_STATUS_CONNECTED;
            stats.successfulConnections++;
            break;

        case MQTT_EVENT_DISCONNECTED:
            // MQTT disconnected - no log (only report errors)
            current_status = MQTT_STATUS_DISCONNECTED;
            stats.disconnections++;
            
            // Auto-reconnect if enabled
            if (MQTT_AUTO_RECONNECT && mqtt_manager_initialized) {
                current_status = MQTT_STATUS_RECONNECTING;
                stats.reconnectAttempts++;
                // Attempting MQTT reconnection - no log (only report errors)
                // Reconnection will be handled by ESP-IDF MQTT client
            }
            break;

        case MQTT_EVENT_SUBSCRIBED:
            // MQTT subscribed - no log (only report errors)
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            // MQTT unsubscribed - no log (only report errors)
            break;

        case MQTT_EVENT_PUBLISHED:
            // MQTT published - no log (only report errors)
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT_EVENT_DATA");
            ESP_LOGD(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGD(TAG, "DATA=%.*s", event->data_len, event->data);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Transport error: %s", esp_err_to_name(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Connection refused");
            }
            current_status = MQTT_STATUS_ERROR;
            error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "mqtt_event_handler", NULL);
            break;

        default:
            ESP_LOGD(TAG, "Other MQTT event id: %d", event->event_id);
            break;
    }
}

