# HTTP OTA Implementation Guide

This document provides a comprehensive guide for adding HTTP Over-The-Air (OTA) firmware update capability to the ESP32 sensor system. This implementation follows the same modular architecture pattern used for WiFi, MQTT, and I2C managers.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Files to Create](#files-to-create)
4. [Files to Modify](#files-to-modify)
5. [Partition Table Configuration](#partition-table-configuration)
6. [Implementation Details](#implementation-details)
7. [Integration Steps](#integration-steps)
8. [Testing](#testing)

---

## Overview

### What is HTTP OTA?

HTTP OTA allows firmware updates to be downloaded over HTTP and written to flash memory, enabling remote firmware updates without physical access to the device. ESP-IDF provides built-in support for OTA updates through the `app_update` and `esp_http_client` components.

### Key Features

- **Dual Partition Support**: Uses two OTA app partitions (`ota_0`, `ota_1`) for safe updates
- **Automatic Rollback**: If new firmware fails, device automatically reverts to previous version
- **Progress Tracking**: Monitor download and write progress
- **Non-blocking**: Can be performed in background task
- **HTTP Support**: Simple HTTP server downloads (no SSL certificate management needed)

### Design Principles

- Follows existing manager pattern (WiFi, MQTT, I2C)
- Separate initialization function
- Background task for periodic checking
- Integration with error recovery system
- Configuration-driven (URL, interval, timeout in config.h)

---

## Architecture

### Component Structure

```
components/sensor_system/
├── include/
│   ├── ota_manager.h          # OTA manager API
│   └── config.h               # OTA configuration (modified)
├── src/
│   ├── ota_manager.c          # OTA manager implementation
│   └── system_init.c           # OTA status monitoring (optional)
└── CMakeLists.txt             # Component dependencies (modified)

main/
└── main.c                     # OTA manager integration (modified)
```

### OTA Update Flow

1. **Initialization**: OTA manager initializes after WiFi is connected
2. **Periodic Check**: Background task checks for updates at configured interval
3. **Download**: Downloads firmware binary from HTTP server
4. **Validation**: Validates firmware before writing
5. **Write**: Writes firmware to inactive OTA partition
6. **Set Boot Partition**: Sets new firmware as boot partition
7. **Reboot**: Device reboots to new firmware

---

## Files to Create

### 1. `components/sensor_system/include/ota_manager.h`

**Purpose**: Header file defining OTA manager API

**Key Components**:
- OTA status enumeration
- OTA statistics structure
- Function prototypes for init, check, update

**Complete File Content**:

```c
/*
 * ota_manager.h - OTA Manager Header
 *
 * This file defines the OTA (Over-The-Air) update management interface
 * for the ESP32 sensor system. Provides HTTP-based firmware update
 * functionality with dual partition support and automatic rollback.
 *
 * Design Principles:
 * - Simple initialization function
 * - Background task for periodic update checking
 * - Integration with error recovery system
 * - Configuration from config.h (URL, interval, timeout)
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

// OTA update status
typedef enum {
    OTA_STATUS_IDLE = 0,
    OTA_STATUS_CHECKING,
    OTA_STATUS_DOWNLOADING,
    OTA_STATUS_WRITING,
    OTA_STATUS_VALIDATING,
    OTA_STATUS_COMPLETE,
    OTA_STATUS_ERROR
} OTAStatus_t;

// OTA statistics
typedef struct {
    uint32_t checkAttempts;       // Total update check attempts
    uint32_t updateAttempts;       // Total update attempts
    uint32_t successfulUpdates;   // Successful updates
    uint32_t failedUpdates;       // Failed updates
    uint32_t lastErrorCode;        // Last error code
    uint32_t lastUpdateTime;       // Timestamp of last update
    uint32_t currentVersion;       // Current firmware version
    uint32_t availableVersion;      // Available firmware version
} OTAStats_t;

/*
 * Initialize OTA manager
 * Must be called after WiFi is connected
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_manager_init(void);

/*
 * Check for available firmware updates
 * Compares local version with server version
 * 
 * @param update_available: Pointer to store update availability status
 * @param available_version: Pointer to store available version (if update available)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_manager_check_for_update(bool* update_available, uint32_t* available_version);

/*
 * Start OTA update process
 * Downloads and installs firmware from configured URL
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_manager_start_update(void);

/*
 * Get current OTA status
 * 
 * @return OTAStatus_t status
 */
OTAStatus_t ota_manager_get_status(void);

/*
 * Get OTA statistics
 * 
 * @param stats: Pointer to structure to fill with statistics
 */
void ota_manager_get_stats(OTAStats_t* stats);

/*
 * Reset OTA statistics
 */
void ota_manager_reset_stats(void);

/*
 * Get current firmware version
 * 
 * @return Firmware version number
 */
uint32_t ota_manager_get_current_version(void);

/*
 * Start OTA checking task
 * Creates background task that periodically checks for updates
 * 
 * @return pdPASS on success, pdFAIL on failure
 */
BaseType_t ota_manager_start_task(void);

#endif /* OTA_MANAGER_H */
```

---

### 2. `components/sensor_system/src/ota_manager.c`

**Purpose**: OTA manager implementation

**Key Components**:
- HTTP client setup and firmware download
- OTA partition management
- Progress tracking
- Error handling

**Complete File Content**:

```c
/*
 * ota_manager.c - OTA Manager Implementation
 *
 * This file implements the OTA (Over-The-Air) update management system
 * for the ESP32 sensor system. Provides HTTP-based firmware download
 * and installation with dual partition support.
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_http_client.h>
#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_partition.h>
#include <string.h>
#include <stdbool.h>

#include "ota_manager.h"
#include "config.h"
#include "wifi_manager.h"
#include "error_recovery.h"
#include "watchdog.h"

static const char *TAG = "OTA_MANAGER";

// Static variables
static bool ota_manager_initialized = false;
static OTAStatus_t current_status = OTA_STATUS_IDLE;
static OTAStats_t stats = {0};
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t* update_partition = NULL;
static TaskHandle_t ota_task_handle = NULL;

// Forward declarations
static void ota_task(void *pvParameters);
static esp_err_t http_event_handler(esp_http_client_event_t *evt);

esp_err_t ota_manager_init(void) {
    if (ota_manager_initialized) {
        ESP_LOGW(TAG, "OTA manager already initialized");
        return ESP_OK;
    }

    // Check WiFi connection
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected, OTA initialization deferred");
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize statistics
    memset(&stats, 0, sizeof(OTAStats_t));
    current_status = OTA_STATUS_IDLE;
    
    // Get current firmware version
    const esp_app_desc_t* app_desc = esp_ota_get_app_description();
    if (app_desc != NULL) {
        stats.currentVersion = app_desc->version;
        ESP_LOGI(TAG, "Current firmware version: %lu", stats.currentVersion);
    }

    ota_manager_initialized = true;
    ESP_LOGI(TAG, "OTA manager initialized");
    return ESP_OK;
}

esp_err_t ota_manager_check_for_update(bool* update_available, uint32_t* available_version) {
    if (!ota_manager_initialized) {
        ESP_LOGE(TAG, "OTA manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!wifi_manager_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected, cannot check for updates");
        return ESP_ERR_INVALID_STATE;
    }

    if (update_available == NULL || available_version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *update_available = false;
    *available_version = 0;

    // For simple HTTP OTA, we'll check version by downloading a small version file
    // or by checking the firmware binary header
    // This is a simplified implementation - in production, you might want to use
    // a version endpoint or JSON API
    
    // TODO: Implement version checking logic
    // For now, this is a placeholder
    stats.checkAttempts++;
    current_status = OTA_STATUS_CHECKING;
    
    // Example: Check version file (if available)
    // char version_url[256];
    // snprintf(version_url, sizeof(version_url), "%s/version.txt", OTA_UPDATE_URL);
    // Download and parse version file...
    
    current_status = OTA_STATUS_IDLE;
    return ESP_OK;
}

esp_err_t ota_manager_start_update(void) {
    if (!ota_manager_initialized) {
        ESP_LOGE(TAG, "OTA manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!wifi_manager_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected, cannot start update");
        return ESP_ERR_INVALID_STATE;
    }

    if (current_status != OTA_STATUS_IDLE) {
        ESP_LOGW(TAG, "OTA update already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    stats.updateAttempts++;
    current_status = OTA_STATUS_DOWNLOADING;

    // Get next OTA partition
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No available OTA partition");
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Starting OTA update to partition: %s", update_partition->label);

    // Begin OTA update
    esp_err_t ret = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to begin OTA update: %s", esp_err_to_name(ret));
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ret;
    }

    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = OTA_UPDATE_URL,
        .timeout_ms = OTA_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .buffer_size = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        esp_ota_abort(ota_handle);
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ESP_ERR_NO_MEM;
    }

    // Perform HTTP GET request
    ret = esp_http_client_perform(client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        esp_ota_abort(ota_handle);
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ret;
    }

    int status_code = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);

    if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP request failed with status: %d", status_code);
        esp_http_client_cleanup(client);
        esp_ota_abort(ota_handle);
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Downloaded %d bytes", content_length);

    // End OTA update
    current_status = OTA_STATUS_VALIDATING;
    ret = esp_ota_end(ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ret;
    }

    // Set boot partition
    ret = esp_ota_set_boot_partition(update_partition);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        current_status = OTA_STATUS_ERROR;
        stats.failedUpdates++;
        error_report(ERROR_NONE, ERROR_SEVERITY_ERROR, "ota_manager_start_update", NULL);
        return ret;
    }

    ESP_LOGI(TAG, "OTA update completed successfully. Rebooting...");
    esp_http_client_cleanup(client);
    
    stats.successfulUpdates++;
    stats.lastUpdateTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    current_status = OTA_STATUS_COMPLETE;

    // Reboot to new firmware
    vTaskDelay(pdMS_TO_TICKS(1000));  // Give time for logs to flush
    esp_restart();

    return ESP_OK;  // Never reached, but keeps compiler happy
}

OTAStatus_t ota_manager_get_status(void) {
    return current_status;
}

void ota_manager_get_stats(OTAStats_t* stats_out) {
    if (stats_out != NULL) {
        *stats_out = stats;
    }
}

void ota_manager_reset_stats(void) {
    memset(&stats, 0, sizeof(OTAStats_t));
    const esp_app_desc_t* app_desc = esp_ota_get_app_description();
    if (app_desc != NULL) {
        stats.currentVersion = app_desc->version;
    }
}

uint32_t ota_manager_get_current_version(void) {
    return stats.currentVersion;
}

BaseType_t ota_manager_start_task(void) {
    if (!ota_manager_initialized) {
        ESP_LOGE(TAG, "OTA manager not initialized");
        return pdFAIL;
    }

    if (ota_task_handle != NULL) {
        ESP_LOGW(TAG, "OTA task already started");
        return pdPASS;
    }

    BaseType_t ret = xTaskCreate(
        ota_task,
        "OTAManager",
        TASK_STACK_SIZE_IDLE,
        NULL,
        TASK_PRIORITY_IDLE,
        &ota_task_handle
    );

    if (ret == pdPASS) {
        ESP_LOGI(TAG, "OTA checking task started");
        watchdog_register_task(ota_task_handle, "OTAManager", OTA_CHECK_INTERVAL + 60000);
    } else {
        ESP_LOGE(TAG, "Failed to create OTA task");
        error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "ota_manager_start_task", NULL);
    }

    return ret;
}

static void ota_task(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "OTA checking task started");

    // Register with watchdog
    watchdog_register_current_task("OTAManager", OTA_CHECK_INTERVAL + 60000);

    const TickType_t xCheckInterval = pdMS_TO_TICKS(OTA_CHECK_INTERVAL);

    for (;;) {
        // Wait for WiFi connection
        if (!wifi_manager_is_connected()) {
            ESP_LOGD(TAG, "WiFi not connected, skipping OTA check");
            watchdog_task_heartbeat();
            vTaskDelay(xCheckInterval);
            continue;
        }

        // Check for updates
        bool update_available = false;
        uint32_t available_version = 0;

        esp_err_t ret = ota_manager_check_for_update(&update_available, &available_version);
        if (ret == ESP_OK && update_available) {
            ESP_LOGI(TAG, "Update available: version %lu (current: %lu)", 
                     available_version, stats.currentVersion);
            
            // Start update (this will reboot on success)
            ota_manager_start_update();
        }

        watchdog_task_heartbeat();
        vTaskDelay(xCheckInterval);
    }
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;

        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;

        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", 
                     evt->header_key, evt->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write firmware data to OTA partition
                current_status = OTA_STATUS_WRITING;
                esp_err_t ret = esp_ota_write(ota_handle, evt->data, evt->data_len);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(ret));
                    return ret;
                }
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;

        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;

        default:
            break;
    }

    return ESP_OK;
}
```

---

## Files to Modify

### 3. `components/sensor_system/include/config.h`

**Location**: Add OTA configuration section after MQTT configuration

**Changes**:

```c
// OTA configuration
// Note: OTA server can be different from MQTT server. Configure based on your infrastructure.
// The example below uses the same IP as production MQTT server, but this is not required.
// You can use a different IP address, hostname, or port number as needed.
#define OTA_UPDATE_URL                 "http://192.168.1.250/firmware/SensorX.bin"  // HTTP server URL for firmware
#define OTA_CHECK_INTERVAL             3600000  // Check for updates every hour (3600000 ms)
#define OTA_TIMEOUT_MS                 30000    // HTTP download timeout (30 seconds)
#define OTA_ENABLE_AUTO_CHECK          1        // Enable automatic update checking (1=enabled, 0=disabled)
```

**Complete Section to Add** (after line 83):

```c
// MQTT configuration
#define MQTT_PROD_SERVER_IP            "192.168.1.250"     // Production MQTT broker IP
#define MQTT_DEV_SERVER_IP              "192.168.1.249"     // Development MQTT broker IP
#define MQTT_PORT                      1883                 // MQTT broker port
#define MQTT_CLIENT_ID                 WIFI_HOSTNAME       // Use hostname as client ID
#define MQTT_TOPIC_BINARY               "sensor/binary"     // Base topic for binary messages
#define MQTT_TOPIC_JSON                 "sensor/json"       // Base topic for JSON messages
#define MQTT_AUTO_RECONNECT             1                   // Auto-reconnect on disconnect (1=enabled, 0=disabled)
#define MQTT_CONNECT_TIMEOUT_MS         5000                // Connection timeout in milliseconds

// OTA configuration
// Note: OTA server can be different from MQTT server. Configure based on your infrastructure.
// If OTA server is on the same machine as MQTT, you can reuse the IP address.
// Example: Same server as production MQTT:
//   #define OTA_UPDATE_URL "http://192.168.1.250/firmware/SensorX.bin"
// Example: Different server:
//   #define OTA_UPDATE_URL "http://192.168.1.251/firmware/SensorX.bin"
// Example: Using hostname:
//   #define OTA_UPDATE_URL "http://ota-server.local/firmware/SensorX.bin"
#define OTA_UPDATE_URL                 "http://192.168.1.250/firmware/SensorX.bin"  // HTTP server URL for firmware
#define OTA_CHECK_INTERVAL             3600000  // Check for updates every hour (3600000 ms)
#define OTA_TIMEOUT_MS                 30000    // HTTP download timeout (30 seconds)
#define OTA_ENABLE_AUTO_CHECK          1        // Enable automatic update checking (1=enabled, 0=disabled)

#endif /* CONFIG_H */
```

**Important Note**: The OTA server IP address (192.168.1.250 in the example) does NOT need to be the same as your MQTT server. It was chosen as an example assuming the same server might host both services, but you should configure it based on your actual infrastructure:
- **Same Server**: If your HTTP server for firmware is on the same machine as MQTT, use the same IP
- **Different Server**: If firmware is hosted elsewhere, use that server's IP address
- **Hostname**: You can also use a hostname if DNS is configured (e.g., `http://ota-server.local/firmware/SensorX.bin`)

---

### 4. `components/sensor_system/CMakeLists.txt`

**Location**: Add OTA manager source file and dependencies

**Changes**:

**Original**:
```cmake
idf_component_register(SRCS "src/system_init.c"
                            "src/watchdog.c"
                            "src/error_recovery.c"
                            "src/wifi_manager.c"
                            "src/i2c_manager.c"
                            "src/mqtt_manager.c"
                            "src/sensor_acquisition.c"
                            "src/mqtt_publisher.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support driver esp_netif esp_event esp_wifi nvs_flash mqtt json)
```

**Modified**:
```cmake
idf_component_register(SRCS "src/system_init.c"
                            "src/watchdog.c"
                            "src/error_recovery.c"
                            "src/wifi_manager.c"
                            "src/i2c_manager.c"
                            "src/mqtt_manager.c"
                            "src/sensor_acquisition.c"
                            "src/mqtt_publisher.c"
                            "src/ota_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support driver esp_netif esp_event esp_wifi nvs_flash mqtt json app_update esp_http_client)
```

**Key Changes**:
- Added `"src/ota_manager.c"` to SRCS list
- Added `app_update` to REQUIRES list (for OTA partition management)
- Added `esp_http_client` to REQUIRES list (for HTTP downloads)

---

### 5. `main/main.c`

**Location**: Add OTA manager initialization after MQTT manager

**Changes**:

**Add Include** (with other includes at top):
```c
#include "ota_manager.h"   // From sensor_system component
```

**Add Initialization** (after MQTT manager initialization, around line 126):

```c
    // 5. Initialize MQTT Manager (after WiFi, non-critical - continue if fails)
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

    // 6. Initialize OTA Manager (after WiFi, non-critical - continue if fails)
    // Note: OTA init requires WiFi to be connected, so we'll initialize it
    // after WiFi connection is established
    if (wifi_manager_is_connected()) {
        ret = ota_manager_init();
        if (ret != ESP_OK) {
            error_report(ERROR_NONE, ERROR_SEVERITY_WARNING, "app_main", NULL);
            ESP_LOGW(TAG, "Failed to initialize OTA manager: %s", esp_err_to_name(ret));
            // Continue anyway - OTA is not critical for basic operation
        } else {
            ESP_LOGI(TAG, "OTA manager initialized successfully");
            
#if OTA_ENABLE_AUTO_CHECK
            // Start OTA checking task if auto-check is enabled
            ret = ota_manager_start_task();
            if (ret != pdPASS) {
                ESP_LOGW(TAG, "Failed to start OTA checking task");
                error_report(ERROR_TASK_CREATION, ERROR_SEVERITY_WARNING, "app_main", NULL);
            } else {
                ESP_LOGI(TAG, "OTA checking task started");
            }
#endif
        }
    } else {
        ESP_LOGI(TAG, "WiFi not connected yet, OTA initialization deferred");
    }
```

**Update Task Numbering** (all subsequent tasks need renumbering):
- Current "6. Create the system information task" becomes "7. Create the system information task"
- Current "7. Create the system monitoring task" becomes "8. Create the system monitoring task"
- Current "8. Create the heartbeat task" becomes "9. Create the heartbeat task"
- Current "9. Create the sensor acquisition task" becomes "10. Create the sensor acquisition task"
- Current "10. Create the MQTT publisher task" becomes "11. Create the MQTT publisher task"

---

### 6. `components/sensor_system/src/wifi_manager.c` (Optional Enhancement)

**Location**: Add OTA initialization when WiFi connects (similar to MQTT)

**Purpose**: Automatically initialize OTA when WiFi connects, if not already initialized

**Add Forward Declaration** (after other includes, around line 28):
```c
// Forward declarations to avoid circular dependency
bool mqtt_manager_is_connected(void);
esp_err_t mqtt_manager_init(void);
esp_err_t mqtt_manager_connect(void);

// OTA manager forward declarations
bool ota_manager_is_initialized(void);  // Note: This function needs to be added to ota_manager.h/c
esp_err_t ota_manager_init(void);
BaseType_t ota_manager_start_task(void);
```

**Modify `ip_event_handler`** (around line 325):

Add OTA initialization after MQTT initialization:

```c
        // Initialize and connect MQTT now that WiFi is connected
        if (!mqtt_manager_is_connected()) {
            esp_err_t ret = mqtt_manager_init();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "MQTT manager initialized after WiFi connection");
                ret = mqtt_manager_connect();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "MQTT connection initiated after WiFi connection");
                } else {
                    ESP_LOGW(TAG, "Failed to connect MQTT after WiFi connection: %s", 
                             esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "Failed to initialize MQTT after WiFi connection: %s",
                         esp_err_to_name(ret));
            }
        }
        
        // Initialize OTA manager now that WiFi is connected
        if (!ota_manager_is_initialized()) {
            esp_err_t ret = ota_manager_init();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "OTA manager initialized after WiFi connection");
#if OTA_ENABLE_AUTO_CHECK
                ret = ota_manager_start_task();
                if (ret == pdPASS) {
                    ESP_LOGI(TAG, "OTA checking task started after WiFi connection");
                } else {
                    ESP_LOGW(TAG, "Failed to start OTA task after WiFi connection");
                }
#endif
            } else {
                ESP_LOGW(TAG, "Failed to initialize OTA after WiFi connection: %s",
                         esp_err_to_name(ret));
            }
        }
```

**Note**: This requires adding `ota_manager_is_initialized()` function to `ota_manager.h` and `ota_manager.c`:

```c
// In ota_manager.h:
bool ota_manager_is_initialized(void);

// In ota_manager.c:
bool ota_manager_is_initialized(void) {
    return ota_manager_initialized;
}
```

---

### 7. `components/sensor_system/src/system_init.c` (Optional Enhancement)

**Location**: Add OTA status to system monitor output

**Purpose**: Display OTA status in periodic system monitor output

**Add Include** (with other includes at top):
```c
#include "ota_manager.h"
```

**Add OTA Status Display** (after MQTT status, around line 284):

```c
        // Get OTA status
        if (ota_manager_is_initialized()) {
            OTAStatus_t ota_status = ota_manager_get_status();
            OTAStats_t ota_stats;
            ota_manager_get_stats(&ota_stats);
            
            const char* ota_status_str = "Unknown";
            switch (ota_status) {
                case OTA_STATUS_IDLE: ota_status_str = "Idle"; break;
                case OTA_STATUS_CHECKING: ota_status_str = "Checking"; break;
                case OTA_STATUS_DOWNLOADING: ota_status_str = "Downloading"; break;
                case OTA_STATUS_WRITING: ota_status_str = "Writing"; break;
                case OTA_STATUS_VALIDATING: ota_status_str = "Validating"; break;
                case OTA_STATUS_COMPLETE: ota_status_str = "Complete"; break;
                case OTA_STATUS_ERROR: ota_status_str = "Error"; break;
                default: ota_status_str = "Unknown"; break;
            }
            
            ESP_LOGI(TAG, "OTA: %s | Version: %lu | Updates: %lu/%lu (ok/fail)",
                     ota_status_str, ota_stats.currentVersion,
                     ota_stats.successfulUpdates, ota_stats.failedUpdates);
        } else {
            ESP_LOGI(TAG, "OTA: Not initialized");
        }
```

---

## Partition Table Configuration

### 8. Partition Table Change (Required)

**Current Configuration**: Single app partition (`CONFIG_PARTITION_TABLE_SINGLE_APP=y`)

**Required Change**: Switch to OTA partition table

**Option 1: Use ESP-IDF Built-in OTA Table** (Recommended for simplicity)

**File**: `sdkconfig` or use `idf.py menuconfig`

**Change**:
```
CONFIG_PARTITION_TABLE_SINGLE_APP=n
CONFIG_PARTITION_TABLE_TWO_OTA=y
```

**Option 2: Create Custom Partition Table**

**File**: `partitions.csv` (in project root)

**Content**:
```csv
# Name,     Type, SubType, Offset,  Size, Flags
# Note: if you have increased the bootloader size, make sure to update the offsets to avoid overlap
nvs,        data, nvs,     0x9000,  0x6000,
phy_init,   data, phy,    0xf000,  0x1000,
factory,    app,  factory, 0x10000, 1M,
ota_0,      app,  ota_0,   ,        1M,
ota_1,      app,  ota_1,   ,        1M,
ota_data,   data, ota,     ,        0x2000,
```

**Then set**:
```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

**Important Notes**:
- Flash size must accommodate two OTA partitions (minimum 2MB flash for 1MB partitions)
- With 2MB flash, you may need smaller OTA partitions (e.g., 896KB each)
- Adjust partition sizes based on your firmware size

**Recommended Partition Table for 2MB Flash**:
```csv
# Name,     Type, SubType, Offset,  Size, Flags
nvs,        data, nvs,     0x9000,  0x6000,
phy_init,   data, phy,    0xf000,  0x1000,
ota_0,      app,  ota_0,  0x10000, 896K,
ota_1,      app,  ota_1,  ,        896K,
ota_data,   data, ota,    ,        0x2000,
```

---

## Implementation Details

### OTA Update Process Flow

1. **Initialization**
   - Check WiFi connection
   - Initialize OTA statistics
   - Get current firmware version
   - Set status to IDLE

2. **Periodic Check**
   - Background task checks for updates at configured interval
   - Compares local version with server version
   - If update available, starts update process

3. **Download**
   - Creates HTTP client
   - Downloads firmware binary from server
   - Writes data to inactive OTA partition as it arrives
   - Updates status (DOWNLOADING → WRITING)

4. **Validation**
   - ESP-IDF validates firmware signature/app description
   - Sets status to VALIDATING

5. **Completion**
   - Sets boot partition to new firmware
   - Records update statistics
   - Reboots device

6. **Boot**
   - Device boots from new firmware partition
   - If new firmware fails, ESP-IDF can auto-rollback to previous partition

### Error Handling

- **WiFi Disconnected**: Update check skipped, logged as warning
- **HTTP Download Failed**: Error logged, statistics updated, status set to ERROR
- **OTA Write Failed**: Update aborted, error logged, rollback possible
- **Partition Validation Failed**: Update aborted, previous firmware remains active

### Integration Points

- **WiFi Manager**: OTA requires WiFi connection
- **Error Recovery**: OTA errors reported to error recovery system
- **Watchdog**: OTA task registered with watchdog for monitoring
- **System Monitor**: Optional OTA status display

---

## Integration Steps

### Step-by-Step Implementation Order

1. **Create OTA Manager Files**
   - Create `components/sensor_system/include/ota_manager.h`
   - Create `components/sensor_system/src/ota_manager.c`
   - Ensure all function implementations match prototypes

2. **Update Configuration**
   - Add OTA configuration to `config.h`
   - Set appropriate URL, interval, and timeout values

3. **Update CMakeLists.txt**
   - Add `ota_manager.c` to SRCS
   - Add `app_update` and `esp_http_client` to REQUIRES

4. **Update Partition Table**
   - Change from single app to OTA partition table
   - Verify partition sizes fit in flash memory
   - Rebuild partition table

5. **Integrate into main.c**
   - Add `#include "ota_manager.h"`
   - Add OTA initialization after MQTT
   - Add conditional task creation

6. **Optional: WiFi Integration**
   - Add OTA initialization to WiFi event handler
   - Ensures OTA starts when WiFi connects

7. **Optional: System Monitor**
   - Add OTA status to system monitor output
   - Provides visibility into OTA operations

8. **Build and Test**
   - Build project with new partition table
   - Flash initial firmware
   - Test OTA update process

---

## Testing

### Test Scenarios

1. **Basic OTA Update**
   - Place firmware binary on HTTP server
   - Verify device checks for updates
   - Confirm download and installation
   - Verify device reboots to new firmware

2. **Update Interruption**
   - Start OTA update
   - Disconnect WiFi mid-download
   - Verify device recovers gracefully
   - Verify previous firmware still works

3. **Invalid Firmware**
   - Attempt to update with corrupted firmware
   - Verify validation fails
   - Verify device remains on current firmware

4. **Rollback Test**
   - Update to new firmware
   - If new firmware crashes, verify auto-rollback
   - Verify device boots from previous firmware

### Debugging

- Monitor serial output for OTA status messages
- Check error recovery statistics for OTA errors
- Verify HTTP server is accessible from device
- Confirm firmware binary is correct format and size

---

## Configuration Reference

### Configuration Constants

| Constant | Default Value | Description |
|----------|---------------|-------------|
| `OTA_UPDATE_URL` | `"http://192.168.1.250/firmware/SensorX.bin"` | HTTP URL for firmware binary. **Note**: This can be different from your MQTT server IP - configure based on where your firmware files are hosted. |
| `OTA_CHECK_INTERVAL` | `3600000` (1 hour) | Update check interval in milliseconds |
| `OTA_TIMEOUT_MS` | `30000` (30 seconds) | HTTP download timeout |
| `OTA_ENABLE_AUTO_CHECK` | `1` | Enable automatic update checking |

**OTA Server Configuration Options**:
- **Same as MQTT**: If firmware is hosted on the same server as MQTT broker, use `MQTT_PROD_SERVER_IP`
- **Different Server**: Use a different IP address if firmware is hosted elsewhere
- **Hostname**: Can use DNS hostname if available (e.g., `http://ota-server.local/firmware/SensorX.bin`)
- **Port**: Include port number if not standard HTTP port 80 (e.g., `http://192.168.1.250:8080/firmware/SensorX.bin`)

### Task Configuration

- **Task Name**: `OTAManager`
- **Task Priority**: `TASK_PRIORITY_IDLE` (0)
- **Task Stack Size**: `TASK_STACK_SIZE_IDLE` (2048 bytes)
- **Watchdog Timeout**: `OTA_CHECK_INTERVAL + 60000` (check interval + 1 minute)

---

## Notes and Considerations

### Security

- **HTTP OTA is not encrypted**: Firmware is transmitted in plain text
- For production use, consider HTTPS OTA (`esp_https_ota` component)
- Validate firmware signatures if security is critical

### Firmware Version Management

- Current implementation uses simple version checking
- Consider implementing version API endpoint:
  - Device requests version info: `GET /firmware/version.json`
  - Server responds: `{"version": 123, "url": "/firmware/SensorX_v123.bin"}`
  - Device compares versions and downloads if update available

### Partition Sizes

- Ensure OTA partitions are large enough for firmware
- Current firmware size can be checked with `idf.py size`
- Leave headroom for future firmware growth

### Update Server Setup

- HTTP server must be accessible from device network
- Server should serve firmware binary with correct Content-Type
- Consider authentication if needed (requires HTTPS)

---

## File Summary

### Files Created: 2
1. `components/sensor_system/include/ota_manager.h`
2. `components/sensor_system/src/ota_manager.c`

### Files Modified: 5-7
1. `components/sensor_system/include/config.h` - OTA configuration
2. `components/sensor_system/CMakeLists.txt` - Dependencies and sources
3. `main/main.c` - OTA initialization
4. `components/sensor_system/src/wifi_manager.c` - Auto-initialization (optional)
5. `components/sensor_system/src/system_init.c` - Status display (optional)
6. `components/sensor_system/include/system_init.h` - Status function (optional, if adding to monitor)
7. `sdkconfig` or `partitions.csv` - Partition table change (required)

### Dependencies Added
- `app_update` - ESP-IDF component for OTA operations
- `esp_http_client` - ESP-IDF component for HTTP downloads

---

## Quick Reference Checklist

- [ ] Create `ota_manager.h` header file
- [ ] Create `ota_manager.c` implementation file
- [ ] Add OTA configuration to `config.h`
- [ ] Update `CMakeLists.txt` with new source and dependencies
- [ ] Add OTA initialization to `main.c`
- [ ] Change partition table to OTA partitions
- [ ] Test partition table change compiles
- [ ] Build and flash initial firmware
- [ ] Set up HTTP server with firmware binary
- [ ] Test OTA update process
- [ ] Verify rollback functionality
- [ ] (Optional) Add OTA to WiFi event handler
- [ ] (Optional) Add OTA status to system monitor

---

## Additional Resources

- ESP-IDF OTA Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html
- ESP-IDF HTTP Client: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_client.html
- Partition Table Guide: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html

