# Future Upgrades and Enhancements

This document outlines planned future upgrades and enhancements for the SensorX ESP32 system. These features are deferred from the initial implementation but represent valuable additions for enhanced functionality.

## Table of Contents

1. [Time Management: Built-in RTC with SNTP](#time-management-built-in-rtc-with-sntp)
2. [Data Logging System](#data-logging-system)
3. [I2C Buzzer Manager](#i2c-buzzer-manager)

---

## Time Management: Built-in RTC with SNTP

### Overview

Replace external I2C RTC (RV1805) with ESP32-C6's built-in RTC synchronized via SNTP (Simple Network Time Protocol). This simplifies hardware requirements, reduces I2C bus traffic, and leverages standard ESP-IDF time management capabilities.

### Rationale

**Current State:**
- External I2C RTC (RV1805) at address 0x69
- Requires I2C communication for time reads
- Additional hardware component
- Manual synchronization with NTP

**Proposed State:**
- Use ESP32-C6's built-in RTC
- Automatic synchronization via SNTP after WiFi connects
- No additional hardware required
- Standard ESP-IDF time functions (`time()`, `gettimeofday()`, `localtime()`)

### Benefits

1. **Hardware Simplification**: Eliminates RV1805 RTC chip
2. **Reduced I2C Traffic**: No periodic RTC reads needed
3. **Automatic Synchronization**: SNTP syncs automatically after WiFi connection
4. **Standard APIs**: Uses ESP-IDF standard time functions
5. **Lower Cost**: One less component to purchase and integrate

### Implementation Details

#### 1. SNTP Initialization

```c
#include "esp_sntp.h"

void sntp_sync_time_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronized via SNTP");
}

void init_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");  // Primary NTP server
    sntp_setservername(1, "time.nist.gov"); // Secondary NTP server
    sntp_set_time_sync_notification_cb(sntp_sync_time_cb);
    sntp_init();
}
```

#### 2. Time Retrieval

```c
#include <time.h>

// Get current time
time_t now;
time(&now);

// Format timestamp
struct tm timeinfo;
localtime_r(&now, &timeinfo);
char timestamp[64];
strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
```

#### 3. Integration Points

- **WiFi Manager**: Call `init_sntp()` after WiFi connects
- **Logging System**: Use `time()` for timestamps
- **MQTT Publisher**: Include timestamp in JSON payload
- **System Monitor**: Display synchronized time status

### Configuration

Add to `config.h`:
```c
// SNTP Configuration
#define SNTP_SERVER_PRIMARY     "pool.ntp.org"
#define SNTP_SERVER_SECONDARY   "time.nist.gov"
#define SNTP_SYNC_INTERVAL_MS   3600000  // 1 hour
#define SNTP_TIMEZONE_OFFSET    0        // UTC offset in seconds
```

### Testing Criteria

- [ ] SNTP initializes after WiFi connection
- [ ] Time synchronizes within 30 seconds of WiFi connect
- [ ] Time persists across reboots (RTC maintains time)
- [ ] Timestamps accurate for logging and MQTT
- [ ] Time syncs periodically (every hour)

### Dependencies

- ESP-IDF SNTP component (`esp_sntp.h`)
- WiFi connection established
- NTP server access (internet or local NTP server)

---

## Data Logging System

### Overview

Implement a comprehensive data logging system for capturing sensor data for troubleshooting and analysis. Multiple implementation options are available, each with different trade-offs.

### Use Case

- **Primary Need**: Capture raw interface data when troubleshooting issues
- **No Serial Link**: Device deployed without physical serial access
- **Remote Access**: Retrieve logs without physical access to device
- **On-Demand**: Enable logging only when needed (not continuous)

### Option 1: HTTP POST to Local Server (Recommended)

#### Architecture

```
ESP32-C6 Sensor
    │
    ├─> WiFi Connection
    │
    └─> HTTP POST to Local Server
         │
         ├─> POST http://192.168.1.250/api/sensor/log
         │   Body: JSON or binary data
         │   Headers: Content-Type, timestamp, device-id
         │
         └─> Server stores to:
             - Database (MySQL, PostgreSQL, InfluxDB)
             - File system (CSV files, JSON files)
             - Time-series database
```

#### Implementation

**1. HTTP Client Manager**

Create `components/sensor_system/src/http_logging_manager.c`:

```c
#include "esp_http_client.h"
#include "esp_log.h"

static const char *TAG = "HTTP_LOGGING";

typedef struct {
    bool enabled;
    char server_url[128];
    uint16_t port;
    uint32_t interval_ms;
    uint32_t buffer_size;
} http_logging_config_t;

static http_logging_config_t logging_config = {
    .enabled = false,
    .server_url = "192.168.1.250",
    .port = 80,
    .interval_ms = 1000,
    .buffer_size = 512
};

esp_err_t http_logging_init(void) {
    // Initialize HTTP client configuration
    return ESP_OK;
}

esp_err_t http_logging_enable(void) {
    logging_config.enabled = true;
    ESP_LOGI(TAG, "HTTP logging enabled");
    return ESP_OK;
}

esp_err_t http_logging_disable(void) {
    logging_config.enabled = false;
    ESP_LOGI(TAG, "HTTP logging disabled");
    return ESP_OK;
}

esp_err_t http_logging_send_entry(const char *json_data) {
    if (!logging_config.enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%d/api/sensor/log", 
             logging_config.server_url, logging_config.port);
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_post_field(client, json_data, strlen(json_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");
    
    esp_err_t ret = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    
    esp_http_client_cleanup(client);
    
    if (ret == ESP_OK && status_code == 200) {
        ESP_LOGD(TAG, "Log entry sent successfully");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to send log entry: %d", status_code);
        return ESP_FAIL;
    }
}
```

**2. Logging Task**

```c
void vHttpLoggingTask(void *pvParameters) {
    ESP_LOGI(TAG, "HTTP logging task started");
    
    const TickType_t xLogInterval = pdMS_TO_TICKS(logging_config.interval_ms);
    
    for (;;) {
        if (logging_config.enabled && wifi_manager_is_connected()) {
            // Format log entry
            char log_buffer[512];
            snprintf(log_buffer, sizeof(log_buffer),
                "{\"timestamp\":%lu,\"cycle\":%ld,\"flow1\":%u,\"adc_x1\":%.3f,...}",
                esp_timer_get_time() / 1000,
                genericSens_.generic.cycle_count,
                flow1.pulses,
                genericSens_.generic.adc_x1);
            
            // Send via HTTP POST
            http_logging_send_entry(log_buffer);
        }
        
        vTaskDelay(xLogInterval);
    }
}
```

**3. Server-Side Example (Python Flask)**

```python
from flask import Flask, request
import json
from datetime import datetime

app = Flask(__name__)

@app.route('/api/sensor/log', methods=['POST'])
def log_sensor_data():
    data = request.get_json()
    timestamp = datetime.now().isoformat()
    
    # Store to file
    filename = f'/logs/sensor_{datetime.now().strftime("%Y%m%d")}.jsonl'
    with open(filename, 'a') as f:
        f.write(json.dumps({**data, 'server_timestamp': timestamp}) + '\n')
    
    return {'status': 'ok'}, 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)
```

#### Advantages

- ✅ No additional hardware required
- ✅ Uses existing WiFi infrastructure
- ✅ Remote access (no physical access needed)
- ✅ Unlimited storage capacity (server-side)
- ✅ Easy to query and analyze
- ✅ Can enable/disable remotely
- ✅ Multiple sensors can log to same server

#### Disadvantages

- ❌ Requires WiFi connection
- ❌ Requires server setup
- ❌ Network dependency

### Option 2: MQTT-Based Logging

#### Architecture

```
ESP32-C6 Sensor
    │
    └─> MQTT Publish (already working)
         │
         ├─> Topic: sensor/log/raw (QoS 1, retain=false)
         │   Payload: JSON or binary log entry
         │
         └─> MQTT Broker (192.168.1.250)
              │
              └─> Server-side subscriber
                   └─> Writes to file/database
```

#### Implementation

**1. Add Logging Topic**

```c
// In mqtt_publisher.c or new logging task
mqtt_manager_publish_json("sensor/log/raw", json_string, 1, false);
// QoS 1 ensures delivery, retain=false for logging
```

**2. Server-Side Subscriber (Python)**

```python
import paho.mqtt.client as mqtt
import json

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    # Write to file or database
    with open(f'/logs/sensor_{date}.jsonl', 'a') as f:
        f.write(json.dumps(data) + '\n')

client = mqtt.Client()
client.on_message = on_message
client.connect("192.168.1.250", 1883, 60)
client.subscribe("sensor/log/raw", qos=1)
client.loop_forever()
```

#### Advantages

- ✅ Uses existing MQTT infrastructure
- ✅ Minimal code changes needed
- ✅ Reliable delivery (QoS 1)
- ✅ Can buffer on broker if server is down

#### Disadvantages

- ❌ Requires MQTT broker running
- ❌ Network dependency

### Option 3: W25Q128 SPI Flash Storage

#### Overview

Use W25Q128 SPI flash module (16 MB) for onboard storage with FATFS file system.

#### Hardware Requirements

- W25Q128 SPI flash module (16 MB)
- SPI connection:
  - CS: GPIO10 (or alternative)
  - SCK: GPIO21 (or alternative)
  - MOSI: GPIO22
  - MISO: GPIO23

**Note**: GPIO21 is now available for SPI SCK (fan control moved to GPIO0). Options:
- Use GPIO21 for SPI SCK (fan control moved to GPIO0)
- Use alternative SPI pins
- Use SPI2_HOST with custom pins

#### Implementation

**1. SPI Flash Initialization**

```c
#include "esp_vfs_fat.h"
#include "driver/spi_master.h"

esp_vfs_fat_spiflash_mount_config_t mount_config = {
    .base_path = "/spiflash",
    .partition_label = "storage",
    .max_files = 5,
    .format_if_mount_failed = true
};

esp_vfs_fat_spiflash_mount("/spiflash", "storage", &mount_config, &s_wl_handle);
```

**2. Logging Manager**

```c
void vFlashLoggingTask(void *pvParameters) {
    FILE *log_file = NULL;
    char filename[64];
    
    // Generate filename with timestamp
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    snprintf(filename, sizeof(filename), 
             "/spiflash/logs/sensor_%04d%02d%02d_%02d%02d%02d.csv",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    
    log_file = fopen(filename, "a");
    if (log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open log file");
        return;
    }
    
    // Write CSV header
    fprintf(log_file, "Timestamp,Cycle,Flow1_Pulses,Flow1_ms,ADC_x1,ADC_x2,...\n");
    
    for (;;) {
        if (logging_enabled) {
            // Take mutex on genericSens_
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Format and write log entry
                fprintf(log_file, "%lu,%ld,%u,%u,%.3f,%.3f,...\n",
                        esp_timer_get_time() / 1000,
                        genericSens_.generic.cycle_count,
                        flow1.pulses,
                        flow1.milliseconds,
                        genericSens_.generic.adc_x1,
                        genericSens_.generic.adc_x2);
                
                xSemaphoreGive(sensor_data_mutex);
                fflush(log_file);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    fclose(log_file);
}
```

#### Storage Capacity

- **W25Q128**: 16 MB total
- **CSV format**: ~250 bytes/entry → ~64,000 entries (~18 hours @ 1/sec)
- **Binary format**: ~104 bytes/entry → ~154,000 entries (~43 hours @ 1/sec)
- **Recommendation**: Use binary format, rotate files hourly, keep last 7-14 days

#### Advantages

- ✅ No network dependency
- ✅ Standard file system (FAT32)
- ✅ Removable (can read on PC)
- ✅ Reliable (no moving parts)

#### Disadvantages

- ❌ Limited capacity (16 MB)
- ❌ Requires SPI pins
- ❌ Physical access needed to retrieve
- ❌ Additional hardware cost (~$3)

### Recommendation

**Primary Choice: HTTP POST to Local Server**

- Best fit for remote access requirement
- No additional hardware
- Uses existing WiFi infrastructure
- Unlimited storage capacity
- Easy to enable/disable remotely

**Fallback: MQTT Logging**

- If HTTP server setup is complex
- Leverages existing MQTT infrastructure
- Minimal code changes

**Alternative: W25Q128**

- If network connectivity is unreliable
- If physical access is acceptable
- If offline logging is required

### Implementation Plan

#### Phase 1: HTTP Logging (Basic)
1. Create HTTP logging manager (`http_logging_manager.h/c`)
2. HTTP POST single log entries when enabled
3. Simple server endpoint (Flask/Python)
4. Store to JSONL files (one file per day)

#### Phase 2: Enhanced Features
1. Batch logging (multiple entries per POST)
2. Retry logic and buffering
3. Remote enable/disable via MQTT
4. Server-side database storage

#### Phase 3: Advanced Features
1. Compression (gzip JSON before POST)
2. Authentication (API key in header)
3. HTTPS (secure logging)
4. Server-side analysis tools

### Configuration

Add to `config.h`:
```c
// HTTP Logging Configuration
#define HTTP_LOGGING_ENABLED           0      // 0=disabled, 1=enabled
#define HTTP_LOGGING_SERVER_IP          "192.168.1.250"
#define HTTP_LOGGING_SERVER_PORT        80
#define HTTP_LOGGING_INTERVAL_MS        1000   // Log every 1 second when enabled
#define HTTP_LOGGING_BUFFER_SIZE       512    // JSON buffer size
#define HTTP_LOGGING_RETRY_COUNT       3      // Retry attempts on failure
#define HTTP_LOGGING_RETRY_DELAY_MS    1000   // Delay between retries
```

### Testing Criteria

- [ ] HTTP POST successfully sends log entries
- [ ] Server receives and stores log data
- [ ] Logging can be enabled/disabled remotely
- [ ] Retry logic handles network failures
- [ ] Buffering prevents data loss during WiFi outages
- [ ] Log format matches expected structure

---

## I2C Buzzer Manager

### Overview

Implement I2C buzzer manager for audio feedback and alerts. Uses Sparkfun Qwiic Buzzer (I2C address 0x34) for system status notifications, error alerts, and user feedback.

### Hardware

- **Device**: Sparkfun Qwiic Buzzer
- **I2C Address**: 0x34
- **Interface**: I2C (Qwiic connector)
- **Features**:
  - Configurable frequency (100-20000 Hz)
  - Configurable duration (1-65535 ms)
  - Configurable volume (0-255)
  - Pre-programmed sound effects

### Use Cases

1. **WiFi Connection Feedback**: Audio tones for connection status
   - Short beep: Connection attempt started
   - Success tone: Connection successful
   - Error tone: Connection failed

2. **MQTT Connection Feedback**: Audio tones for MQTT status
   - High→Low tones: Production server connected
   - Low→High tones: Development server connected
   - Error tone: Connection failed

3. **Error Alerts**: Audio alerts for system errors
   - Short high-pitched beep: I2C device error
   - Longer lower tone: MQTT/connection error
   - Repeated beeps: Critical error

4. **System Status**: Periodic status beeps
   - Single beep: System operational
   - Pattern beeps: Specific status indicators

### Implementation

#### 1. Buzzer Manager Header

Create `components/sensor_system/include/i2c_buzzer_manager.h`:

```c
#ifndef I2C_BUZZER_MANAGER_H
#define I2C_BUZZER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Buzzer configuration
#define BUZZER_I2C_ADDR         0x34
#define BUZZER_DEFAULT_VOLUME    255  // Max volume (0-255)

// Sound effect IDs (Sparkfun Qwiic Buzzer)
#define BUZZER_SOUND_YES        2   // "Robot saying 'Yes'"
#define BUZZER_SOUND_NO         4   // "Sound effect 'NO'"

// Function prototypes
esp_err_t i2c_buzzer_manager_init(void);
esp_err_t i2c_buzzer_manager_deinit(void);
bool i2c_buzzer_manager_is_present(void);

// Basic tone control
esp_err_t i2c_buzzer_configure(uint16_t frequency_hz, uint16_t duration_ms, uint8_t volume);
esp_err_t i2c_buzzer_on(void);
esp_err_t i2c_buzzer_off(void);

// Sound effects
esp_err_t i2c_buzzer_play_sound_effect(uint8_t effect_id, uint8_t volume);

// Predefined alert functions
esp_err_t i2c_buzzer_wifi_connecting(void);
esp_err_t i2c_buzzer_wifi_connected(void);
esp_err_t i2c_buzzer_wifi_failed(void);
esp_err_t i2c_buzzer_mqtt_prod_connected(void);
esp_err_t i2c_buzzer_mqtt_dev_connected(void);
esp_err_t i2c_buzzer_mqtt_failed(void);
esp_err_t i2c_buzzer_error_alert(uint8_t error_type, uint8_t error_count);

#endif // I2C_BUZZER_MANAGER_H
```

#### 2. Buzzer Manager Implementation

Create `components/sensor_system/src/i2c_buzzer_manager.c`:

```c
#include "i2c_buzzer_manager.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "I2C_BUZZER";

// Buzzer register addresses (Sparkfun Qwiic Buzzer)
#define BUZZER_REG_ID           0x00
#define BUZZER_REG_FREQUENCY    0x01
#define BUZZER_REG_TIME         0x03
#define BUZZER_REG_VOLUME       0x05
#define BUZZER_REG_SOUND_EFFECT 0x06
#define BUZZER_REG_COMMAND      0x07

// Command values
#define BUZZER_CMD_ON           0x01
#define BUZZER_CMD_OFF          0x00

static bool buzzer_present = false;
static i2c_master_dev_handle_t buzzer_device = NULL;

esp_err_t i2c_buzzer_manager_init(void) {
    if (buzzer_present) {
        ESP_LOGW(TAG, "Buzzer manager already initialized");
        return ESP_OK;
    }
    
    // Get I2C bus handle
    i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus_handle();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Configure buzzer device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BUZZER_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &buzzer_device);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Buzzer not found at 0x%02X", BUZZER_I2C_ADDR);
        return ret;
    }
    
    // Probe device
    uint8_t id_reg = 0;
    ret = i2c_master_transmit_receive(buzzer_device, &BUZZER_REG_ID, 1, &id_reg, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to probe buzzer");
        i2c_master_bus_rm_device(buzzer_device);
        buzzer_device = NULL;
        return ret;
    }
    
    buzzer_present = true;
    ESP_LOGI(TAG, "Buzzer initialized at 0x%02X", BUZZER_I2C_ADDR);
    
    return ESP_OK;
}

bool i2c_buzzer_manager_is_present(void) {
    return buzzer_present;
}

esp_err_t i2c_buzzer_configure(uint16_t frequency_hz, uint16_t duration_ms, uint8_t volume) {
    if (!buzzer_present || buzzer_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[6];
    data[0] = BUZZER_REG_FREQUENCY;
    data[1] = frequency_hz & 0xFF;
    data[2] = (frequency_hz >> 8) & 0xFF;
    data[3] = BUZZER_REG_TIME;
    data[4] = duration_ms & 0xFF;
    data[5] = (duration_ms >> 8) & 0xFF;
    
    esp_err_t ret = i2c_master_transmit(buzzer_device, data, 6, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t vol_data[2] = {BUZZER_REG_VOLUME, volume};
    ret = i2c_master_transmit(buzzer_device, vol_data, 2, pdMS_TO_TICKS(100));
    
    return ret;
}

esp_err_t i2c_buzzer_on(void) {
    if (!buzzer_present || buzzer_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t cmd_data[2] = {BUZZER_REG_COMMAND, BUZZER_CMD_ON};
    return i2c_master_transmit(buzzer_device, cmd_data, 2, pdMS_TO_TICKS(100));
}

esp_err_t i2c_buzzer_off(void) {
    if (!buzzer_present || buzzer_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t cmd_data[2] = {BUZZER_REG_COMMAND, BUZZER_CMD_OFF};
    return i2c_master_transmit(buzzer_device, cmd_data, 2, pdMS_TO_TICKS(100));
}

esp_err_t i2c_buzzer_play_sound_effect(uint8_t effect_id, uint8_t volume) {
    if (!buzzer_present || buzzer_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[3] = {BUZZER_REG_SOUND_EFFECT, effect_id, volume};
    return i2c_master_transmit(buzzer_device, data, 3, pdMS_TO_TICKS(100));
}

// Predefined alert functions
esp_err_t i2c_buzzer_wifi_connecting(void) {
    if (!buzzer_present) return ESP_OK;
    return i2c_buzzer_configure(1000, 100, BUZZER_DEFAULT_VOLUME);
}

esp_err_t i2c_buzzer_wifi_connected(void) {
    if (!buzzer_present) return ESP_OK;
    return i2c_buzzer_play_sound_effect(BUZZER_SOUND_YES, BUZZER_DEFAULT_VOLUME);
}

esp_err_t i2c_buzzer_wifi_failed(void) {
    if (!buzzer_present) return ESP_OK;
    esp_err_t ret = i2c_buzzer_configure(400, 500, BUZZER_DEFAULT_VOLUME);
    if (ret == ESP_OK) {
        i2c_buzzer_on();
    }
    return ret;
}

esp_err_t i2c_buzzer_mqtt_prod_connected(void) {
    if (!buzzer_present) return ESP_OK;
    // High→Low tones (2kHz then 1.5kHz, 1 sec each)
    esp_err_t ret = i2c_buzzer_configure(2000, 1000, BUZZER_DEFAULT_VOLUME);
    if (ret == ESP_OK) {
        i2c_buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(1100));
        i2c_buzzer_configure(1500, 1000, BUZZER_DEFAULT_VOLUME);
        i2c_buzzer_on();
    }
    return ret;
}

esp_err_t i2c_buzzer_mqtt_dev_connected(void) {
    if (!buzzer_present) return ESP_OK;
    // Low→High tones (1.5kHz then 2kHz, 1 sec each)
    esp_err_t ret = i2c_buzzer_configure(1500, 1000, BUZZER_DEFAULT_VOLUME);
    if (ret == ESP_OK) {
        i2c_buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(1100));
        i2c_buzzer_configure(2000, 1000, BUZZER_DEFAULT_VOLUME);
        i2c_buzzer_on();
    }
    return ret;
}

esp_err_t i2c_buzzer_mqtt_failed(void) {
    if (!buzzer_present) return ESP_OK;
    return i2c_buzzer_play_sound_effect(BUZZER_SOUND_NO, BUZZER_DEFAULT_VOLUME);
}

esp_err_t i2c_buzzer_error_alert(uint8_t error_type, uint8_t error_count) {
    if (!buzzer_present) return ESP_OK;
    
    // I2C device errors: short, high-pitched beep
    if (error_type >= 0x10 && error_type <= 0x17) {
        i2c_buzzer_configure(2000, 150, BUZZER_DEFAULT_VOLUME);
        i2c_buzzer_on();
    } else {
        // Other errors: longer, lower tone
        i2c_buzzer_configure(800, 300, BUZZER_DEFAULT_VOLUME);
        i2c_buzzer_on();
    }
    
    // Additional beep for repeated errors
    if (error_count > 1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        i2c_buzzer_configure(1200, 100, BUZZER_DEFAULT_VOLUME);
        i2c_buzzer_on();
    }
    
    return ESP_OK;
}
```

#### 3. Integration Points

**WiFi Manager:**
```c
// In wifi_manager.c
#include "i2c_buzzer_manager.h"

void wifi_connection_started(void) {
    i2c_buzzer_wifi_connecting();
}

void wifi_connection_success(void) {
    i2c_buzzer_wifi_connected();
}

void wifi_connection_failed(void) {
    i2c_buzzer_wifi_failed();
}
```

**MQTT Manager:**
```c
// In mqtt_manager.c
#include "i2c_buzzer_manager.h"

void mqtt_prod_connected(void) {
    i2c_buzzer_mqtt_prod_connected();
}

void mqtt_dev_connected(void) {
    i2c_buzzer_mqtt_dev_connected();
}

void mqtt_connection_failed(void) {
    i2c_buzzer_mqtt_failed();
}
```

**Error Recovery:**
```c
// In error_recovery.c
#include "i2c_buzzer_manager.h"

void error_report(uint8_t error_code, uint8_t severity, const char *source, void *data) {
    // ... existing error reporting ...
    
    // Audio alert for errors
    if (severity >= ERROR_SEVERITY_ERROR) {
        i2c_buzzer_error_alert(error_code, error_count);
    }
}
```

### Configuration

Add to `config.h`:
```c
// Buzzer Configuration
#define BUZZER_ENABLED                  1      // 0=disabled, 1=enabled
#define BUZZER_I2C_ADDR                 0x34
#define BUZZER_DEFAULT_VOLUME           255    // 0-255
#define BUZZER_WIFI_FEEDBACK_ENABLED   1      // Audio feedback for WiFi events
#define BUZZER_MQTT_FEEDBACK_ENABLED   1      // Audio feedback for MQTT events
#define BUZZER_ERROR_ALERTS_ENABLED     1      // Audio alerts for errors
```

### Testing Criteria

- [ ] Buzzer initializes and is detected at 0x34
- [ ] WiFi connection tones work correctly
- [ ] MQTT connection tones work correctly
- [ ] Error alerts play appropriate tones
- [ ] Sound effects play correctly
- [ ] Buzzer gracefully handles missing device
- [ ] No I2C bus conflicts with other devices

### Dependencies

- I2C Manager (for bus access)
- ESP-IDF I2C Master Driver
- Sparkfun Qwiic Buzzer hardware

### Notes

- Buzzer is optional - system should work without it
- All buzzer functions should check `buzzer_present` before operations
- Buzzer feedback can be disabled via configuration
- Consider user environment (some applications may not want audio feedback)

---

## Implementation Priority

1. **Time Management (SNTP/RTC)**: High priority - simplifies hardware and improves reliability
2. **Data Logging (HTTP POST)**: Medium priority - valuable for troubleshooting
3. **I2C Buzzer**: Low priority - nice-to-have feature for user feedback

---

## Revision History

- **2025-01-15**: Initial document creation
  - Documented SNTP/RTC upgrade approach
  - Documented HTTP POST logging system
  - Documented MQTT logging alternative
  - Documented W25Q128 SPI flash option
  - Documented I2C Buzzer manager implementation

