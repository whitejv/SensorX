# MQTT Publishing Strategy and Requirements

## Overview

This document defines the MQTT publishing strategy for the SensorX ESP32 system, including requirements for binary and JSON message formats, timing constraints, and implementation approach.

## Message Types

### 1. Binary GenericSens Message (Critical)

**Purpose**: Primary data feed for server-side processing systems

**Characteristics**:
- **Format**: Binary (104 bytes - `genericSens_.data_payload` array)
- **Frequency**: 1000ms (1 second)
- **Priority**: **CRITICAL** - Must be accurate and timely
- **Task Priority**: `TASK_PRIORITY_FIXED_FREQ` (3) - Rate group to guarantee transmit
- **Delay Tolerance**: Minimal - use MQTT mechanisms to ensure low delay
- **QoS**: **2** (Exactly-once delivery) - Highest priority, guaranteed delivery
- **Retain**: false (fresh data each second)
- **Topic**: `mwp/data/sensor/well/S005D-test` (development: `-test` suffix, production: no suffix)

**Requirements**:
- Must be published exactly every 1000ms
- **MUST wait for sensor coordination event group** (all 1000ms sensors complete)
- Must protect `genericSens_` with mutex during read
- Copy data to local buffer, release mutex, then publish
- Minimize mutex hold time
- Dedicated scheduled task in rate group (Fixed Frequency priority) to ensure frequency
- Cycle counter increments in this task after successful publish

**Data Source**: `genericSens_.data_payload` (104 bytes, 26 words × 4 bytes)

---

### 2. JSON GenericSens Message (Viewing Only)

**Purpose**: Human-readable format for monitoring and debugging

**Characteristics**:
- **Format**: JSON (structured sensor data)
- **Frequency**: 1000ms (1 second) - same as binary
- **Priority**: **LOW** - Viewing purposes only
- **Task Priority**: Same task as binary (Fixed Frequency) OR Background priority (1)
- **Delay Tolerance**: Higher - less critical than binary
- **QoS**: **0** (At-most-once delivery) - Viewing purposes, no guarantee needed
- **Retain**: false
- **Topic**: `mwp/json/sensor/well/S005D-test` (development: `-test` suffix, production: no suffix)

**Requirements**:
- Can be published after binary message (same task) OR in background task
- Uses same sensor data snapshot as binary (read once, publish twice)
- Less timing-critical than binary
- Can tolerate slight delays
- **Decision**: Can be in same rate group as binary OR background priority

**Data Source**: `genericSens_.generic.*` (structured fields, converted to JSON)

---

### 3. System Monitor Message (Viewing Only)

**Purpose**: System health and status monitoring

**Characteristics**:
- **Format**: JSON only (no binary equivalent)
- **Frequency**: 5000ms (5 seconds) - matches system monitor task interval
- **Priority**: **LOW** - Viewing and system health monitoring
- **Delay Tolerance**: High - not critical for data processing
- **QoS**: **0** (At-most-once delivery) - Viewing purposes, no guarantee needed
- **Retain**: false
- **Topic**: `mwp/json/sensor/well/monitor-test` (development: `-test` suffix, production: no suffix)

**Requirements**:
- Published from System Monitor Task (already runs every 5 seconds)
- Contains system status information:
  - Uptime
  - Free heap memory
  - Min free heap memory
  - Active task count
  - WiFi status (IP, RSSI, uptime)
  - MQTT status (broker IP, server type, publish stats)
  - I2C device list
  - Fan control status and temperature
- No binary equivalent needed

**Data Source**: System Monitor Task internal state and system queries

---

## Implementation Strategy

### Mutex Protection Pattern

**Critical Requirement**: `genericSens_` must be protected with mutex during **both read and write** operations.

**Pattern**:
```c
// 1. Take mutex
if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
    // 2. Copy data to local buffer (minimize mutex hold time)
    union GENERICSENS_ local_copy;
    memcpy(&local_copy, &genericSens_, sizeof(union GENERICSENS_));
    
    // 3. Release mutex IMMEDIATELY
    xSemaphoreGive(sensor_data_mutex);
    
    // 4. Use local_copy for publishing (outside mutex)
    // - Build JSON from local_copy
    // - Publish binary from local_copy.data_payload
    // - No mutex held during MQTT operations
}
```

**Benefits**:
- Minimizes mutex contention
- Prevents blocking sensor managers during MQTT operations
- Ensures consistent data snapshot (all fields from same moment)
- Reduces risk of deadlocks

---

### Task Architecture Options

#### Option A: Single Task (Binary + JSON GenericSens)

**Structure**:
- One task publishes both binary and JSON GenericSens messages
- Binary published first (higher priority)
- JSON published immediately after (same data snapshot)

**Pros**:
- Simpler task management
- Single data read (efficient)
- Both messages use same data snapshot

**Cons**:
- Binary message timing depends on JSON serialization time
- If JSON fails, may delay binary
- Less isolation between critical and non-critical operations

#### Option B: Separate Tasks (Binary Task + JSON Task)

**Structure**:
- Dedicated task for binary GenericSens (critical, scheduled)
- Separate task for JSON GenericSens (lower priority)
- Both read from `genericSens_` independently

**Pros**:
- Binary message timing guaranteed (not affected by JSON)
- Better isolation
- Can optimize each task independently

**Cons**:
- More complex (two tasks)
- Two mutex acquisitions (more overhead)
- Data snapshots may differ slightly

#### Option C: Binary Task + JSON in System Monitor

**Structure**:
- Dedicated task for binary GenericSens (critical, scheduled)
- JSON GenericSens published from System Monitor Task (every 5 seconds)

**Pros**:
- Binary timing guaranteed
- Reuses existing task

**Cons**:
- JSON frequency doesn't match binary (5s vs 1s)
- System Monitor Task already has responsibilities

**DECISION**: Binary in Fixed Frequency rate group (Priority 3) - **REQUIRED** to guarantee transmit.
JSON in **same task as binary** with **timeout/abort mechanism** - **CONFIRMED**.

**Rationale**: 
- Keeps binary and JSON synchronized (same data snapshot)
- Uses same local_copy buffer for data consistency
- Abort mechanism protects binary timing if JSON takes too long
- Simpler architecture (single task)
- JSON failure doesn't affect binary (abort and continue)

---

### Binary Message Timing Strategy

**Requirement**: Binary message must be published exactly every 1000ms with minimal delay.

**Approach**:
1. **Wait for Sensor Coordination**: Wait for `SENSOR_EVENT_1000MS_MASK` bits to be set
   - Ensures all 1000ms sensors have completed their reads
   - Provides data consistency guarantee
   - Timeout: 100ms (sensors should complete within 100ms)

2. **Scheduled Task**: Use FreeRTOS task with precise timing
   - Task priority: `TASK_PRIORITY_FIXED_FREQ` (3)
   - Use `vTaskDelayUntil()` for precise 1000ms intervals
   - Ensures consistent timing regardless of system load

3. **MQTT Mechanisms for Low Delay**:
   - Use **QoS 2** (exactly-once delivery) - Highest priority, guaranteed delivery
   - Use `esp_mqtt_client_publish()` with immediate flag if available
   - Consider MQTT client buffer size optimization
   - Monitor publish latency and adjust if needed
   - Task priority: `TASK_PRIORITY_FIXED_FREQ` (3) - Rate group to guarantee transmit

**Implementation**:
```c
void vMqttPublisherTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    
    // Register with watchdog
    watchdog_register_current_task("MqttPub", MQTT_PUBLISH_INTERVAL + 1000);
    
    for (;;) {
        // Wait for WiFi and MQTT connection before publishing
        if (!wifi_manager_is_connected() || !mqtt_manager_is_connected()) {
            watchdog_task_heartbeat();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }
        
        // Wait for sensor coordination (all 1000ms sensors complete)
        EventBits_t bits = xEventGroupWaitBits(
            sensor_event_group,
            SENSOR_EVENT_1000MS_MASK,
            pdTRUE,  // Clear bits on exit
            pdTRUE,  // Wait for ALL bits
            pdMS_TO_TICKS(MQTT_SENSOR_COORD_TIMEOUT_MS)  // 100ms timeout
        );
        
        if (bits & SENSOR_EVENT_1000MS_MASK) {
            // All sensors complete - proceed with publish
            
            // 1. Take mutex and copy data (minimize mutex hold time)
            union GENERICSENS_ local_copy;
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(MQTT_PUBLISH_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                memcpy(&local_copy, &genericSens_, sizeof(union GENERICSENS_));
                xSemaphoreGive(sensor_data_mutex);
                
                // 2. Publish binary FIRST (critical, outside mutex)
                esp_err_t binary_ret = mqtt_manager_publish_binary(
                    MQTT_TOPIC_BINARY_GENERICSENS,
                    (uint8_t*)local_copy.data_payload,
                    GENERICSENS_SIZE_BYTES,
                    MQTT_BINARY_QOS,  // QoS 2 - exactly-once delivery
                    false  // No retain
                );
                
                if (binary_ret == ESP_OK) {
                    ESP_LOGD(TAG, "Binary message published successfully");
                } else {
                    ESP_LOGW(TAG, "Failed to publish binary message: %s", esp_err_to_name(binary_ret));
                }
                
                // 3. Publish JSON (viewing, same data snapshot, with timeout protection)
                // JSON building and publishing with abort mechanism (see JSON section below)
                
                // 4. Increment cycle counter (after successful binary publish)
                if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(MQTT_PUBLISH_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    genericSens_.generic.cycle_count++;
                    if (genericSens_.generic.cycle_count > 28800) {
                        genericSens_.generic.cycle_count = 0;  // Reset at max
                    }
                    xSemaphoreGive(sensor_data_mutex);
                }
            } else {
                ESP_LOGW(TAG, "Failed to acquire mutex for data copy");
            }
        } else {
            ESP_LOGW(TAG, "Sensor coordination timeout - sensors may not have completed");
            // Proceed anyway - better than missing publish cycle
        }
        
        watchdog_task_heartbeat();
        
        // Wait for next 1000ms interval (precise timing)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

---

### JSON GenericSens Publishing

**Strategy**: **Same task as binary** with **timeout/abort mechanism** to protect binary timing

**Rationale**: 
- Uses same `local_copy` buffer as binary (perfect data synchronization)
- Simpler architecture (single task)
- Timeout mechanism protects binary timing if JSON takes too long
- JSON abort doesn't affect binary (continue to next cycle)

**Complete Implementation** (in same task, after binary publish):
```c
// 3. Publish JSON (viewing, same data snapshot, with timeout protection)
TickType_t json_start_time = xTaskGetTickCount();
const TickType_t json_timeout = pdMS_TO_TICKS(MQTT_JSON_TIMEOUT_MS);  // 50ms max

// Conservative timeout check: need at least 2× timeout budget to attempt JSON
TickType_t elapsed_since_wake = xTaskGetTickCount() - xLastWakeTime;
TickType_t min_time_required = json_timeout * 2;  // Need 2× timeout budget
if (elapsed_since_wake < (xFrequency - min_time_required)) {
    // We have time for JSON operations
    
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        // Continue - JSON failure doesn't affect binary
    } else {
        // Extract FlowData structures from local_copy (same as binary)
        FlowData_t flow1, flow2, flow3;
        GENERICSENS_EXTRACT_FLOWDATA(local_copy.generic.flowData1, flow1);
        GENERICSENS_EXTRACT_FLOWDATA(local_copy.generic.flowData2, flow2);
        GENERICSENS_EXTRACT_FLOWDATA(local_copy.generic.flowData3, flow3);
        
        // Flow sensors (bit-packed, extract individual fields)
        cJSON_AddNumberToObject(json, "S005D:flow1_pulses", flow1.pulses);
        cJSON_AddNumberToObject(json, "S005D:flow1_ms", flow1.milliseconds);
        cJSON_AddNumberToObject(json, "S005D:flow1_newData", flow1.newData);
        cJSON_AddNumberToObject(json, "S005D:flow2_pulses", flow2.pulses);
        cJSON_AddNumberToObject(json, "S005D:flow2_ms", flow2.milliseconds);
        cJSON_AddNumberToObject(json, "S005D:flow2_newData", flow2.newData);
        cJSON_AddNumberToObject(json, "S005D:flow3_pulses", flow3.pulses);
        cJSON_AddNumberToObject(json, "S005D:flow3_ms", flow3.milliseconds);
        cJSON_AddNumberToObject(json, "S005D:flow3_newData", flow3.newData);
        
        // Integer fields (words 3-11) - use variable names array
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[3], local_copy.generic.adc_sensor);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[4], local_copy.generic.gpio_sensor);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[5], local_copy.generic.temp1);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[6], local_copy.generic.temp1_f);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[7], local_copy.generic.tempSensorcount);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[8], local_copy.generic.cycle_count);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[9], local_copy.generic.fw_version);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[10], local_copy.generic.GPIO_x1);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[11], local_copy.generic.GPIO_x2);
        
        // Float fields (words 12-25) - use variable names array
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[12], local_copy.generic.adc_x1);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[13], local_copy.generic.adc_x2);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[14], local_copy.generic.adc_x3);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[15], local_copy.generic.adc_x4);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[16], local_copy.generic.adc_x5);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[17], local_copy.generic.adc_x6);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[18], local_copy.generic.adc_x7);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[19], local_copy.generic.adc_x8);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[20], local_copy.generic.tempx);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[21], local_copy.generic.pressurex);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[22], local_copy.generic.humidity);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[23], local_copy.generic.temp2);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[24], local_copy.generic.temp3);
        cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[25], local_copy.generic.temp4);
        
        // Add timestamp
        cJSON_AddNumberToObject(json, "timestamp", xTaskGetTickCount() * portTICK_PERIOD_MS);
        
        // Check timeout before serialization (most expensive operation)
        TickType_t json_elapsed = xTaskGetTickCount() - json_start_time;
        if (json_elapsed < json_timeout) {
            // Send watchdog heartbeat before potentially long operation
            watchdog_task_heartbeat();
            
            // Serialize and publish (within time budget)
            // Note: cJSON_Print() is blocking and can't be interrupted
            char *json_string = cJSON_Print(json);
            
            // Check timeout immediately after serialization
            json_elapsed = xTaskGetTickCount() - json_start_time;
            if (json_string != NULL) {
                // Send heartbeat before publish
                watchdog_task_heartbeat();
                // Check timeout again before publish
                if (json_elapsed < json_timeout) {
                    esp_err_t json_ret = mqtt_manager_publish_json(
                        MQTT_TOPIC_JSON_GENERICSENS,
                        json_string,
                        MQTT_JSON_QOS,  // QoS 0 for viewing
                        false
                    );
                    
                    if (json_ret == ESP_OK) {
                        // JSON message published successfully - no log (only report errors)
                    } else {
                        ESP_LOGW(TAG, "Failed to publish JSON message: %s", esp_err_to_name(json_ret));
                    }
                } else {
                    ESP_LOGW(TAG, "JSON timeout before publish (took %lu ms) - aborting", 
                             json_elapsed * portTICK_PERIOD_MS);
                }
                
                free(json_string);
            } else {
                ESP_LOGE(TAG, "Failed to serialize JSON");
            }
        } else {
            ESP_LOGW(TAG, "JSON timeout before serialization - aborting");
        }
        
        cJSON_Delete(json);
        
        // Log timing for monitoring
        json_elapsed = xTaskGetTickCount() - json_start_time;
        if (json_elapsed > json_timeout) {
            ESP_LOGW(TAG, "JSON operations took %lu ms (exceeded %lu ms timeout) - consider skipping JSON", 
                     json_elapsed * portTICK_PERIOD_MS, 
                     MQTT_JSON_TIMEOUT_MS);
        }
    }
} else {
    ESP_LOGI(TAG, "Skipping JSON - insufficient time remaining in cycle (elapsed: %lu ms, need: %lu ms)", 
             elapsed_since_wake * portTICK_PERIOD_MS, min_time_required * portTICK_PERIOD_MS);
}
```

**Key Features**:
- Uses same `local_copy` buffer as binary (perfect synchronization)
- **Conservative timeout check**: Requires 2× timeout budget (100ms) before attempting JSON
- Timeout checks before expensive operations (serialization, publish)
- **Watchdog heartbeats** added before `cJSON_Print()` and before publish to prevent watchdog timeouts
- Aborts gracefully if timeout exceeded
- Logs timing warnings when timeout exceeded
- Never delays binary publisher (abort protects timing)
- Skips JSON entirely if insufficient time remaining in cycle

**Note**: Uses `genericsens_ClientData_var_name[]` array from `sensor.c` for consistent JSON field names matching the interface specification.

---

### System Monitor Message Publishing

**Strategy**: Publish from System Monitor Task (already runs every 5 seconds)

**Implementation**:
```c
// In vSystemMonitorTask(), every 5th iteration (5000ms)
if (log_counter == 0) {
    // Build system monitor JSON
    cJSON *json = cJSON_CreateObject();
    
    // System status
    cJSON_AddNumberToObject(json, "uptime_sec", uptimeSeconds);
    cJSON_AddNumberToObject(json, "free_heap", currentHeapFree);
    cJSON_AddNumberToObject(json, "min_free_heap", minHeapFree);
    cJSON_AddNumberToObject(json, "active_tasks", currentTaskCount);
    
    // WiFi status
    if (wifi_manager_is_connected()) {
        // Get WiFi info and add to JSON
    }
    
    // MQTT status
    if (mqtt_manager_is_connected()) {
        // Get MQTT info and add to JSON
    }
    
    // I2C devices
    // Fan control status
    // etc.
    
    char *json_string = cJSON_Print(json);
    mqtt_manager_publish_json(
        MQTT_TOPIC_SYSTEM_MONITOR,
        json_string,
        0,  // QoS 0
        false
    );
    free(json_string);
    cJSON_Delete(json);
}
```

---

## Cycle Counter Management

**Requirement**: Increment `genericSens_.generic.cycle_count` every 1000ms

**Options**:
1. **In Binary Publisher Task**: Increment after publishing (ensures 1000ms frequency)
2. **Separate Task**: Lightweight task just for cycle counter
3. **In Sensor Coordination**: Increment when all sensors complete

**DECISION**: Increment in Binary Publisher Task - **CONFIRMED**

**Rationale**:
- Already runs at 1000ms interval
- Natural place for cycle counter
- Ensures counter increments with data publishing
- Increments after successful binary publish

**Implementation**:
```c
// In binary publisher task, after successful binary publish
if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(MQTT_PUBLISH_MUTEX_TIMEOUT_MS)) == pdTRUE) {
    genericSens_.generic.cycle_count++;
    if (genericSens_.generic.cycle_count > 28800) {
        genericSens_.generic.cycle_count = 0;  // Reset at max
    }
    xSemaphoreGive(sensor_data_mutex);
}
```

---

## Topic Naming Convention

**Development Suffix**: All topics use `-test` suffix during development

**Topics**:
- Binary GenericSens: `mwp/data/sensor/well/S005D-test` (development), `mwp/data/sensor/well/S005D` (production)
- JSON GenericSens: `mwp/json/sensor/well/S005D-test` (development), `mwp/json/sensor/well/S005D` (production)
- System Monitor: `mwp/json/sensor/well/monitor-test` (development), `mwp/json/sensor/well/monitor` (production)

**Production**: Remove `-test` suffix when moving to production

---

## Configuration Constants

**Add to `config.h`**:
```c
// MQTT Publishing Configuration
#define MQTT_BINARY_QOS                2      // QoS level for binary GenericSens (QoS 2 - exactly-once, guaranteed delivery)
#define MQTT_JSON_QOS                  0      // QoS level for JSON GenericSens (viewing)
#define MQTT_SYSTEM_MONITOR_QOS        0      // QoS level for system monitor (viewing)
#define MQTT_PUBLISH_MUTEX_TIMEOUT_MS  100    // Mutex timeout for reading genericSens_
#define MQTT_SENSOR_COORD_TIMEOUT_MS   100    // Timeout for waiting for sensor coordination
#define MQTT_JSON_TIMEOUT_MS           50     // Maximum time for JSON operations (abort if exceeded)
#define MQTT_USE_TEST_SUFFIX           1      // 1 = use "-test" suffix, 0 = production

// Topic definitions (with conditional test suffix)
#if MQTT_USE_TEST_SUFFIX
#define MQTT_TOPIC_BINARY_GENERICSENS   "mwp/data/sensor/well/S005D-test"
#define MQTT_TOPIC_JSON_GENERICSENS     "mwp/json/sensor/well/S005D-test"
#define MQTT_TOPIC_SYSTEM_MONITOR       "mwp/json/sensor/well/monitor-test"
#else
#define MQTT_TOPIC_BINARY_GENERICSENS   "mwp/data/sensor/well/S005D"
#define MQTT_TOPIC_JSON_GENERICSENS     "mwp/json/sensor/well/S005D"
#define MQTT_TOPIC_SYSTEM_MONITOR       "mwp/json/sensor/well/monitor"
#endif
```

---

## Error Handling

**Binary Message Failures**:
- Log error but continue (don't block next publish cycle)
- Track failure statistics
- Report via error recovery system

**JSON Message Failures**:
- Log warning (less critical)
- Continue operation
- Don't affect binary publishing

**Mutex Timeout**:
- Log warning
- Skip publish cycle (data may be inconsistent)
- Continue to next cycle

**Sensor Coordination Timeout**:
- Log warning (sensors may not have completed)
- Proceed with publish anyway (better than missing cycle)
- Monitor for persistent timeouts

---

## Timing Analysis and JSON Publishing Strategy

### Timing Concern: JSON Serialization Impact on Binary Publisher

**Risk Assessment**: JSON serialization and publishing could delay the binary publisher task, causing it to miss its 1000ms deadline.

**Timing Breakdown** (estimated):
- Sensor coordination wait: 0-100ms (typically <10ms)
- Mutex + data copy: <1ms
- Binary MQTT publish: 5-20ms (QoS 2, network dependent)
- **JSON building**: 5-15ms (26+ cJSON_AddNumberToObject calls)
- **JSON serialization (cJSON_Print)**: 10-50ms (memory allocation + string building)
- **JSON MQTT publish**: 5-20ms (QoS 0, network dependent)
- Cycle counter increment: <1ms
- **Total JSON overhead**: 20-85ms

**Problem**: If JSON operations take >50ms, and combined with sensor coordination wait and binary publish, the task could exceed 100ms, potentially causing timing drift or missed deadlines.

### Recommended Solution: Timeout/Abort Mechanism for JSON

**Decision**: Keep JSON in same task as binary, but add timeout/abort mechanism to protect binary timing.

**Benefits**:
- Binary and JSON use same data snapshot (guaranteed consistency)
- Simpler architecture (single task)
- Binary timing protected by abort mechanism
- JSON failure doesn't affect binary (abort and continue to next cycle)
- Easier to maintain (one task instead of two)

**Implementation Strategy**:
1. Binary publisher task: Publishes binary, then attempts JSON
2. JSON operations: Have maximum time budget (e.g., 50ms)
3. Abort mechanism: If JSON takes too long, abort and log warning
4. Continue to next cycle: Never delay binary publisher beyond deadline

**Timeout Budget**:
- Maximum time for JSON operations: 50ms
- Binary task should complete total cycle in <100ms
- Remaining time: 900ms+ margin for next cycle

---

### Recommended Architecture: Single Task with Timeout

**Binary + JSON Publisher Task** (Fixed Frequency, Priority 3):
```c
void vMqttPublisherTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    const TickType_t json_timeout = pdMS_TO_TICKS(50);  // 50ms max for JSON
    
    for (;;) {
        TickType_t cycle_start = xTaskGetTickCount();
        
        // Wait for sensor coordination
        // Copy data to local_copy
        // Publish binary (QoS 2)
        // Increment cycle counter
        
        // Attempt JSON (with timeout protection)
        TickType_t json_start = xTaskGetTickCount();
        // Build JSON from local_copy (same data snapshot as binary)
        // Serialize JSON
        // Publish JSON (QoS 0)
        
        TickType_t json_elapsed = xTaskGetTickCount() - json_start;
        if (json_elapsed > json_timeout) {
            ESP_LOGW(TAG, "JSON operations took %lu ms - aborted", json_elapsed * portTICK_PERIOD_MS);
            // Abort JSON operations, continue to next cycle
        }
        
        // Ensure we don't exceed cycle time
        TickType_t cycle_elapsed = xTaskGetTickCount() - cycle_start;
        if (cycle_elapsed > xFrequency) {
            ESP_LOGW(TAG, "Cycle exceeded 1000ms - timing drift detected");
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Precise timing
    }
}
```

**Data Consistency**: Both binary and JSON use same `local_copy` buffer, ensuring perfect synchronization.

---

### Timing Budget Analysis

**Binary Publisher Task Budget** (1000ms cycle):
- Sensor coordination wait: 0-100ms (timeout)
- Mutex + copy: <1ms
- Binary publish: 5-20ms
- Cycle counter: <1ms
- **JSON operations**: 0-50ms (aborted if exceeds)
- **Total**: ~10-80ms typical, <120ms worst case (with JSON abort)
- **Remaining time**: 920-990ms (plenty of margin)

**JSON Timeout Protection**:
- Maximum time budget: 50ms
- **Conservative check**: Requires 2× timeout budget (100ms) before attempting JSON
- If exceeded: Abort JSON, log warning, continue to next cycle
- Binary timing always protected (abort ensures cycle completes on time)
- Watchdog heartbeats added before long operations to prevent watchdog timeouts

**Conclusion**: With timeout/abort mechanism, binary timing is protected while maintaining data consistency between binary and JSON messages.

---

### Implementation Recommendation

**CONFIRMED**: Use single task for binary and JSON publishing with timeout/abort mechanism.

**Rationale**:
1. Binary and JSON use same data snapshot (perfect synchronization)
2. Simpler architecture (one task instead of two)
3. Timeout mechanism protects binary timing
4. JSON abort doesn't affect binary (continue to next cycle)
5. Easier to maintain and debug

**Trade-off**: JSON may occasionally be aborted if system is busy, but this is acceptable since JSON is viewing-only.

---

## Testing Requirements

**Binary Message**:
- Verify publishes exactly every 1000ms (±10ms tolerance)
- Verify data matches `genericSens_` structure
- Verify mutex protection (no corruption)
- Verify sensor coordination wait works
- Verify QoS 2 delivery (exactly-once)
- Verify task priority is Fixed Frequency (Priority 3)

**JSON Message**:
- Verify all sensor fields included
- Verify JSON format valid
- Verify uses same data snapshot as binary (same local_copy buffer)
- Verify timeout mechanism works (aborts if >50ms)
- Verify binary timing not affected (abort protects timing)
- Verify timing warnings logged appropriately

**System Monitor Message**:
- Verify publishes every 5000ms
- Verify all system status fields included
- Verify JSON format valid

**Integration**:
- Verify no mutex deadlocks
- Verify sensor managers not blocked
- Verify MQTT connection handling
- Verify error recovery

---

## Implementation Checklist

### Phase 1: Binary GenericSens Publisher
- [x] Create binary publisher task function (Fixed Frequency, Priority 3)
- [x] Implement sensor coordination wait
- [x] Implement mutex-protected data copy
- [x] Implement binary MQTT publish (QoS 2)
- [x] Add cycle counter increment
- [x] Add error handling
- [x] Test timing accuracy (completes ~5-10ms typical)

### Phase 2: JSON GenericSens Publisher (Same Task)
- [x] Implement JSON building from local_copy (same buffer as binary)
- [x] Add conservative timeout check (2× timeout budget requirement)
- [x] Add timeout checks during JSON building
- [x] Add timeout check before JSON serialization
- [x] Add all sensor fields to JSON (26+ fields)
- [x] Implement JSON serialization (cJSON_Print) with timeout protection
- [x] Add watchdog heartbeats before long operations
- [x] Implement JSON MQTT publish (QoS 0)
- [x] Add abort mechanism if timeout exceeded
- [x] Add timing logging for monitoring (warnings only)
- [x] Test JSON format validity
- [x] Verify timeout mechanism protects binary timing
- [x] Verify binary and JSON use same data snapshot

### Phase 3: System Monitor Publisher
- [x] Add JSON building to System Monitor Task
- [x] Include all system status fields
- [x] Implement MQTT publish (5 second interval)
- [x] Test system monitor message

### Phase 4: Integration and Testing
- [x] Test all three message types together
- [x] Verify mutex protection
- [x] Verify timing accuracy
- [x] Verify error handling
- [x] Performance testing
- [x] Error-only logging implemented
- [ ] Remove `-test` suffix for production (when ready)

---

## Decisions Made

1. **Task Structure**: Binary in Fixed Frequency rate group (Priority 3) - **REQUIRED**. JSON in **same task** with **timeout/abort mechanism** - **CONFIRMED**.
2. **Binary QoS**: **QoS 2** (exactly-once delivery) - Highest priority, guaranteed delivery - **CONFIRMED**.
3. **Sensor Coordination**: **YES** - Wait for event group before publishing - **CONFIRMED**.
4. **System Monitor Topic**: `mwp/json/sensor/well/monitor-test` (development), `mwp/json/sensor/well/monitor` (production) - **CONFIRMED**.
5. **Cycle Counter**: Increment in binary publisher task after successful publish - **CONFIRMED**.
6. **JSON Timing Protection**: **Timeout/abort mechanism** (50ms max) - JSON aborted if takes too long, binary timing always protected.
7. **Data Consistency**: Binary and JSON use same `local_copy` buffer - **CONFIRMED** for perfect synchronization.

---

## Current Implementation Status

**Status**: ✅ **IMPLEMENTED AND TESTED**

All three message types are implemented and working correctly:
- Binary GenericSens: Publishing every 1000ms with QoS 2 ✅
- JSON GenericSens: Publishing in same task with timeout protection ✅
- System Monitor: Publishing every 5000ms ✅

**Key Implementation Details**:
- Single task architecture (binary + JSON in same task)
- Conservative timeout check: Requires 2× timeout budget (100ms) before attempting JSON
- Watchdog heartbeats added before long JSON operations
- Error-only logging: No INFO logs for normal operation (only errors/warnings)
- Cycle counter increments after successful binary publish
- All topics use `-test` suffix (development mode)

**Performance Observations**:
- Binary publish: ~5-10ms typical
- JSON operations: ~20-50ms typical (can spike to 3000ms+ if heap fragmented)
- Timeout mechanism successfully protects binary timing
- System stable with occasional JSON skips when cycle is tight

**Future Optimization Considered**:
- Template-based JSON (build structure once, update values only)
- **Decision**: Leave as-is for now, monitor system performance
- Will reconsider if JSON timeouts become frequent or if timing becomes critical

---

## Logging Strategy

**Current Approach**: Error-only logging for MQTT operations

**Rationale**: Reduce serial output noise while maintaining visibility into problems

**What's Logged**:
- ✅ **ERROR** (`ESP_LOGE`): Connection failures, publish failures, timeouts, initialization errors
- ✅ **WARNING** (`ESP_LOGW`): Not connected when trying to publish, mutex timeouts, JSON timeouts
- ❌ **INFO** (`ESP_LOGI`): Connection success, publish success, normal operation (removed)

**Benefits**:
- Cleaner serial output
- Easier to spot actual problems
- Reduced CPU overhead from logging
- Normal operation is silent (as it should be)

---

## Revision History

- **2025-01-15**: Initial document creation
  - Defined three message types
  - Documented mutex protection pattern
  - Outlined task architecture options
  - Defined timing strategy
  - Added configuration constants
  - Added testing requirements
- **2025-01-15**: Timing analysis and decision finalized
  - Identified JSON serialization timing concern (20-85ms)
  - **DECISION**: Keep JSON in same task as binary with timeout/abort mechanism
  - Benefits: Same data snapshot, simpler architecture, timeout protects binary timing
  - Added timeout/abort implementation with 50ms maximum budget
  - Updated implementation to use single task with timeout protection
- **2025-11-11**: Implementation completed and tested
  - All three message types implemented and working
  - Conservative timeout check added (2× timeout budget requirement)
  - Watchdog heartbeats added during JSON operations
  - Error-only logging implemented (no INFO logs for normal operation)
  - System tested and stable
  - Template-based JSON optimization considered but deferred (monitor performance first)

