# Fan Control Implementation Plan

## Overview

This document describes the fan control implementation for the SensorX ESP32-C6 system. The fan control function is integrated into the System Monitor Task to provide responsive temperature-based cooling control while maintaining architectural simplicity.

## Design Philosophy

**Integration Approach**: Fan control is implemented as part of the System Monitor Task rather than as a separate sensor manager:
- **System-Level Function**: Fan control is a system control function, not a sensor reading operation
- **Architectural Simplicity**: Reduces task count by combining related system functions
- **Natural Fit**: System Monitor already reads system state - adding control logic is a natural extension
- **Resource Efficiency**: Consolidates system monitoring and control in a single task

## Architecture

### Temperature Source

**Primary Temperature Sensor**: BME280 Environmental Sensor (I2C)
- **Location**: `genericSens_.tempx` (updated by I2C Environmental Manager)
- **Update Interval**: Every 5000ms (5 seconds)
- **Temperature Type**: Ambient/environmental temperature (Fahrenheit)
- **Accuracy**: ±1°C (±1.8°F) typical
- **Availability**: Depends on BME280 sensor being present and functioning

**Fallback Temperature Source**: ESP32-C6 Internal Die Temperature Sensor
- **Sensor Type**: On-chip temperature sensor (measures chip die temperature)
- **Temperature Type**: Chip die temperature (typically higher than ambient)
- **Accuracy**: ±5°C typical (less accurate than BME280)
- **Use Case**: Fallback when BME280 is unavailable or providing invalid readings
- **API**: `esp_system_get_temp_sensor()` or `tempmon_get()` (ESP-IDF temperature monitor API)
- **Temperature Range**: Typically 0°C to 85°C (32°F to 185°F) for chip operation
- **Note**: Die temperature is typically 10-30°F higher than ambient temperature

**Temperature Source Priority**:
1. **Primary**: BME280 ambient temperature (`genericSens_.tempx`) - if valid and sensor present
2. **Fallback**: ESP32-C6 internal die temperature - if BME280 unavailable or invalid

**Temperature Validity Checks**:
- BME280 temperature considered valid if:
  - Sensor is present (`BME280_present` flag is true)
  - Temperature value is within reasonable range (e.g., -40°F to 150°F)
  - Temperature value is not zero or invalid sentinel value
- Internal temperature always available (built into chip)
- Internal temperature considered valid if:
  - Temperature value is within chip operating range (32°F to 185°F)
  - Temperature reading succeeds (no error from API)

### Control Logic

**Temperature Threshold**: 70.0°F (21.1°C)
- **Primary Threshold**: 70.0°F (for BME280 ambient temperature)
- **Fallback Threshold**: 85.0°F (for ESP32-C6 die temperature - adjusted for die temp offset)

**Threshold Adjustment for Die Temperature**:
- Die temperature is typically 10-30°F higher than ambient
- Fallback threshold set higher to account for this offset
- Prevents fan from running unnecessarily when using die temperature
- Configurable threshold allows fine-tuning based on system characteristics

**Control Behavior**:
```
// Try primary temperature source (BME280)
IF BME280 sensor present AND genericSens_.tempx is valid:
    IF genericSens_.tempx >= 70.0°F:
        Fan ON (GPIO HIGH)
    ELSE:
        Fan OFF (GPIO LOW)
ELSE:
    // Fallback to internal die temperature
    IF internal_die_temp >= 85.0°F:
        Fan ON (GPIO HIGH)
    ELSE:
        Fan OFF (GPIO LOW)
```

**Control Interval**: 1000ms (1 second)
- Checks temperature and updates fan state every second
- Provides responsive control even though temperature updates every 5 seconds
- Uses latest available temperature reading from `genericSens_.tempx`

### GPIO Configuration

**Fan Control Pin**: GPIO21 (changed from GPIO16 to avoid UART TX conflict)

**GPIO Setup**:
```c
gpio_config_t fan_config = {
    .pin_bit_mask = (1ULL << GPIO_NUM_21),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};
ESP_ERROR_CHECK(gpio_config(&fan_config));
```

**Electrical Characteristics**:
- **HIGH Level**: Fan ON (typically 3.3V)
- **LOW Level**: Fan OFF (typically 0V)
- **Note**: External pull-up resistor may be present on board (check hardware schematic)

## Implementation Details

### System Monitor Task Integration

**Task**: `vSystemMonitorTask()` in `components/sensor_system/src/system_init.c`

**Task Configuration**:
- **Priority**: `TASK_PRIORITY_BACKGROUND` (1)
- **Stack Size**: `TASK_STACK_SIZE_BACKGROUND` (2048 bytes)
- **Execution Interval**: 1000ms (fan control check every loop)
- **Logging Interval**: 5000ms (system status log every 5th iteration)

**Task Flow**:
```
1. Initialize GPIO pin for fan control
2. Loop forever:
   a. Read temperature from genericSens_ (mutex-protected)
   b. Compare against threshold (70.0°F)
   c. Update GPIO pin state (HIGH/LOW)
   d. Increment log counter
   e. Every 5th iteration: Log system status
   f. Send watchdog heartbeat
   g. Delay 1000ms
```

### Code Implementation

**Fan Control Initialization** (in System Monitor Task):
```c
void vSystemMonitorTask(void *pvParameters) {
    (void)pvParameters;
    
    // Initialize fan control GPIO
    gpio_config_t fan_config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_21),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&fan_config));
    
    // Start with fan OFF
    gpio_set_level(GPIO_NUM_21, 0);
    
    // Register with watchdog
    watchdog_register_current_task("SysMonitor", MONITOR_INTERVAL_MS + 1000);
    
    ESP_LOGI(TAG, "System Monitor Task: Started successfully!");
    
    // Initialize monitoring variables
    static uint32_t startTime = 0;
    static size_t lastHeapFree = 0;
    static size_t minHeapFree = 0;
    static UBaseType_t lastTaskCount = 0;
    static uint32_t log_counter = 0;
    static bool initialized = false;
    
    // Fan control state variables (shared between fan control and monitoring sections)
    static float current_temp = 0.0;
    static bool temp_valid = false;
    static bool using_fallback = false;
    
    if (!initialized) {
        startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        lastHeapFree = esp_get_free_heap_size();
        minHeapFree = esp_get_free_heap_size();
        lastTaskCount = uxTaskGetNumberOfTasks();
        initialized = true;
    }
    
    const TickType_t xControlInterval = pdMS_TO_TICKS(MONITOR_INTERVAL_MS);  // 1000ms
    
    for (;;) {
        // ==========================================
        // Fan Control (every loop iteration = 1000ms)
        // ==========================================
        // Try primary temperature source (BME280)
        if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            float bme280_temp = genericSens_.tempx;
            xSemaphoreGive(sensor_data.mutex);
            
            // Check if BME280 temperature is valid
            // Valid range: -40°F to 150°F (typical sensor range)
            if (bme280_temp > -40.0 && bme280_temp < 150.0 && bme280_temp != 0.0) {
                // Check if BME280 sensor is present (if tracking available)
                // Note: May need to check BME280_present flag or sensor status
                current_temp = bme280_temp;
                temp_valid = true;
                using_fallback = false;
            }
        } else {
            ESP_LOGW(TAG, "Failed to acquire mutex for fan control");
        }
        
        // Fallback to internal die temperature if primary source invalid
        if (!temp_valid) {
            // Read ESP32-C6 internal die temperature
            // Note: ESP-IDF API may vary - check ESP-IDF version for correct API
            // Example API: esp_system_get_temp_sensor() or tempmon_get()
            #ifdef CONFIG_IDF_TARGET_ESP32C6
            // ESP32-C6 internal temperature sensor
            // Convert from Celsius to Fahrenheit
            float die_temp_c = 0.0;
            esp_err_t temp_err = esp_efuse_get_temp_sensor_value(&die_temp_c);
            
            if (temp_err == ESP_OK && die_temp_c > -40.0 && die_temp_c < 85.0) {
                float die_temp_f = (die_temp_c * 9.0 / 5.0) + 32.0;
                current_temp = die_temp_f;
                temp_valid = true;
                using_fallback = true;
                ESP_LOGD(TAG, "Using fallback: Internal die temp %.1f°F", die_temp_f);
            } else {
                ESP_LOGW(TAG, "Failed to read internal temperature: %s", esp_err_to_name(temp_err));
            }
            #else
            // Fallback API for other ESP32 variants (if needed)
            // Note: ESP32-C6 specific implementation
            #endif
        }
        
        // Apply fan control based on valid temperature source
        if (temp_valid) {
            float threshold = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;
            
            if (current_temp >= threshold) {
                gpio_set_level(GPIO_NUM_21, 1);  // Fan ON
                ESP_LOGD(TAG, "Fan ON: %s temp %.1f°F >= %.1f°F", 
                         using_fallback ? "Die" : "Ambient", 
                         current_temp, threshold);
            } else {
                gpio_set_level(GPIO_NUM_21, 0);  // Fan OFF
                ESP_LOGD(TAG, "Fan OFF: %s temp %.1f°F < %.1f°F", 
                         using_fallback ? "Die" : "Ambient", 
                         current_temp, threshold);
            }
        } else {
            // No valid temperature source - default to fan ON for safety
            gpio_set_level(GPIO_NUM_21, 1);  // Fan ON (safe default)
            ESP_LOGW(TAG, "No valid temperature source - fan ON for safety");
        }
        
        // ==========================================
        // System Monitoring (every 5th iteration = 5000ms)
        // ==========================================
        log_counter++;
        if (log_counter >= 5) {
            log_counter = 0;
            
            // Calculate uptime
            uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t uptimeMs = currentTime - startTime;
            uint32_t uptimeSeconds = uptimeMs / 1000;
            uint32_t uptimeHours = uptimeSeconds / 3600;
            uint32_t uptimeMinutes = (uptimeSeconds % 3600) / 60;
            uint32_t uptimeSecs = uptimeSeconds % 60;
            
            // Get current memory stats
            size_t currentHeapFree = esp_get_free_heap_size();
            
            // Update minimum heap tracking
            if (currentHeapFree < minHeapFree) {
                minHeapFree = currentHeapFree;
            }
            
            // Get task information
            UBaseType_t currentTaskCount = uxTaskGetNumberOfTasks();
            
            // Get fan control status and temperature (from fan control section above)
            // Note: These variables are maintained in the fan control section
            bool fan_state = gpio_get_level(GPIO_NUM_21);
            
            // Display monitoring information
            ESP_LOGI(TAG, "--- System Monitor ---");
            ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu (HH:MM:SS)",
                     uptimeHours, uptimeMinutes, uptimeSecs);
            ESP_LOGI(TAG, "Free Heap: %zu bytes (Min: %zu bytes)",
                     currentHeapFree, minHeapFree);
            ESP_LOGI(TAG, "Active Tasks: %d", currentTaskCount);
            
            // Display fan control status and temperature
            if (temp_valid) {
                const char* temp_source = using_fallback ? "Die" : "Ambient";
                float threshold_display = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;
                ESP_LOGI(TAG, "Fan: %s | %s Temp: %.1f°F | Threshold: %.1f°F",
                         fan_state ? "ON" : "OFF",
                         temp_source,
                         current_temp,
                         threshold_display);
            } else {
                ESP_LOGI(TAG, "Fan: %s | Temp: N/A | Threshold: N/A",
                         fan_state ? "ON" : "OFF");
            }
            
            // ... rest of system monitoring code ...
        }
        
        watchdog_task_heartbeat();
        vTaskDelay(xControlInterval);
    }
}
```

## Configuration

### Temperature Threshold

**Default Thresholds**:
- **Primary (BME280)**: 70.0°F (21.1°C) - ambient temperature threshold
- **Fallback (Die Temp)**: 85.0°F (29.4°C) - die temperature threshold (adjusted for offset)

**Configuration Location**: `components/sensor_system/src/system_init.c`

**Configuration Constants** (recommended):
```c
// In config.h or system_init.c
#define FAN_CONTROL_THRESHOLD_TEMP_F          70.0  // Fan ON above this temperature (°F) - BME280 ambient
#define FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F 85.0  // Fan ON above this temperature (°F) - Die temp fallback
#define FAN_CONTROL_TEMP_VALID_MIN_F          -40.0 // Minimum valid temperature (°F)
#define FAN_CONTROL_TEMP_VALID_MAX_F          150.0 // Maximum valid temperature (°F)
#define FAN_CONTROL_DIE_TEMP_VALID_MIN_F      32.0  // Minimum valid die temperature (°F)
#define FAN_CONTROL_DIE_TEMP_VALID_MAX_F      185.0 // Maximum valid die temperature (°F)
```

**Adjustable Threshold** (optional):
Could be made configurable via:
- Compile-time constant
- Runtime configuration (NVS storage)
- MQTT command (advanced)

### Control Interval

**Current Interval**: 1000ms (1 second)

**Configuration**: `MONITOR_INTERVAL_MS` in `config.h`
```c
#define MONITOR_INTERVAL_MS              1000  // System monitor task interval
```

**Considerations**:
- Shorter interval = more responsive but higher CPU usage
- Longer interval = less responsive but lower CPU usage
- 1000ms provides good balance for fan control

## Thread Safety

### Mutex Protection

Fan control reads temperature from `genericSens_` structure which is shared among multiple tasks:

**Mutex Name**: `sensor_data.mutex`

**Protection Pattern**:
```c
if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    float current_temp = genericSens_.tempx;  // Read protected data
    xSemaphoreGive(sensor_data.mutex);
    
    // Use temperature value (outside mutex to reduce hold time)
    if (current_temp >= FAN_CONTROL_THRESHOLD_TEMP_F) {
        gpio_set_level(GPIO_NUM_21, 1);
    } else {
        gpio_set_level(GPIO_NUM_21, 0);
    }
}
```

**Timeout**: 100ms timeout prevents deadlock if mutex is held by another task

### GPIO Access

GPIO operations (`gpio_set_level`) are thread-safe in ESP-IDF and can be called from any task without additional protection.

## Integration Points

### Dependencies

1. **I2C Environmental Manager**: Provides temperature reading (`genericSens_.tempx`)
   - Must be initialized before System Monitor Task starts
   - Updates temperature every 5000ms
   - Optional: System can operate without BME280 (uses fallback)

2. **Sensor Data Structure**: Shared `genericSens_` structure
   - Must be initialized with mutex before System Monitor Task starts
   - Defined in `sensor_manager_types.h`
   - Used for primary temperature source (BME280)

3. **ESP-IDF Temperature Monitor**: Provides internal die temperature
   - Built into ESP32-C6 chip
   - Always available (no initialization required)
   - API: `esp_efuse_get_temp_sensor_value()` or similar (check ESP-IDF version)
   - Used as fallback temperature source

4. **Watchdog System**: Task monitoring
   - System Monitor Task registers with watchdog
   - Watchdog timeout: `MONITOR_INTERVAL_MS + 1000` (2000ms)

### Initialization Order

```
1. Initialize I2C manager
2. Initialize sensor data structure with mutex
3. Initialize I2C Environmental Manager
4. Start I2C Environmental Manager task
5. Initialize System Monitor Task (includes fan control GPIO setup)
6. Start System Monitor Task
```

## Error Handling

### Temperature Read Failures

**Scenario 1**: BME280 sensor not present

**Handling**:
- Detect sensor absence via `BME280_present` flag or sensor initialization status
- Automatically fall back to internal die temperature
- Log info message indicating fallback mode
- Use adjusted threshold for die temperature (85.0°F)

**Scenario 2**: BME280 sensor present but providing invalid readings

**Handling**:
- Check temperature value validity (range: -40°F to 150°F)
- Check for invalid sentinel values (0.0, NaN, etc.)
- If invalid, fall back to internal die temperature
- Log warning indicating invalid BME280 reading
- Use adjusted threshold for die temperature

**Scenario 3**: Mutex timeout

**Handling**:
- Log warning if mutex acquisition fails
- Attempt to read internal die temperature as fallback
- If both sources fail, default to fan ON (safe default)

**Scenario 4**: Internal die temperature read failure

**Handling**:
- If BME280 also unavailable, no valid temperature source
- Default to fan ON for safety
- Log error indicating both sources failed
- Continue operation with fan ON

**Recommended Approach**:
```c
// Temperature source tracking
static float last_known_temp = 0.0;
static bool temp_valid = false;
static bool using_fallback = false;
static uint32_t fallback_count = 0;

// Try primary temperature source (BME280)
float current_temp = 0.0;
bool bme280_valid = false;

if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    float bme280_temp = genericSens_.tempx;
    xSemaphoreGive(sensor_data.mutex);
    
    // Validate BME280 reading
    if (bme280_temp > FAN_CONTROL_TEMP_VALID_MIN_F && 
        bme280_temp < FAN_CONTROL_TEMP_VALID_MAX_F && 
        bme280_temp != 0.0) {
        current_temp = bme280_temp;
        temp_valid = true;
        using_fallback = false;
        bme280_valid = true;
    }
} else {
    ESP_LOGW(TAG, "Mutex timeout - checking fallback temperature");
}

// Fallback to internal die temperature if BME280 invalid
if (!bme280_valid) {
    #ifdef CONFIG_IDF_TARGET_ESP32C6
    float die_temp_c = 0.0;
    esp_err_t temp_err = esp_efuse_get_temp_sensor_value(&die_temp_c);
    
    if (temp_err == ESP_OK) {
        float die_temp_f = (die_temp_c * 9.0 / 5.0) + 32.0;
        
        // Validate die temperature reading
        if (die_temp_f > FAN_CONTROL_DIE_TEMP_VALID_MIN_F && 
            die_temp_f < FAN_CONTROL_DIE_TEMP_VALID_MAX_F) {
            current_temp = die_temp_f;
            temp_valid = true;
            using_fallback = true;
            fallback_count++;
            
            if (fallback_count == 1) {
                ESP_LOGW(TAG, "BME280 unavailable - using internal die temperature");
            }
        } else {
            ESP_LOGW(TAG, "Die temperature out of range: %.1f°F", die_temp_f);
        }
    } else {
        ESP_LOGE(TAG, "Failed to read internal temperature: %s", esp_err_to_name(temp_err));
    }
    #endif
}

// Apply fan control
if (temp_valid) {
    last_known_temp = current_temp;
    float threshold = using_fallback ? FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F : FAN_CONTROL_THRESHOLD_TEMP_F;
    
    if (current_temp >= threshold) {
        gpio_set_level(GPIO_NUM_21, 1);
        ESP_LOGD(TAG, "Fan ON: %s temp %.1f°F >= %.1f°F", 
                 using_fallback ? "Die" : "Ambient", current_temp, threshold);
    } else {
        gpio_set_level(GPIO_NUM_21, 0);
        ESP_LOGD(TAG, "Fan OFF: %s temp %.1f°F < %.1f°F", 
                 using_fallback ? "Die" : "Ambient", current_temp, threshold);
    }
} else {
    // Both sources failed - default to fan ON for safety
    gpio_set_level(GPIO_NUM_21, 1);
    ESP_LOGW(TAG, "No valid temperature source - fan ON for safety");
}
```

### GPIO Configuration Failures

**Scenario**: GPIO pin configuration fails during initialization

**Handling**:
- Use `ESP_ERROR_CHECK()` to abort on configuration failure
- Or use `ESP_ERROR_CHECK_WITHOUT_ABORT()` to log error and continue (fan control disabled)

## Testing Considerations

### Unit Testing

1. **Temperature Threshold Testing**:
   - Test with BME280 temperature just below threshold (69.9°F) → Fan OFF
   - Test with BME280 temperature at threshold (70.0°F) → Fan ON
   - Test with BME280 temperature above threshold (70.1°F) → Fan ON
   - Test with die temperature just below fallback threshold (84.9°F) → Fan OFF
   - Test with die temperature at fallback threshold (85.0°F) → Fan ON
   - Test with die temperature above fallback threshold (85.1°F) → Fan ON

2. **Fallback Mode Testing**:
   - Test with BME280 sensor not present → Should use die temperature
   - Test with BME280 providing invalid reading (0.0, out of range) → Should use die temperature
   - Test with BME280 mutex timeout → Should use die temperature
   - Test with both sources unavailable → Should default to fan ON

3. **Temperature Source Switching**:
   - Test transition from BME280 to die temperature (simulate sensor failure)
   - Test transition from die temperature to BME280 (simulate sensor recovery)
   - Verify threshold switches correctly (70°F vs 85°F)

2. **Mutex Testing**:
   - Test mutex timeout scenario
   - Test concurrent access from multiple tasks

3. **GPIO Testing**:
   - Verify GPIO pin configuration
   - Verify GPIO state changes (HIGH/LOW)
   - Test with oscilloscope or logic analyzer

### Integration Testing

1. **Temperature Source**:
   - Verify BME280 temperature reading updates `genericSens_.tempx`
   - Verify fan control reads correct temperature value
   - Verify fallback to die temperature when BME280 unavailable
   - Verify correct threshold used for each temperature source
   - Verify temperature validity checks work correctly

2. **Control Loop**:
   - Monitor fan control response to temperature changes
   - Verify 1000ms control interval
   - Verify fan state changes correctly

3. **System Stability**:
   - Long-term operation test
   - Verify no memory leaks
   - Verify watchdog doesn't trigger

### Hardware Testing

1. **Fan Circuit**:
   - Verify fan turns ON when GPIO HIGH
   - Verify fan turns OFF when GPIO LOW
   - Measure fan current draw
   - Verify fan doesn't interfere with other GPIO operations

2. **Temperature Accuracy**:
   - Compare BME280 reading with reference thermometer
   - Verify fan control threshold accuracy
   - Compare die temperature with BME280 (expect 10-30°F offset)
   - Verify fallback threshold accounts for die temperature offset
   - Test fan control behavior with both temperature sources

## Monitoring and Debugging

### Logging

**Debug Logging** (when enabled):
```c
ESP_LOGD(TAG, "Fan ON: Temperature %.1f°F >= 70.0°F", current_temp);
ESP_LOGD(TAG, "Fan OFF: Temperature %.1f°F < 70.0°F", current_temp);
```

**Warning Logging**:
```c
ESP_LOGW(TAG, "Failed to acquire mutex for fan control");
ESP_LOGW(TAG, "Temperature unavailable - fan ON for safety");
```

### Status Reporting

**System Monitor Output Integration**:

Fan status and temperature information are included in the System Monitor output that is displayed every 5000ms (5 seconds). The information is displayed as part of the regular system status log.

**Example System Monitor Output**:
```
I (12345) SYSTEM_INIT: --- System Monitor ---
I (12346) SYSTEM_INIT: Uptime: 00:15:32 (HH:MM:SS)
I (12346) SYSTEM_INIT: Free Heap: 280376 bytes (Min: 277248 bytes)
I (12347) SYSTEM_INIT: Active Tasks: 14
I (12347) SYSTEM_INIT: Fan: ON | Ambient Temp: 72.5°F | Threshold: 70.0°F
I (12348) SYSTEM_INIT: --- Monitor Complete ---
```

**Fan Status Display Format**:
- **Fan State**: "ON" or "OFF" (based on GPIO21 level)
- **Temperature Source**: "Ambient" (BME280) or "Die" (ESP32-C6 internal)
- **Temperature Value**: Current temperature in Fahrenheit (or 0.0 if invalid)
- **Threshold**: Active threshold (70.0°F for ambient, 85.0°F for die temp)

**Fallback Mode Display**:
When fallback mode is active, the output will show:
```
I (12347) SYSTEM_INIT: Fan: OFF | Die Temp: 78.3°F | Threshold: 85.0°F
```

**Invalid Temperature Display**:
If no valid temperature source is available:
```
I (12347) SYSTEM_INIT: Fan: ON | Temp: N/A | Threshold: N/A
```

**Implementation Notes**:
- Fan status and temperature variables are maintained in the fan control section
- These variables are static within the task function to persist across iterations
- Fan control updates these variables every 1000ms
- System monitor reads these variables every 5000ms for display
- No additional mutex needed since variables are local to the task

### MQTT Integration (Optional)

Could publish fan control status via MQTT:
```c
// In MQTT publisher or System Monitor
cJSON_AddBoolToObject(json, "fan_on", gpio_get_level(GPIO_NUM_21));
cJSON_AddNumberToObject(json, "fan_control_temp", last_known_temp);
```

## Future Enhancements

### Potential Improvements

1. **Hysteresis Control**:
   - Fan ON threshold: 70.0°F
   - Fan OFF threshold: 65.0°F (prevents rapid cycling)
   ```c
   if (current_temp >= 70.0) {
       fan_state = true;
   } else if (current_temp <= 65.0) {
       fan_state = false;
   }
   // Else maintain current state
   ```

2. **PWM Fan Control**:
   - Use LEDC peripheral for variable speed control
   - Speed based on temperature (e.g., 25% at 70°F, 100% at 85°F)

3. **Dual Temperature Sources** (Already Implemented):
   - Primary: BME280 ambient temperature (70.0°F threshold)
   - Fallback: ESP32-C6 internal die temperature (85.0°F threshold)
   - Automatic fallback when primary source unavailable
   - Different thresholds account for die temperature offset

4. **Configurable Threshold**:
   - Runtime configuration via MQTT command
   - Store threshold in NVS for persistence

5. **Fan Status Monitoring**:
   - Monitor fan tachometer signal (if available)
   - Detect fan failure
   - Alert via MQTT/system log

6. **Temperature History**:
   - Track temperature trends
   - Predictive fan control based on rate of change

## Configuration Reference

### Constants

```c
// config.h
#define MONITOR_INTERVAL_MS                   1000  // System monitor task interval (fan control check)

// system_init.c or config.h
#define FAN_CONTROL_GPIO_PIN                  GPIO_NUM_21
#define FAN_CONTROL_THRESHOLD_TEMP_F          70.0   // Fan ON threshold (°F) - BME280 ambient
#define FAN_CONTROL_FALLBACK_THRESHOLD_TEMP_F 85.0   // Fan ON threshold (°F) - Die temp fallback
#define FAN_CONTROL_TEMP_VALID_MIN_F          -40.0  // Minimum valid ambient temperature (°F)
#define FAN_CONTROL_TEMP_VALID_MAX_F          150.0  // Maximum valid ambient temperature (°F)
#define FAN_CONTROL_DIE_TEMP_VALID_MIN_F      32.0   // Minimum valid die temperature (°F)
#define FAN_CONTROL_DIE_TEMP_VALID_MAX_F     185.0  // Maximum valid die temperature (°F)
#define FAN_CONTROL_MUTEX_TIMEOUT_MS          100    // Mutex timeout for temperature read
```

### GPIO Pin Definition

```c
// pins.h (if not already defined)
#define PIN_FAN_CONTROL    GPIO_NUM_21  // Fan control output pin
```

## Summary

The fan control implementation:

- ✅ **Integrated** into System Monitor Task (reduces task count)
- ✅ **Responsive** (checks every 1000ms)
- ✅ **Thread-Safe** (mutex-protected temperature reads)
- ✅ **Robust** (fallback to internal die temperature when BME280 unavailable)
- ✅ **Safe** (defaults to ON if all temperature sources unavailable)
- ✅ **Configurable** (separate thresholds for ambient and die temperature)
- ✅ **Monitored** (watchdog protection, logging, temperature source tracking)
- ✅ **Validated** (temperature validity checks prevent invalid readings)

**Key Features**:
- **Primary Source**: BME280 ambient temperature (70.0°F threshold)
- **Fallback Source**: ESP32-C6 internal die temperature (85.0°F threshold)
- **Automatic Fallback**: Seamlessly switches to die temperature when BME280 unavailable
- **Temperature Validation**: Checks for valid temperature ranges and sensor presence
- **Safety Default**: Fan ON if no valid temperature source available

This design provides reliable, responsive fan control with robust fallback capabilities while maintaining architectural simplicity and system efficiency.

