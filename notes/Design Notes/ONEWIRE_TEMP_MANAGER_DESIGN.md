# One-Wire Temperature Sensor Manager Design

## Overview

This document describes the design approach for the One-Wire Temperature Manager in the SensorX ESP32-C6 system. This manager handles DS18B20 temperature sensors connected via a single One-Wire bus, implementing cooperative multitasking to ensure other tasks can execute during long sensor read operations.

## Design Philosophy

**Cooperative Multitasking**: The One-Wire protocol involves timing-sensitive operations that can take significant time (750ms per sensor typical). To prevent blocking other tasks of the same priority, the implementation yields CPU time between sensor reads using FreeRTOS `vTaskDelay()`.

**Complete Cycle Reading**: All sensors are read within a single 5000ms task cycle. Small yield points (10-20ms) between sensors allow other same-priority tasks brief execution windows while completing all reads within the allocated time.

**Priority Considerations**: Running at `TASK_PRIORITY_BACKGROUND` (1), the lowest priority ensures higher priority tasks (sensor acquisition, MQTT publishing) are never blocked.

## Hardware Overview

### One-Wire Protocol

**Protocol Characteristics**:
- Single GPIO pin for bidirectional communication
- Bit-banged protocol (software-driven timing)
- Pull-up resistor required (typically 4.7kΩ)
- Timing-sensitive operations with precise delays

**DS18B20 Temperature Sensor**:
- 12-bit resolution (0.0625°C resolution)
- Temperature range: -55°C to +125°C
- Conversion time: 750ms typical (12-bit resolution)
- Can be powered parasitically or with external VCC

### Bus Configuration

**Single Bus Architecture**:
- All DS18B20 sensors share one GPIO pin
- Each sensor has unique 64-bit ROM address
- Sensors are addressed individually using ROM codes
- Up to 4 sensors supported (based on `genericSens_` structure)

**GPIO Pin Selection**:
- Pin: Configurable (typically GPIO_NUM_6 or similar)
- Mode: Open-drain with external pull-up
- Shared with other sensors on same bus

## Task Design

### Task Configuration

**Priority**: `TASK_PRIORITY_BACKGROUND` (1) - Lowest priority
- Ensures higher priority tasks can always preempt
- Fan control, I2C sensors, MQTT publishing have higher priority

**Stack Size**: `TASK_STACK_SIZE_BACKGROUND` (2048 bytes)
- One-Wire protocol requires minimal stack
- Room for ROM code storage and conversion buffers

**Execution Interval**: 5000ms (5 seconds)
- All 4 sensors read per cycle
- Complete scan every 5 seconds
- Temperature updates are slow-changing, 5s interval provides good refresh rate

**Watchdog Timeout**: 6000ms (5000ms + 1000ms margin)
- Protects against hung One-Wire operations
- Allows up to 1 second overrun before watchdog trigger

### Task Lifecycle

```
1. Task initialization
   a. Initialize One-Wire GPIO pin
   b. Scan bus for DS18B20 devices
   c. Store ROM codes for discovered sensors
   d. Register with watchdog system

2. Main task loop:
   a. Read all sensors sequentially (0-3)
   b. For each sensor:
      - Request temperature conversion (non-blocking)
      - Wait for conversion (750ms)
      - Read temperature from scratchpad
      - Convert to Fahrenheit
      - Update genericSens_ structure (mutex-protected)
      - Small yield (10-20ms) between sensors
   c. Send watchdog heartbeat
   d. Delay for task interval (5000ms)
```

## Cooperative Multitasking Strategy

### Problem: Long Blocking Operations

**One-Wire Read Sequence**:
1. Reset pulse: ~480μs
2. Send ROM command: ~100μs
3. Send function command (CONVERT_T): ~100μs
4. **Wait for conversion**: ~750ms (the problem!)
5. Reset pulse: ~480μs
6. Send ROM command: ~100μs
7. Send function command (READ_SCRATCHPAD): ~100μs
8. Read 9 bytes: ~1ms
9. Verify CRC: ~1ms

**Total Time**: ~850ms per sensor, ~3.4 seconds for 4 sensors

### Solution: Yield Points

**Yield Strategy**: Insert `vTaskDelay()` calls between sensor operations to allow other tasks to execute.

**Yield Points**:
1. **Between sensor reads**: Small yield (10-20ms) after reading each sensor
2. **After all sensors read**: Main task delay (5000ms) until next cycle

**Yield Duration**:
- Minimal yields (10-20ms): Between sensors - just enough to allow other tasks brief execution
- No yield during conversion wait: Conversion happens in hardware, we wait synchronously
- Balance: Minimize yield overhead while ensuring other tasks can execute
- Total cycle time: ~3.4 seconds reading + ~60ms yields = ~3.46 seconds (well within 5000ms)

### Implementation Pattern

```c
void vOneWireTempManagerTask(void *pvParameters) {
    // Initialize One-Wire bus and scan for devices
    onewire_init(GPIO_NUM_6);
    onewire_scan_devices();  // Discover all DS18B20 sensors
    
    const TickType_t xTaskInterval = pdMS_TO_TICKS(5000);
    const TickType_t xYieldBetweenSensors = pdMS_TO_TICKS(20);  // Small yield between sensors
    
    for (;;) {
        uint32_t cycle_start = xTaskGetTickCount();
        
        // Read all sensors sequentially within one cycle
        for (uint8_t sensor_index = 0; sensor_index < num_sensors; sensor_index++) {
            uint32_t sensor_start = xTaskGetTickCount();
            
            // Step 1: Request temperature conversion
            onewire_reset();
            onewire_select(rom_codes[sensor_index]);
            onewire_write(0x44);  // CONVERT_T command
            // Conversion starts - sensor is now busy
            
            // Wait for conversion (750ms) - no yield here, conversion is hardware process
            vTaskDelay(pdMS_TO_TICKS(750));
            
            // Step 2: Read temperature from scratchpad
            onewire_reset();
            onewire_select(rom_codes[sensor_index]);
            onewire_write(0xBE);  // READ_SCRATCHPAD
            
            uint8_t scratchpad[9];
            for (int i = 0; i < 9; i++) {
                scratchpad[i] = onewire_read();
            }
            
            // Verify CRC
            if (onewire_crc8(scratchpad, 8) == scratchpad[8]) {
                // Convert to temperature
                int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
                float temp_celsius = raw_temp / 16.0;
                float temp_fahrenheit = (temp_celsius * 9.0 / 5.0) + 32.0;
                
                // Update genericSens_ (mutex-protected)
                if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    switch (sensor_index) {
                        case 0:
                            genericSens_.temp1 = (int)temp_fahrenheit;
                            memcpy(&genericSens_.temp1_f, &temp_fahrenheit, sizeof(float));
                            break;
                        case 1:
                            genericSens_.temp2 = temp_fahrenheit;
                            break;
                        case 2:
                            genericSens_.temp3 = temp_fahrenheit;
                            break;
                        case 3:
                            genericSens_.temp4 = temp_fahrenheit;
                            break;
                    }
                    xSemaphoreGive(sensor_data.mutex);
                }
            } else {
                ESP_LOGW(TAG, "CRC error reading sensor %d", sensor_index);
            }
            
            // Monitor per-sensor read time
            uint32_t sensor_elapsed = (xTaskGetTickCount() - sensor_start) * portTICK_PERIOD_MS;
            if (sensor_elapsed > 1000) {
                ESP_LOGW(TAG, "Sensor %d read took %lu ms", sensor_index, sensor_elapsed);
            }
            
            // YIELD POINT: Small yield between sensors (except after last sensor)
            // Allows other same-priority tasks brief execution window
            if (sensor_index < num_sensors - 1) {
                vTaskDelay(xYieldBetweenSensors);  // 20ms yield
            }
        }
        
        // Calculate total cycle time
        uint32_t cycle_elapsed = (xTaskGetTickCount() - cycle_start) * portTICK_PERIOD_MS;
        if (cycle_elapsed > 5000) {
            ESP_LOGW(TAG, "Complete cycle exceeded interval: %lu ms", cycle_elapsed);
        }
        
        // Send watchdog heartbeat
        watchdog_task_heartbeat();
        
        // YIELD POINT: Main task delay until next cycle
        // This is the primary yield - allows all other tasks to execute
        vTaskDelay(xTaskInterval);
    }
}
```

## Timing Analysis

### Without Yield Points

**Scenario**: Reading all 4 sensors sequentially without yielding

```
Time    Task Activity
0ms     Start sensor 0 conversion
750ms   Read sensor 0 temperature
850ms   Complete sensor 0 read
850ms   Start sensor 1 conversion
1600ms  Read sensor 1 temperature
1700ms  Complete sensor 1 read
1700ms  Start sensor 2 conversion
2450ms  Read sensor 2 temperature
2550ms  Complete sensor 2 read
2550ms  Start sensor 3 conversion
3300ms  Read sensor 3 temperature
3400ms  Complete sensor 3 read
3400ms  Task complete, delay 5000ms
```

**Impact**: 
- Blocks CPU for 3.4 seconds continuously
- Other same-priority tasks wait 3.4 seconds
- Fan control delayed (runs at same priority)
- System Monitor delayed

### With Minimal Yield Points

**Scenario**: Reading all 4 sensors in one cycle with small yields between sensors

```
Cycle (0-5000ms):
  0ms      Start sensor 0 conversion
  750ms    Read sensor 0 temperature
  850ms    Update genericSens_.temp1
  870ms    Yield (20ms) - other tasks can run
  890ms    Start sensor 1 conversion
  1640ms   Read sensor 1 temperature
  1740ms   Update genericSens_.temp2
  1760ms   Yield (20ms) - other tasks can run
  1780ms   Start sensor 2 conversion
  2530ms   Read sensor 2 temperature
  2630ms   Update genericSens_.temp3
  2650ms   Yield (20ms) - other tasks can run
  2670ms   Start sensor 3 conversion
  3420ms   Read sensor 3 temperature
  3520ms   Update genericSens_.temp4
  3540ms   Watchdog heartbeat
  3540ms   Delay until next cycle (5000ms)
```

**Impact**:
- Total reading time: ~3.54 seconds (well within 5000ms)
- Small yield windows (20ms each) allow other tasks to execute
- Fan control can execute during yield windows
- System Monitor can execute during yield windows
- All sensors updated every 5 seconds

## Priority Impact Analysis

### Higher Priority Tasks (Not Affected)

**Tasks with Priority > 1**:
- PCNT Flow Manager (Priority 5)
- I2C ADC Manager (Priority 3)
- MQTT Publisher (Priority 3)
- Sensor Acquisition (Priority 5)

**Behavior**: These tasks can preempt One-Wire task at any time. One-Wire reads can be interrupted, but this is acceptable since One-Wire protocol handles interruptions gracefully (reset and retry).

### Same Priority Tasks (Affected)

**Tasks with Priority = 1**:
- I2C Environmental Manager (Priority 1)
- System Monitor Task (Priority 1) - includes fan control

**Impact Without Yields**:
- If One-Wire task runs for 3.4 seconds without yielding
- Fan control check delayed by up to 3.4 seconds
- System Monitor logging delayed

**Impact With Minimal Yields**:
- One-Wire task yields 20ms between each sensor read
- Total yield time: ~60ms (3 yields × 20ms)
- Fan control can execute during yield windows
- System Monitor can execute during yield windows
- Small delays but all sensors complete within 5000ms

### Lower Priority Tasks (Not Applicable)

One-Wire task is already at lowest priority (1), so no lower priority tasks exist.

## Yield Point Details

### Yield Point: Between Sensor Reads (20ms)

**Purpose**: Allow other same-priority tasks brief execution window between sensor reads

**Timing**: 
- Small yield (10-20ms) after reading each sensor
- Applied between sensors 0-1, 1-2, 2-3 (not after sensor 3)
- Total yield overhead: ~60ms (3 yields × 20ms)

**Considerations**:
- Minimal overhead: 60ms out of 5000ms = 1.2% overhead
- Allows fan control and System Monitor to execute during reads
- Balance: Too short = minimal benefit, too long = exceeds 5000ms limit
- Optimal: 20ms provides enough time for other tasks to execute brief operations

**Implementation**:
```c
// After reading sensor and updating genericSens_
if (sensor_index < num_sensors - 1) {
    vTaskDelay(pdMS_TO_TICKS(20));  // Small yield between sensors
}
```

**Why Not Yield During Conversion Wait?**:
- Conversion is a hardware process that happens automatically
- We must wait synchronously for conversion to complete
- Yielding during conversion doesn't help - we still need to wait the full 750ms
- Better to yield after reading, when we're actually doing CPU work

### Main Task Delay (5000ms)

**Purpose**: Primary yield - wait until next complete sensor scan cycle

**Timing**: 
- Matches task interval (5000ms)
- Allows all other tasks to execute fully
- Complete 4-sensor scan every cycle

**Considerations**:
- Ensures other tasks get adequate CPU time
- Provides predictable timing for sensor updates
- All sensors updated every 5 seconds

**Implementation**:
```c
watchdog_task_heartbeat();
vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for next cycle
```

## Error Handling

### Sensor Read Failures

**CRC Errors**:
- Detect invalid data
- Log warning but continue
- Don't update `genericSens_` for that sensor
- Retry on next cycle

**Sensor Not Responding**:
- One-Wire reset fails
- Log error but continue
- Skip sensor for this cycle
- Retry on next cycle

**Timeout Protection**:
- Maximum read time per sensor: 1000ms
- Maximum cycle time (all 4 sensors): 4500ms safety margin
- If cycle exceeds timeout, abort remaining sensors and continue
- Prevents watchdog trigger

### Mutex Timeout

**Scenario**: `genericSens_` mutex unavailable

**Handling**:
- Timeout after 100ms
- Log warning
- Skip update for this cycle
- Retry on next cycle

**Impact**: 
- Temperature reading continues but not stored
- Non-critical - will retry next cycle
- Prevents deadlock

## Watchdog Protection

### Watchdog Configuration

**Registration**:
```c
watchdog_register_current_task("OneWireTempMgr", 6000);
```

**Heartbeat**:
- Called after each sensor read
- Timeout: 6000ms (5000ms interval + 1000ms margin)
- Protects against hung operations

### Protection Scenarios

**Hung One-Wire Read**:
- If sensor read hangs, watchdog triggers after 6 seconds
- System reset or error recovery triggered
- Prevents permanent system lock

**Infinite Loop**:
- Watchdog detects lack of heartbeat
- System recovery initiated
- Prevents system freeze

## Performance Monitoring

### Execution Time Tracking

**Monitor per-sensor read time**:
```c
uint32_t start_time = xTaskGetTickCount();
// ... read sensor ...
uint32_t elapsed = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
if (elapsed > 2000) {
    ESP_LOGW(TAG, "Sensor %d read took %lu ms", sensor_index, elapsed);
}
```

### Total Cycle Time Monitoring

**Monitor complete cycle time**:
```c
uint32_t cycle_start = xTaskGetTickCount();
// ... complete sensor read cycle ...
uint32_t cycle_elapsed = (xTaskGetTickCount() - cycle_start) * portTICK_PERIOD_MS;
if (cycle_elapsed > 5000) {
    ESP_LOGW(TAG, "Cycle exceeded interval: %lu ms", cycle_elapsed);
}
```

### Stack Usage Monitoring

**Monitor stack high-water mark**:
```c
UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
if (stack_remaining < 512) {
    ESP_LOGW(TAG, "Low stack: %d bytes remaining", stack_remaining);
}
```

## Configuration

### Timing Constants

**Configuration in `config.h`**:
```c
#define ONEWIRE_TEMP_TASK_INTERVAL_MS    5000   // Task execution interval (complete scan)
#define ONEWIRE_CONVERSION_DELAY_MS     750    // DS18B20 conversion time per sensor
#define ONEWIRE_YIELD_BETWEEN_MS        20     // Yield between sensors (minimal)
#define ONEWIRE_READ_TIMEOUT_MS         1000   // Maximum read time per sensor
#define ONEWIRE_CYCLE_TIMEOUT_MS        4500   // Maximum cycle time (safety margin)
```

### GPIO Configuration

**Pin Selection**:
```c
#define ONEWIRE_PIN                     GPIO_NUM_6  // One-Wire bus pin
```

**GPIO Setup**:
```c
gpio_config_t ow_config = {
    .pin_bit_mask = (1ULL << ONEWIRE_PIN),
    .mode = GPIO_MODE_INPUT_OUTPUT_OD,  // Open-drain mode
    .pull_up_en = GPIO_PULLUP_ENABLE,   // Enable internal pull-up (or external)
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};
ESP_ERROR_CHECK(gpio_config(&ow_config));
```

## Integration Points

### Dependencies

1. **GPIO Driver**: One-Wire protocol uses GPIO for bit-banging
2. **Sensor Data Structure**: Updates `genericSens_.temp1` through `temp4`
3. **Mutex**: Protects `genericSens_` structure access
4. **Watchdog**: Task monitoring and protection

### Initialization Order

```
1. Initialize GPIO system
2. Initialize sensor data structure with mutex
3. Initialize One-Wire GPIO pin
4. Scan One-Wire bus for devices
5. Create One-Wire Manager task
6. Task registers with watchdog
```

## Testing Considerations

### Unit Testing

1. **Single Sensor Read**:
   - Verify temperature conversion and read
   - Verify CRC checking
   - Verify Fahrenheit conversion

2. **Multiple Sensors**:
   - Verify all 4 sensors read in one cycle
   - Verify all sensors read correctly
   - Verify proper ROM code addressing
   - Verify cycle completes within 5000ms

3. **Yield Points**:
   - Verify yields occur at correct times
   - Verify other tasks can execute during yields
   - Measure actual execution time

### Integration Testing

1. **Task Interaction**:
   - Verify fan control not blocked
   - Verify System Monitor executes regularly
   - Verify I2C operations not blocked

2. **Timing Verification**:
   - Measure actual task execution time per sensor
   - Measure total cycle time (all 4 sensors)
   - Verify cycle completes within 4500ms (safety margin)
   - Verify watchdog doesn't trigger
   - Verify yields occur at correct intervals

3. **Long-Term Operation**:
   - Run for extended period
   - Verify no memory leaks
   - Verify consistent sensor readings

### Hardware Testing

1. **Sensor Discovery**:
   - Verify all sensors detected
   - Verify ROM codes stored correctly
   - Test with 1, 2, 3, 4 sensors

2. **Temperature Accuracy**:
   - Compare readings with reference thermometer
   - Verify temperature conversion accuracy
   - Test across temperature range

3. **Protocol Reliability**:
   - Test with long cables
   - Test with varying pull-up resistance
   - Test with noisy environment

## Summary

The One-Wire Temperature Manager design:

- ✅ **Complete Scan**: Reads all 4 sensors within one 5000ms cycle
- ✅ **Minimal Yields**: Small yields (20ms) between sensors - just 1.2% overhead
- ✅ **Non-Blocking**: Allows other same-priority tasks brief execution windows
- ✅ **Efficient**: Total cycle time ~3.54 seconds (well within 5000ms budget)
- ✅ **Protected**: Watchdog and timeout protection
- ✅ **Reliable**: Error handling and retry logic

**Key Design Decision**: Reading all sensors in one cycle with minimal yields (20ms) between sensors ensures:
- All sensors updated every 5 seconds
- Other same-priority tasks can execute during yield windows
- Complete cycle completes within allocated time
- Minimal CPU overhead (60ms yields out of 5000ms = 1.2%)

**Timing Breakdown**:
- Sensor reads: ~850ms × 4 = 3400ms
- Yields between sensors: 20ms × 3 = 60ms
- Other overhead: ~80ms
- **Total**: ~3540ms (well within 5000ms interval)

This approach provides efficient temperature monitoring while maintaining system responsiveness and ensuring all sensors are updated every cycle.

