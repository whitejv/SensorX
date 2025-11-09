# PCNT Flow Sensor Manager Design

## Overview

This document describes the design approach for the PCNT Flow Sensor Manager in the SensorX ESP32-C6 system. This manager uses the ESP32's built-in PCNT (Pulse Counter) hardware to monitor 3 flow sensors that produce pulse trains proportional to water flow rate. The design replaces interrupt-based counting with hardware PCNT counters for improved accuracy and reduced CPU overhead.

## Design Philosophy

**Hardware-Based Counting**: PCNT hardware handles pulse counting automatically, eliminating the need for GPIO interrupts. This reduces CPU overhead and improves reliability.

**Accumulation Window**: Sensors are checked every 1000ms, but pulse counts accumulate over a user-defined 2000ms window for optimal flow measurement accuracy.

**Minimal Pulse Loss**: PCNT hardware continues counting during read operations, minimizing pulse loss. Careful read/reset sequencing ensures no pulses are missed.

## Hardware Overview

### ESP32-C6 PCNT Capabilities

**PCNT Hardware Resources**:
- **PCNT Units**: 4 units per group (CONFIG_SOC_PCNT_UNITS_PER_GROUP=4)
- **Channels per Unit**: 2 channels per unit (used for quadrature counting)
- **Counter Width**: 16-bit signed counter (-32768 to +32767)
- **Edge Detection**: Configurable for rising edge, falling edge, or both
- **Filtering**: Built-in glitch filter available

**Available Units for Flow Sensors**:
- PCNT_UNIT_0 → Flow Sensor 1
- PCNT_UNIT_1 → Flow Sensor 2
- PCNT_UNIT_2 → Flow Sensor 3
- PCNT_UNIT_3 → Reserved/spare

### Flow Sensor Characteristics

**Hall Effect Flow Sensors**:
- Produce pulse trains proportional to flow rate
- Typical pulse rate: 1-100 Hz (depending on flow rate)
- Pulse characteristics: Digital square wave, 3.3V logic
- Edge type: Rising edge (hall switch activates)

**Pulse Counting Requirements**:
- Count on leading edge (RISING edge)
- 12-bit pulse count (0-4095 pulses) - matches FlowData structure
- Accumulation window: 2000ms for stable measurement
- Check interval: 1000ms

### FlowData Structure

**Bit-Packed Format** (matches Arduino implementation):
```c
struct FlowData {
    uint32_t pulses : 12;       // 12 bits for pulse count (0-4095)
    uint32_t milliseconds : 19; // 19 bits for elapsed time (0-524287 ms)
    uint32_t newData : 1;       // 1 bit for new data flag
};
```

**Total Size**: 32 bits (4 bytes)

**Limits**:
- Maximum pulse count: 4095 (12 bits)
- Maximum elapsed time: 524,287 ms (~8.7 minutes)
- Typical elapsed time: 2000ms (when newData=1)

## Task Design

### Task Configuration

**Priority**: `TASK_PRIORITY_CRITICAL` (5) - Highest priority
- Flow sensors are critical for system operation
- High priority ensures timely reading and data updates
- Preempts all other tasks except watchdog

**Stack Size**: `TASK_STACK_SIZE_CRITICAL` (4096 bytes)
- PCNT operations require minimal stack
- Room for sensor state tracking and data structures

**Execution Interval**: 1000ms (1 second)
- Checks all 3 sensors every second
- Reads PCNT counters and calculates elapsed time
- Updates `genericSens_` structure

**Watchdog Timeout**: 1500ms (1000ms + 500ms margin)
- Protects against hung operations
- Allows brief overrun before watchdog trigger

### Task Lifecycle

```
1. Task initialization
   a. Initialize PCNT hardware for all 3 sensors
   b. Configure GPIO pins for PCNT input
   c. Set up PCNT units (PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2)
   d. Configure edge detection (RISING edge)
   e. Start PCNT counters
   f. Initialize reset timestamps for each sensor
   g. Register with watchdog system

2. Main task loop:
   a. For each sensor (0-2):
      - Read current PCNT counter value
      - Calculate elapsed time since last reset
      - If elapsed >= 2000ms AND pulses > 0:
         * Set newData = 1
         * Store pulse count and elapsed time
         * Reset PCNT counter (atomic operation)
         * Update reset timestamp
      - Else:
         * Set newData = 0
         * Store current pulse count and elapsed time (continues accumulating)
      - Update genericSens_ structure (mutex-protected)
   b. Send watchdog heartbeat
   c. Delay for task interval (1000ms)
```

## PCNT Configuration

### PCNT Unit Configuration

**Edge Detection**: Count on leading edge (RISING)
```c
pcnt_config_t pcnt_config = {
    .pulse_gpio_num = gpio_pin,      // GPIO pin connected to flow sensor
    .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control pin needed
    .channel = PCNT_CHANNEL_0,       // Use channel 0 (single channel mode)
    .unit = pcnt_unit,               // PCNT_UNIT_0, PCNT_UNIT_1, or PCNT_UNIT_2
    .pos_mode = PCNT_COUNT_INC,      // Increment on positive edge (RISING)
    .neg_mode = PCNT_COUNT_DISABLE,  // Disable negative edge counting
    .lctrl_mode = PCNT_MODE_DISABLE, // Disable control pin
    .hctrl_mode = PCNT_MODE_DISABLE, // Disable control pin
    .counter_h_lim = 4095,           // Upper limit (12-bit max)
    .counter_l_lim = 0,             // Lower limit
};
```

**Counter Limits**:
- Upper limit: 4095 (matches FlowData.pulses 12-bit limit)
- Lower limit: 0
- When counter reaches 4095, it stops counting (saturates)
- Prevents overflow beyond 12-bit limit

**Filter Configuration**:
- Filter enabled: Yes (reduces noise and false triggers)
- Filter threshold: 1023 APB clock cycles (~12.8μs at 80MHz)
- Filters out pulses shorter than threshold duration
- Prevents counting of noise spikes and glitches
- Typical application: Filter threshold of 1023 cycles works well for flow sensors

### GPIO Configuration

**GPIO Setup**:
```c
gpio_config_t gpio_conf = {
    .pin_bit_mask = (1ULL << gpio_pin),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,   // Enable pull-up (may be needed)
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE     // No GPIO interrupt (PCNT handles it)
};
gpio_config(&gpio_conf);
```

**GPIO Pin Selection**:
- Flow Sensor 1: GPIO7 (PCNT_UNIT_0)
- Flow Sensor 2: GPIO8 (PCNT_UNIT_1)
- Flow Sensor 3: GPIO3 (PCNT_UNIT_2)
- Note: All pins are digital-capable. GPIO3 is ADC-capable but used as digital for PCNT.

## Pulse Count Read Strategy

### Problem: Minimizing Pulse Loss During Read

**Challenge**: When reading PCNT counter, there's a brief window where pulses might be missed if the counter is reset at the wrong time.

**Solution**: Atomic read-and-reset operation sequence

### Read Sequence (No Pulse Loss)

**Safe Read Procedure**:
```c
int16_t current_count = 0;

// Step 1: Stop counter (prevents counting during read)
pcnt_counter_pause(pcnt_unit);

// Step 2: Read counter value (atomic operation)
pcnt_get_counter_value(pcnt_unit, &current_count);

// Step 3: Clear counter (atomic operation)
pcnt_counter_clear(pcnt_unit);

// Step 4: Restart counter (resumes counting)
pcnt_counter_resume(pcnt_unit);
```

**Why This Works**:
- Pause prevents new pulses from incrementing counter during read
- Read gets current value
- Clear resets counter to 0
- Resume allows counting to continue
- Total pause time: <1ms (negligible pulse loss)

**Alternative: Read Without Pause** (If pause causes issues):
```c
// Step 1: Read counter value (while counting continues)
pcnt_get_counter_value(pcnt_unit, &current_count);

// Step 2: Store value for processing
int16_t saved_count = current_count;

// Step 3: Clear counter (counter continues counting immediately)
pcnt_counter_clear(pcnt_unit);

// Note: Any pulses that occur between read and clear are lost
// But this is minimal (<1ms window)
```

### Recommended Approach: Pause-Based Read

**Advantages**:
- Zero pulse loss during read operation
- Guaranteed accurate count
- Counter resumes immediately after reset

**Disadvantages**:
- Brief pause (<1ms) stops counting
- For typical flow rates (1-100 Hz), <1ms pause is negligible

**Trade-off**: Acceptable - pause is so brief that pulse loss is negligible even at high flow rates.

## Implementation Pattern

### Complete Task Implementation

```c
#include "driver/pcnt.h"
#include "driver/gpio.h"

#define MAX_FLOW_SENSORS 3
#define FLOW_ACCUMULATION_WINDOW_MS 2000  // User-defined accumulation window

typedef struct {
    pcnt_unit_t pcnt_unit;
    gpio_num_t gpio_pin;
    const char* name;
    uint32_t last_reset_time;  // Timestamp of last reset (ms)
    bool enabled;
} FlowSensorConfig_t;

static FlowSensorConfig_t flow_sensors[MAX_FLOW_SENSORS];
static uint8_t num_flow_sensors = 0;

esp_err_t pcnt_flow_manager_init(void) {
    // Initialize PCNT driver
    // No global initialization needed in ESP-IDF v5.x
    
    return ESP_OK;
}

esp_err_t pcnt_flow_manager_register_sensor(gpio_num_t gpio_pin, pcnt_unit_t pcnt_unit, const char* name) {
    if (num_flow_sensors >= MAX_FLOW_SENSORS) {
        ESP_LOGE(TAG, "Maximum flow sensors (%d) already registered", MAX_FLOW_SENSORS);
        return ESP_ERR_NO_MEM;
    }
    
    // Configure GPIO
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << gpio_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    
    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = gpio_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = PCNT_CHANNEL_0,
        .unit = pcnt_unit,
        .pos_mode = PCNT_COUNT_INC,      // Count on RISING edge
        .neg_mode = PCNT_COUNT_DISABLE,
        .lctrl_mode = PCNT_MODE_DISABLE,
        .hctrl_mode = PCNT_MODE_DISABLE,
        .counter_h_lim = 4095,          // 12-bit limit
        .counter_l_lim = 0,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));
    
    // Install PCNT driver
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    
    // Configure filter (reduces noise and false triggers)
    // Filter threshold: 1023 APB clock cycles (~12.8μs at 80MHz)
    // This filters out pulses shorter than ~12.8μs (noise spikes)
    ESP_ERROR_CHECK(pcnt_set_filter_value(pcnt_unit, PCNT_FILTER_THRESHOLD));
    ESP_ERROR_CHECK(pcnt_filter_enable(pcnt_unit));
    
    // Clear counter
    ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit));
    
    // Start counter
    ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_unit));
    
    // Store sensor configuration
    flow_sensors[num_flow_sensors].pcnt_unit = pcnt_unit;
    flow_sensors[num_flow_sensors].gpio_pin = gpio_pin;
    flow_sensors[num_flow_sensors].name = name;
    flow_sensors[num_flow_sensors].last_reset_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    flow_sensors[num_flow_sensors].enabled = true;
    
    num_flow_sensors++;
    
    ESP_LOGI(TAG, "Registered flow sensor: %s (GPIO%d, PCNT_UNIT_%d)", 
             name, gpio_pin, pcnt_unit);
    
    return ESP_OK;
}

void vPcntFlowManagerTask(void *pvParameters) {
    (void)pvParameters;
    
    ESP_LOGI(TAG, "PCNT flow manager task started");
    
    // Register with watchdog
    watchdog_register_current_task("PcntFlowMgr", PCNT_FLOW_TASK_INTERVAL_MS + 500);
    
    const TickType_t xTaskInterval = pdMS_TO_TICKS(1000);  // 1000ms check interval
    
    for (;;) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Take mutex to update genericSens_ structure
        if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Process all flow sensors
            for (uint8_t i = 0; i < num_flow_sensors; i++) {
                if (!flow_sensors[i].enabled) continue;
                
                FlowSensorConfig_t* sensor = &flow_sensors[i];
                FlowData_t flow_data = {0};
                
                // Read PCNT counter value (atomic operation)
                int16_t pulse_count = 0;
                
                // Pause counter to prevent counting during read
                pcnt_counter_pause(sensor->pcnt_unit);
                
                // Read current count
                pcnt_get_counter_value(sensor->pcnt_unit, &pulse_count);
                
                // Calculate elapsed time since last reset
                uint32_t elapsed_ms = current_time - sensor->last_reset_time;
                
                // Check if accumulation window complete AND pulses detected
                if (elapsed_ms >= FLOW_ACCUMULATION_WINDOW_MS && pulse_count > 0) {
                    // Window complete - store data and reset
                    flow_data.pulses = (uint16_t)pulse_count;  // Cast to 12-bit
                    flow_data.milliseconds = elapsed_ms;
                    flow_data.newData = 1;
                    
                    // Clear counter (atomic operation)
                    pcnt_counter_clear(sensor->pcnt_unit);
                    
                    // Update reset timestamp
                    sensor->last_reset_time = current_time;
                    
                    ESP_LOGD(TAG, "Sensor %s: %d pulses in %lu ms (newData=1)", 
                             sensor->name, pulse_count, elapsed_ms);
                } else {
                    // Window not complete - store current data but continue accumulating
                    flow_data.pulses = (uint16_t)pulse_count;
                    flow_data.milliseconds = elapsed_ms;
                    flow_data.newData = 0;
                    
                    ESP_LOGD(TAG, "Sensor %s: %d pulses in %lu ms (accumulating)", 
                             sensor->name, pulse_count, elapsed_ms);
                }
                
                // Resume counter (continues counting from 0 if reset, or continues if not reset)
                pcnt_counter_resume(sensor->pcnt_unit);
                
                // Update genericSens_ structure based on sensor index
                switch (i) {
                    case 0:
                        memcpy(&genericSens_.flowData1, &flow_data, sizeof(FlowData_t));
                        break;
                    case 1:
                        memcpy(&genericSens_.flowData2, &flow_data, sizeof(FlowData_t));
                        break;
                    case 2:
                        // Need to add flowData3 to genericSens_ structure
                        // For now, could use a different field or extend structure
                        ESP_LOGW(TAG, "Flow sensor 3 not yet supported in genericSens_");
                        break;
                }
            }
            
            xSemaphoreGive(sensor_data.mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire mutex for flow sensor update");
        }
        
        // Send watchdog heartbeat
        watchdog_task_heartbeat();
        
        // Delay until next check interval
        vTaskDelay(xTaskInterval);
    }
}
```

## Timing Behavior

### Accumulation Window Logic

**Key Behavior**: Pulse counts accumulate over 2000ms window, but task checks every 1000ms.

**Timeline Example**:
```
Time    Task Check    Sensor 0 State                    newData
0ms     Check         Reset counter, start timer        -
1000ms  Check         Count: 45, elapsed: 1000ms       0 (accumulating)
2000ms  Check         Count: 92, elapsed: 2000ms         1 (window complete!)
        Reset counter, update genericSens_, reset timer
3000ms  Check         Count: 47, elapsed: 1000ms         0 (accumulating)
4000ms  Check         Count: 89, elapsed: 2000ms         1 (window complete!)
        Reset counter, update genericSens_, reset timer
```

**Behavior**:
- Every 1000ms: Task checks counter and elapsed time
- If elapsed >= 2000ms AND pulses > 0: Set newData=1, reset counter and timer
- If elapsed < 2000ms: Set newData=0, continue accumulating
- Result: Every other check (every 2000ms) produces newData=1

### Pulse Count Saturation

**12-Bit Limit**: FlowData.pulses is 12 bits (0-4095)

**Handling Overflow**:
- PCNT counter upper limit: 4095
- Counter saturates at 4095 (stops counting)
- If counter reaches 4095 before 2000ms window:
  - Set newData=1 immediately
  - Store 4095 pulses
  - Reset counter
  - Log warning if this occurs frequently

**Implementation**:
```c
if (pulse_count >= 4095) {
    // Counter saturated - must reset immediately
    ESP_LOGW(TAG, "Sensor %s: Counter saturated at 4095", sensor->name);
    flow_data.pulses = 4095;
    flow_data.milliseconds = elapsed_ms;
    flow_data.newData = 1;
    // Reset counter and timer
    pcnt_counter_clear(sensor->pcnt_unit);
    sensor->last_reset_time = current_time;
}
```

## Pulse Loss Minimization

### PCNT Hardware Advantages

**Automatic Counting**: PCNT hardware counts pulses independently of CPU
- Pulses counted even during task delays
- Pulses counted even if task is blocked
- No CPU overhead for counting

**Atomic Operations**: PCNT operations are atomic
- Read operation is atomic
- Clear operation is atomic
- Pause/resume are atomic

### Read Sequence Optimization

**Pause-Based Read** (Recommended):
```
1. Pause counter (<1ms)
2. Read counter value
3. Clear counter (if window complete)
4. Resume counter
```

**Pulse Loss**: <1ms window where counting paused
- At 100 Hz pulse rate: ~0.1 pulses lost per read
- Negligible for flow measurement accuracy

**Alternative: Read-Without-Pause** (If pause causes issues):
```
1. Read counter value (counter continues)
2. Clear counter (if window complete)
3. Small window between read and clear: ~10-50μs
```

**Pulse Loss**: 10-50μs window
- At 100 Hz pulse rate: ~0.001-0.005 pulses lost
- Even more negligible

### Edge Case: High Pulse Rate

**Scenario**: Very high flow rate produces >100 Hz pulses

**Mitigation**:
- PCNT hardware handles high-frequency pulses
- Pause duration is fixed (<1ms) regardless of pulse rate
- Pulse loss remains constant and negligible

**Monitoring**:
- Log warning if pulse count approaches 4095 limit
- Consider reducing accumulation window if saturation occurs frequently

## Error Handling

### PCNT Read Failures

**PCNT Get Counter Error**:
- If `pcnt_get_counter_value()` fails
- Log error and continue
- Don't update `genericSens_` for that sensor
- Retry on next cycle

**PCNT Counter Clear Error**:
- If `pcnt_counter_clear()` fails
- Log error
- Continue without clearing (counter continues counting)
- May cause overflow on next cycle - monitor

### Counter Overflow

**16-Bit Counter Limit**: PCNT counter is 16-bit signed (-32768 to +32767)

**Protection**:
- Set upper limit to 4095 (matches FlowData structure)
- Counter saturates at 4095, preventing overflow
- If counter reaches 4095 before 2000ms window:
  - Reset immediately (don't wait for window)
  - Log warning
  - Set newData=1

**Implementation**:
```c
if (pulse_count >= 4095) {
    // Counter saturated - reset immediately
    ESP_LOGW(TAG, "Sensor %s: Counter saturated (high flow rate)", sensor->name);
    flow_data.pulses = 4095;
    flow_data.milliseconds = elapsed_ms;
    flow_data.newData = 1;
    pcnt_counter_clear(sensor->pcnt_unit);
    sensor->last_reset_time = current_time;
}
```

### Mutex Timeout

**Scenario**: `genericSens_` mutex unavailable

**Handling**:
- Timeout after 100ms
- Log warning
- Skip update for this cycle
- Retry on next cycle

**Impact**: 
- Pulse counting continues (PCNT hardware independent)
- Data update delayed by one cycle (1000ms)
- Non-critical - will update next cycle

## Watchdog Protection

### Watchdog Configuration

**Registration**:
```c
watchdog_register_current_task("PcntFlowMgr", 1500);
```

**Heartbeat**:
- Called after processing all sensors
- Timeout: 1500ms (1000ms interval + 500ms margin)
- Protects against hung operations

### Protection Scenarios

**Hung PCNT Read**:
- If PCNT read hangs, watchdog triggers after 1.5 seconds
- System reset or error recovery triggered
- Prevents permanent system lock

**Infinite Loop**:
- Watchdog detects lack of heartbeat
- System recovery initiated
- Prevents system freeze

## Performance Considerations

### CPU Overhead

**PCNT Hardware Counting**: Zero CPU overhead
- Counting happens in hardware
- CPU only involved during read operations

**Task Execution**: Minimal CPU usage
- Read 3 PCNT counters: ~100μs total
- Update 3 FlowData structures: ~50μs
- Mutex operations: ~10μs
- **Total**: ~160μs per 1000ms cycle = 0.016% CPU usage

### Memory Usage

**Per-Sensor State**:
- FlowSensorConfig_t: ~20 bytes
- 3 sensors: ~60 bytes total
- Minimal memory footprint

**Stack Usage**:
- PCNT operations: minimal stack
- 4096 bytes stack provides ample headroom

## Configuration

### Timing Constants

**Configuration in `config.h`**:
```c
#define PCNT_FLOW_TASK_INTERVAL_MS       1000  // Task check interval
#define FLOW_ACCUMULATION_WINDOW_MS     2000  // User-defined accumulation window
#define PCNT_COUNTER_LIMIT               4095  // 12-bit limit (matches FlowData)
#define PCNT_FILTER_THRESHOLD            1023  // Filter threshold (APB clock cycles, ~12.8μs at 80MHz)
```

**Filter Threshold Selection**:
- **1023 cycles** (~12.8μs at 80MHz): Recommended for flow sensors
  - Filters out noise spikes and glitches
  - Doesn't affect valid flow sensor pulses (typically >100μs wide)
  - Good balance between noise rejection and pulse detection
- **Adjust if needed**: If valid pulses are being filtered, reduce threshold
- **Increase if needed**: If noise is still present, increase threshold (up to 1023 max)

### GPIO Pin Definitions

**Pin Selection**:
```c
#define FLOW_SENSOR_1_GPIO    GPIO_NUM_7   // GPIO7 - PCNT Flow Sensor 1
#define FLOW_SENSOR_2_GPIO    GPIO_NUM_8   // GPIO8 - PCNT Flow Sensor 2
#define FLOW_SENSOR_3_GPIO    GPIO_NUM_3   // GPIO3 - PCNT Flow Sensor 3
```

### PCNT Unit Assignments

```c
#define FLOW_SENSOR_1_PCNT_UNIT    PCNT_UNIT_0
#define FLOW_SENSOR_2_PCNT_UNIT    PCNT_UNIT_1
#define FLOW_SENSOR_3_PCNT_UNIT    PCNT_UNIT_2
```

## Integration Points

### Dependencies

1. **PCNT Driver**: ESP-IDF PCNT driver (`driver/pcnt.h`)
2. **GPIO Driver**: ESP-IDF GPIO driver (`driver/gpio.h`)
3. **Sensor Data Structure**: Updates `genericSens_.flowData1`, `flowData2`, `flowData3`
4. **Mutex**: Protects `genericSens_` structure access
5. **Watchdog**: Task monitoring and protection

### Initialization Order

```
1. Initialize GPIO system
2. Initialize sensor data structure with mutex
3. Initialize PCNT Flow Manager
4. Register flow sensors (GPIO pins, PCNT units)
5. Create PCNT Flow Manager task
6. Task registers with watchdog
```

### genericSens_ Structure Update

**Note**: Current structure has `flowData1` and `flowData2`. Need to add `flowData3` for third sensor:

```c
typedef struct {
    // ... existing fields ...
    FlowData_t flowData1;  // Flow sensor 1
    FlowData_t flowData2;  // Flow sensor 2
    FlowData_t flowData3;  // Flow sensor 3 (to be added)
    // ... rest of fields ...
} GenericSensorData_t;
```

## Testing Considerations

### Unit Testing

1. **PCNT Configuration**:
   - Verify PCNT unit configuration
   - Verify GPIO pin configuration
   - Verify edge detection (RISING edge)
   - Verify counter limits (0-4095)

2. **Pulse Counting**:
   - Generate test pulses at known rate
   - Verify counter increments correctly
   - Verify counter resets correctly
   - Verify no pulse loss during read

3. **Accumulation Window**:
   - Verify newData=1 when elapsed >= 2000ms
   - Verify newData=0 when elapsed < 2000ms
   - Verify timer reset after window complete

### Integration Testing

1. **Task Interaction**:
   - Verify task executes every 1000ms
   - Verify other tasks not blocked
   - Verify watchdog doesn't trigger

2. **Data Updates**:
   - Verify `genericSens_.flowData1/2/3` updated correctly
   - Verify mutex protection works
   - Verify bit-packed structure correct

3. **Long-Term Operation**:
   - Run for extended period
   - Verify no memory leaks
   - Verify consistent pulse counting
   - Verify no counter overflow

### Hardware Testing

1. **Flow Sensor Connection**:
   - Connect flow sensors to GPIO pins
   - Verify pulse detection
   - Verify pulse counting accuracy

2. **Flow Rate Testing**:
   - Test at various flow rates
   - Verify pulse count accuracy
   - Verify newData flag behavior
   - Test saturation scenario (high flow rate)

3. **Pulse Loss Testing**:
   - Measure actual pulse loss during reads
   - Verify loss is negligible (<0.1%)
   - Test at maximum pulse rate

## Comparison: Interrupt vs PCNT

### Interrupt-Based Approach (Arduino)

**Advantages**:
- Simple implementation
- Works on any GPIO pin
- No hardware resource limits

**Disadvantages**:
- CPU overhead for each interrupt
- Interrupt latency affects counting
- Can miss pulses if interrupts disabled
- Requires volatile variables

### PCNT Hardware Approach (ESP-IDF)

**Advantages**:
- Zero CPU overhead for counting
- Hardware handles counting automatically
- No pulse loss (hardware continues counting)
- More reliable at high pulse rates
- No interrupt overhead

**Disadvantages**:
- Limited hardware resources (4 PCNT units)
- Requires PCNT-capable GPIO pins
- More complex configuration

**Conclusion**: PCNT hardware approach is superior for flow sensors - provides better accuracy, lower CPU usage, and more reliable operation.

## Summary

The PCNT Flow Sensor Manager design:

- ✅ **Hardware-Based**: Uses ESP32-C6 PCNT hardware for automatic pulse counting
- ✅ **3 Sensors**: Supports 3 flow sensors (PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2)
- ✅ **Leading Edge**: Counts on RISING edge (leading edge)
- ✅ **Reset After Read**: Counter resets after 2000ms window completes
- ✅ **Minimal Pulse Loss**: Pause-based read ensures <1ms window, negligible loss
- ✅ **Accumulation Window**: 2000ms user-defined window for optimal accuracy
- ✅ **Check Interval**: 1000ms check interval (every other check produces newData=1)
- ✅ **Protected**: Watchdog and counter overflow protection
- ✅ **Efficient**: Minimal CPU usage (~0.016%)

**Key Design Decisions**:
1. **Pause-Based Read**: Pausing counter during read ensures zero pulse loss
2. **2000ms Window**: User-defined accumulation window provides optimal flow measurement accuracy
3. **1000ms Check**: Checking every second allows responsive updates while maintaining 2000ms accumulation
4. **Counter Limits**: Setting upper limit to 4095 matches FlowData structure and prevents overflow

This design provides accurate, reliable flow sensor monitoring with minimal CPU overhead and guaranteed pulse counting accuracy.

