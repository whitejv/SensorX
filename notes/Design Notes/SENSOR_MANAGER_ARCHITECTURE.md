# Sensor Manager Architecture Documentation

## Overview

This document describes the RTOS task-based sensor manager architecture for the SensorX ESP32 system. This approach groups sensors by interface type and read interval, with each group managed by its own FreeRTOS task running at an appropriate priority and timing interval.

## Design Philosophy

**Hybrid Approach**: This architecture balances simplicity with organization:
- **Organized**: Sensors grouped by interface (PCNT, I2C, One-Wire)
- **Simple**: No complex abstraction layers, direct hardware access
- **RTOS-Native**: Leverages FreeRTOS tasks for scheduling and isolation
- **Maintainable**: Clear separation of concerns without over-engineering

## Architecture Components

### 1. Sensor Manager Types

Sensors are organized into manager groups based on their interface and optimal read interval:

| Manager Type | Interface | Read Interval | Priority | Task Stack |
|-------------|----------|--------------|----------|------------|
| **PCNT Flow Manager** | PCNT (Pulse Counter) | 1000ms* | Critical (5) | 4096 bytes |
| **I2C ADC Manager** | I2C (ADS1015/ADS1115) | 1000ms | Fixed Freq (3) | 3072 bytes |
| **I2C GPIO Manager** | I2C (MCP23X17) | 1000ms | Fixed Freq (3) | 3072 bytes |
| **GPIO Discrete Manager** | Direct GPIO | 1000ms | Fixed Freq (3) | 3072 bytes |
| **I2C Environmental Manager** | I2C (BME280, etc.) | 5000ms | Background (1) | 2048 bytes |
| **One-Wire Temperature Manager** | One-Wire (DS18B20) | 5000ms | Background (1) | 2048 bytes |

**Note**: Fan control is handled by the System Monitor Task (not a sensor manager). See System Monitor section below.

*Flow sensors are read every 1000ms but accumulate pulses over 2000ms for optimal accuracy. The `newData` flag indicates when a complete 2000ms accumulation is ready.

### 2. Shared Sensor Data Structure

All sensor managers update a single global data structure (`genericSens_`) that provides:
- **Structured access**: Named fields for each sensor type
- **Raw byte access**: `data_payload` array for MQTT binary transmission
- **Thread-safe access**: Mutex protection for concurrent updates

```c
// Conceptual structure (exact implementation may vary)
typedef struct {
    // Flow sensors (PCNT) - updated by PCNT Flow Manager
    FlowData_t flowData1;        // Bit-packed: pulses (12), milliseconds (19), newData (1)
    FlowData_t flowData2;        // Bit-packed: pulses (12), milliseconds (19), newData (1)
    
    // Temperature sensors (One-Wire) - updated by One-Wire Manager
    int temp1;                   // Integer temperature (F)
    float temp1_f;              // Float temperature (F)
    float temp2;                 // Float temperature (F)
    float temp3;                 // Float temperature (F)
    float temp4;                 // Float temperature (F)
    
    // ADC sensors (I2C) - updated by I2C ADC Manager
    int adc_sensor;              // Basic ADC reading
    float adc_x1;                // ADC channel 1 (pressure, current, sound, etc.)
    float adc_x2;                // ADC channel 2
    float adc_x3;                // ADC channel 3
    float adc_x4;                // ADC channel 4
    float adc_x5;                // ADC channel 5
    float adc_x6;                // ADC channel 6
    float adc_x7;                // ADC channel 7
    float adc_x8;                // ADC channel 8
    
    // Environmental sensors (I2C) - updated by I2C Environmental Manager
    float tempx;                 // BME280 temperature (F)
    float humidity;              // BME280 humidity (%)
    float pressurex;              // BME280 pressure (PSI)
    
    // GPIO sensors - updated by GPIO managers
    int gpio_sensor;             // GPIO input status (discrete inputs)
    int GPIO_x1;                 // GPIO expander port A (MCP23X17)
    int GPIO_x2;                 // GPIO expander port B (MCP23X17)
    
    // System data
    uint32_t cycle_count;        // Main loop cycle counter
    int fw_version;              // Firmware version
    int tempSensorcount;          // Number of One-Wire sensors found
    
    // Raw byte array for MQTT binary transmission
    uint8_t data_payload[];      // Shared memory with structured data above
} GenericSensorData_t;

extern GenericSensorData_t genericSens_;
```

**Key Points:**
- The `data_payload` array shares memory with the structured fields (union)
- This allows direct MQTT binary publishing: `mqtt_publish(..., genericSens_.data_payload, len)`
- All sensor managers update their respective fields in this structure
- Mutex protection ensures thread-safe concurrent updates

## Sensor Manager Details

### PCNT Flow Sensor Manager

**Purpose**: Manages flow sensors using ESP32 PCNT (Pulse Counter) hardware

**Interface**: PCNT hardware units with GPIO interrupt input

**Sensors Supported**:
- Flow sensors (Hall effect, paddle wheel, etc.)
- Any sensor that produces pulse output

**Read Interval**: 1000ms (1 second) - **reads every second, but accumulates over 2000ms**

**Task Configuration**:
- Priority: `TASK_PRIORITY_CRITICAL` (5)
- Stack: `TASK_STACK_SIZE_CRITICAL` (4096 bytes)
- Watchdog Interval: 1500ms (1000ms + 500ms margin)

**Implementation Details**:
- Uses PCNT hardware units (PCNT_UNIT_0, PCNT_UNIT_1, etc.)
- GPIO interrupts increment pulse counters continuously
- Task reads counters every 1000ms and calculates deltas
- **Key Behavior**: Flow sensors accumulate pulses over 2000ms intervals for optimal accuracy
  - Every 1000ms: Task reads PCNT counter, calculates elapsed time since last reset
  - If elapsed time >= 2000ms AND pulses > 0: Sets `newData = 1`, resets counter and timer
  - If elapsed time < 2000ms: Sets `newData = 0`, continues accumulating
  - This ensures every other read cycle has fresh flow data (2000ms accumulation)
- Updates `genericSens_.flowData1/flowData2` every cycle (even when `newData = 0`)
- Bit-packed FlowData structure: 12 bits pulses, 19 bits milliseconds, 1 bit newData flag

**Registration**:
```c
pcnt_flow_manager_register_sensor(GPIO_NUM_7, PCNT_UNIT_0, "Flow1");
pcnt_flow_manager_register_sensor(GPIO_NUM_8, PCNT_UNIT_1, "Flow2");
pcnt_flow_manager_register_sensor(GPIO_NUM_3, PCNT_UNIT_2, "Flow3");
```

**Updates genericSens_ fields**:
- `genericSens_.flowData1` (bit-packed FlowData structure) - updated every 1000ms
- `genericSens_.flowData2` (bit-packed FlowData structure) - updated every 1000ms
- `genericSens_.flowData3` (bit-packed FlowData structure) - updated every 1000ms
- `genericSens_.flowData1.newData` / `flowData2.newData` / `flowData3.newData` - set to 1 only when 2000ms accumulation complete

### I2C ADC Sensor Manager

**Purpose**: Manages I2C ADC sensors (ADS1015, ADS1115) for pressure, current, sound, etc.

**Interface**: I2C (ADS1015 @ 0x48, ADS1115 @ 0x49)

**Sensors Supported**:
- Pressure sensors (via ADC channel)
- Current sensors (via ADC channel)
- Sound level sensors (via ADC channel)
- Any analog sensor connected to ADC channels

**Read Interval**: 1000ms (1 second)

**Task Configuration**:
- Priority: `TASK_PRIORITY_FIXED_FREQ` (3)
- Stack: `TASK_STACK_SIZE_FIXED_FREQ` (3072 bytes)
- Watchdog Interval: 1500ms (1000ms + 500ms margin)

**Implementation Details**:
- Uses existing `i2c_manager` for I2C bus access
- Creates device handles for ADS1015/ADS1115
- Reads ADC channels sequentially
- Applies scale factors to convert voltage to physical units
- Updates `genericSens_.adc_x1` through `genericSens_.adc_x8` based on channel mapping
- Signals completion via event group for publishing coordination

**Registration**:
```c
i2c_adc_manager_register_sensor(0x48, 0, SENSOR_TYPE_PRESSURE, "Pressure", 100.0);
i2c_adc_manager_register_sensor(0x48, 1, SENSOR_TYPE_CURRENT, "Current", 50.0);
i2c_adc_manager_register_sensor(0x48, 2, SENSOR_TYPE_SOUND, "Sound", 1.0);
```

**Updates genericSens_ fields**:
- `genericSens_.adc_x1` through `genericSens_.adc_x8` (based on channel mapping)
- `genericSens_.adc_sensor` (basic ADC reading, if configured)

### I2C GPIO Manager

**Purpose**: Manages I2C GPIO expander (MCP23X17) for reading discrete inputs

**Interface**: I2C (MCP23X17 @ 0x20)

**Sensors Supported**:
- MCP23X17 GPIO expander (Port A and Port B)
- 16 discrete inputs via I2C

**Read Interval**: 1000ms (1 second)

**Task Configuration**:
- Priority: `TASK_PRIORITY_FIXED_FREQ` (3)
- Stack: `TASK_STACK_SIZE_FIXED_FREQ` (3072 bytes)
- Watchdog Interval: 1500ms (1000ms + 500ms margin)

**Implementation Details**:
- Uses existing `i2c_manager` for I2C bus access
- Creates device handle for MCP23X17 at address 0x20
- Reads Port A and Port B registers sequentially
- Updates `genericSens_.GPIO_x1` (Port A) and `genericSens_.GPIO_x2` (Port B)
- Signals completion via event group for publishing coordination

**Registration**:
```c
i2c_gpio_manager_init();
i2c_gpio_manager_register_expander(0x20);  // MCP23X17 address
i2c_gpio_manager_start_task();
```

**Updates genericSens_ fields**:
- `genericSens_.GPIO_x1` (MCP23X17 Port A - 8 bits)
- `genericSens_.GPIO_x2` (MCP23X17 Port B - 8 bits)

### GPIO Discrete Manager

**Purpose**: Manages direct GPIO discrete inputs (config pins, status pins, etc.)

**Interface**: Direct GPIO pins

**Sensors Supported**:
- Discrete digital inputs (config pins, status indicators, etc.)
- Any GPIO pin configured as input

**Read Interval**: 1000ms (1 second)

**Task Configuration**:
- Priority: `TASK_PRIORITY_FIXED_FREQ` (3)
- Stack: `TASK_STACK_SIZE_FIXED_FREQ` (3072 bytes)
- Watchdog Interval: 1500ms (1000ms + 500ms margin)

**Implementation Details**:
- Reads GPIO pins directly (no I2C overhead)
- Combines multiple GPIO inputs into single value
- Updates `genericSens_.gpio_sensor` with combined discrete input value
- Signals completion via event group for publishing coordination

**Registration**:
```c
gpio_discrete_manager_init();
gpio_discrete_manager_register_input(GPIO_NUM_X, "Config1");
gpio_discrete_manager_register_input(GPIO_NUM_Y, "Config2");
gpio_discrete_manager_start_task();
```

**Updates genericSens_ fields**:
- `genericSens_.gpio_sensor` (combined discrete input value, e.g., `(discInput2 << 1) | discInput1`)

### I2C Environmental Sensor Manager

**Purpose**: Manages I2C environmental sensors (BME280, BMP280, etc.)

**Interface**: I2C (BME280 @ 0x77, etc.)

**Sensors Supported**:
- BME280 (temperature, humidity, pressure)
- BMP280 (temperature, pressure)
- Other I2C environmental sensors

**Read Interval**: 5000ms (5 seconds)

**Task Configuration**:
- Priority: `TASK_PRIORITY_BACKGROUND` (1)
- Stack: `TASK_STACK_SIZE_BACKGROUND` (2048 bytes)
- Watchdog Interval: 6000ms (5000ms + 1000ms margin)

**Implementation Details**:
- Uses existing `i2c_manager` for I2C bus access
- Reads temperature, humidity, and pressure registers
- Performs unit conversions (Celsius to Fahrenheit, Pascal to PSI)
- Updates `genericSens_.tempx`, `genericSens_.humidity`, `genericSens_.pressurex`

**Registration**:
```c
i2c_env_manager_register_bme280(0x77);
```

**Updates genericSens_ fields**:
- `genericSens_.tempx` (temperature in Fahrenheit)
- `genericSens_.humidity` (humidity percentage)
- `genericSens_.pressurex` (pressure in PSI)

### One-Wire Temperature Manager

**Purpose**: Manages One-Wire temperature sensors (DS18B20)

**Interface**: One-Wire bus (single GPIO pin)

**Sensors Supported**:
- DS18B20 temperature sensors
- Other One-Wire temperature sensors

**Read Interval**: 5000ms (5 seconds)

**Task Configuration**:
- Priority: `TASK_PRIORITY_BACKGROUND` (1)
- Stack: `TASK_STACK_SIZE_BACKGROUND` (2048 bytes)
- Watchdog Interval: 6000ms (5000ms + 1000ms margin)

**Implementation Details**:
- Uses One-Wire protocol on single GPIO pin
- Scans bus for devices on initialization
- Reads sensors sequentially (one per task cycle)
- Updates `genericSens_.temp1` through `genericSens_.temp4` based on sensor index
- Stores ROM codes for device addressing

**Registration**:
```c
onewire_temp_manager_init(GPIO_NUM_6);
onewire_temp_manager_scan_devices();  // Auto-discovers DS18B20 sensors
```

**Updates genericSens_ fields**:
- `genericSens_.temp1` (integer temperature in Fahrenheit)
- `genericSens_.temp1_f` (float temperature in Fahrenheit)
- `genericSens_.temp2` through `genericSens_.temp4` (float temperatures)
- `genericSens_.tempSensorcount` (number of sensors found)

### Time Management (Built-in RTC with SNTP)

**Purpose**: Provides accurate timestamps for data logging using ESP32-C6 built-in RTC synchronized with NTP

**Interface**: Built-in ESP32-C6 RTC + SNTP (Simple Network Time Protocol)

**Initialization**: After WiFi connection established

**Implementation Details**:
- Uses ESP-IDF built-in SNTP client for automatic time synchronization
- SNTP syncs with NTP servers (e.g., pool.ntp.org) after WiFi connects
- Periodic re-sync (configurable, default 1 hour)
- Standard C time functions available: `time()`, `gettimeofday()`, `localtime()`, `strftime()`
- No external I2C RTC device needed - simplifies hardware and reduces I2C bus traffic
- Time persists across deep sleep (if configured)
- Sufficient accuracy for logging timestamps (±1 second typical)

**Initialization**:
```c
// After WiFi connects, initialize SNTP
#include "lwip/apps/sntp.h"

void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");  // Primary NTP server
    sntp_init();
}
```

**Usage**:
```c
#include <time.h>

// Get current time
time_t now;
time(&now);

// Format timestamp for logging
struct tm timeinfo;
localtime_r(&now, &timeinfo);
char timestamp[32];
strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
```

**Benefits**:
- No I2C device required (one less device on bus)
- Automatic synchronization with NTP servers
- Standard C library functions (no custom driver needed)
- Lower hardware cost
- Less power consumption
- Simpler codebase

## System-Level Control Functions

### Fan Control (System Monitor Task)

**Purpose**: Controls cooling fan based on temperature readings from environmental sensor

**Location**: Implemented as part of the System Monitor Task (`vSystemMonitorTask`)

**Control Logic**: 
- Fan ON when `genericSens_.tempx >= 70.0°F`
- Fan OFF when `genericSens_.tempx < 70.0°F`

**Control Interval**: 1000ms (1 second) - checked every loop iteration

**Implementation Details**:
- Fan control runs every 1000ms within the System Monitor Task
- System Monitor logging output occurs every 5000ms (separate counter)
- Reads temperature from `genericSens_.tempx` (updated by I2C Environmental Manager)
- Controls GPIO output pin (HIGH = fan ON, LOW = fan OFF)
- Uses mutex-protected read of `genericSens_` structure
- Provides responsive fan control while sensor reads occur every 5000ms

**Rationale for System Monitor Integration**:
- Fan control is a system-level control function, not a sensor manager
- System Monitor already reads system state - adding fan control is a natural fit
- Reduces task count and simplifies architecture
- Keeps system-level control functions together

**Implementation Example**:
```c
// In vSystemMonitorTask() - runs every 1000ms
void vSystemMonitorTask(void *pvParameters) {
    static uint32_t log_counter = 0;
    const TickType_t xControlInterval = pdMS_TO_TICKS(1000);  // Fan control every 1s
    const TickType_t xLogInterval = pdMS_TO_TICKS(5000);     // Logging every 5s
    
    for (;;) {
        // Fan control (every loop iteration = 1000ms)
        if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            float current_temp = genericSens_.tempx;
            xSemaphoreGive(sensor_data.mutex);
            
            if (current_temp >= 70.0) {
                gpio_set_level(GPIO_NUM_0, 1);  // Fan ON
            } else {
                gpio_set_level(GPIO_NUM_0, 0);  // Fan OFF
            }
        }
        
        // System monitoring (every 5th iteration = 5000ms)
        log_counter++;
        if (log_counter >= 5) {
            log_counter = 0;
            // ... existing system monitoring code ...
        }
        
        watchdog_task_heartbeat();
        vTaskDelay(xControlInterval);
    }
}
```

## Task Lifecycle

### Initialization Sequence

1. **System Initialization** (`app_main()`)
   - Initialize I2C manager
   - Initialize sensor data structure with mutex
   - Initialize event group for sensor coordination
   - Initialize all sensor managers

2. **Manager Initialization**
   - Each manager initializes its hardware interface
   - Scans for connected devices
   - Registers sensors with manager

3. **Task Creation**
   - Each manager creates its own FreeRTOS task
   - Tasks register with watchdog system
   - Tasks begin executing at their configured intervals

### Task Execution Flow

Each sensor manager task follows this pattern:

```
1. Task starts
2. Register with watchdog
3. Loop forever:
   a. Take mutex on genericSens_ structure
   b. Read all sensors managed by this manager
   c. Update appropriate fields in genericSens_
   d. Release mutex
   e. Signal completion via event group (for 1000ms sensors)
   f. Send heartbeat to watchdog
   g. Delay for configured interval
```

### Thread Safety

All sensor managers use a mutex (`sensor_data.mutex`) to protect concurrent access to `genericSens_`:

- **Read operations**: MQTT publisher, system monitor, etc. take mutex before reading
- **Write operations**: Sensor managers take mutex before updating fields
- **Timeout**: Mutex operations timeout after 100ms to prevent deadlock

## Publishing Coordination

### Synchronization Strategy

The MQTT publisher runs at 1000ms intervals and must wait for all sensors that read at 1000ms intervals to complete before publishing. This ensures data consistency and prevents publishing stale or incomplete data.

### Event Group Coordination

FreeRTOS Event Groups are used to coordinate sensor completion:

```c
// sensor_coordination.h
#define SENSOR_EVENT_FLOW_COMPLETE       BIT0  // PCNT flow sensors
#define SENSOR_EVENT_ADC_COMPLETE        BIT1  // I2C ADC sensors
#define SENSOR_EVENT_GPIO_I2C_COMPLETE  BIT2  // I2C GPIO expander
#define SENSOR_EVENT_GPIO_DISC_COMPLETE  BIT3  // Discrete GPIO inputs
#define SENSOR_EVENT_ENV_COMPLETE       BIT4  // Environmental sensors (5000ms)
#define SENSOR_EVENT_TEMP_COMPLETE      BIT5  // Temperature sensors (5000ms)

// All sensors that must complete before 1000ms publish
#define SENSOR_EVENT_1000MS_MASK        (SENSOR_EVENT_FLOW_COMPLETE | \
                                         SENSOR_EVENT_ADC_COMPLETE | \
                                         SENSOR_EVENT_GPIO_I2C_COMPLETE | \
                                         SENSOR_EVENT_GPIO_DISC_COMPLETE)

extern EventGroupHandle_t sensor_event_group;
```

### Sensor Manager Completion Signaling

Each sensor manager sets its completion bit after updating `genericSens_`:

```c
// Example from I2C ADC Manager
void vI2cAdcManagerTask(void *pvParameters) {
    // ... read sensors and update genericSens_ ...
    
    // Signal completion
    xEventGroupSetBits(sensor_event_group, SENSOR_EVENT_ADC_COMPLETE);
    
    watchdog_task_heartbeat();
    vTaskDelay(xInterval);
}
```

### MQTT Publisher Coordination

The MQTT publisher waits for all 1000ms sensors to complete:

```c
void vMqttPublisherTask(void *pvParameters) {
    // Wait for ALL 1000ms sensors to complete
    EventBits_t bits = xEventGroupWaitBits(
        sensor_event_group,
        SENSOR_EVENT_1000MS_MASK,  // Wait for Flow + ADC + GPIO I2C + GPIO Discrete
        pdTRUE,                     // Clear bits after reading
        pdTRUE,                     // Wait for ALL bits
        pdMS_TO_TICKS(100)         // 100ms timeout
    );
    
    if ((bits & SENSOR_EVENT_1000MS_MASK) == SENSOR_EVENT_1000MS_MASK) {
        // All sensors completed, safe to publish
        // Take mutex and publish binary + JSON payloads
    }
}
```

### Timing Behavior

**1000ms Sensors** (all must complete before publishing):
- Flow sensors: Read every 1000ms, accumulate over 2000ms
- ADC sensors: Read every 1000ms
- GPIO expander: Read every 1000ms
- Discrete GPIO: Read every 1000ms

**5000ms Sensors** (publish uses available data):
- Environmental sensors: Read every 5000ms
- Temperature sensors: Read every 5000ms
- These update less frequently but are included in every publish cycle

**Timeline Example**:
```
Time    Flow    ADC    GPIO I2C  GPIO Disc  Publisher
0ms     Read    Read   Read      Read       Wait
50ms    -       -      -         -         Wait
100ms   Done    Done   Done      Done       Publish ✅
1000ms  Read    Read   Read      Read       Wait
1050ms  Done    Done   Done      Done       Publish ✅
2000ms  Read    Read   Read      Read       Wait (Flow newData=1 this cycle)
2050ms  Done    Done   Done      Done       Publish ✅
```

### Benefits

- **Data Consistency**: Publisher always has complete data from all 1000ms sensors
- **No Race Conditions**: Event groups ensure sensors complete before publishing
- **Timeout Protection**: 100ms timeout prevents deadlock if sensors are slow
- **Thread-Safe**: Mutex protects `genericSens_` during read/write operations
- **Predictable Timing**: Publisher maintains exactly 1000ms intervals

## genericSens_ Interface Updates

Each sensor manager is responsible for updating the appropriate fields in `genericSens_`:

### PCNT Flow Manager Updates
```c
// Updates flowData1 and flowData2 structures
genericSens_.flowData1.pulses = pulse_delta;
genericSens_.flowData1.milliseconds = time_delta;
// Set newData flag only when 2000ms accumulation complete
genericSens_.flowData1.newData = (elapsed_ms >= 2000 && pulse_delta > 0) ? 1 : 0;
// Same for flowData2
```

### I2C ADC Manager Updates
```c
// Updates ADC channels based on sensor configuration
genericSens_.adc_x1 = pressure_value;      // If channel 0 is pressure
genericSens_.adc_x2 = current_value;       // If channel 1 is current
genericSens_.adc_x3 = sound_value;         // If channel 2 is sound
// ... and so on for adc_x4 through adc_x8
```

### I2C GPIO Manager Updates
```c
// Updates GPIO expander readings
genericSens_.GPIO_x1 = portAValue;  // MCP23X17 Port A
genericSens_.GPIO_x2 = portBValue;  // MCP23X17 Port B
```

### GPIO Discrete Manager Updates
```c
// Updates discrete GPIO inputs
int ioInput = (discInput2 << 1) | discInput1;
genericSens_.gpio_sensor = ioInput;
```

### I2C Environmental Manager Updates
```c
// Updates BME280 readings
genericSens_.tempx = temperature_fahrenheit;
genericSens_.humidity = humidity_percent;
genericSens_.pressurex = pressure_psi;
```

### One-Wire Temperature Manager Updates
```c
// Updates temperature sensors sequentially
genericSens_.temp1 = (int)temperature_fahrenheit;
memcpy(&genericSens_.temp1_f, &temperature_fahrenheit, sizeof(float));
// On next cycle:
genericSens_.temp2 = temperature_fahrenheit;
// ... and so on for temp3 and temp4
```


## File Structure

```
components/sensor_system/
├── include/
│   ├── sensor_manager_types.h      # Shared data structures and types
│   ├── sensor_coordination.h        # Event group coordination
│   ├── pcnt_flow_manager.h          # PCNT flow sensor manager
│   ├── i2c_adc_manager.h           # I2C ADC sensor manager
│   ├── i2c_gpio_manager.h          # I2C GPIO expander manager
│   ├── gpio_discrete_manager.h     # Discrete GPIO manager
│   ├── i2c_env_manager.h           # I2C environmental sensor manager
│   ├── onewire_temp_manager.h      # One-Wire temperature manager
├── src/
│   ├── sensor_manager_types.c      # Shared data implementation
│   ├── sensor_coordination.c       # Event group implementation
│   ├── pcnt_flow_manager.c         # PCNT manager implementation
│   ├── i2c_adc_manager.c           # I2C ADC manager implementation
│   ├── i2c_gpio_manager.c          # I2C GPIO manager implementation
│   ├── gpio_discrete_manager.c     # Discrete GPIO manager implementation
│   ├── i2c_env_manager.c           # I2C environmental manager
│   └── onewire_temp_manager.c      # One-Wire manager
```

## Integration Points

### MQTT Publisher Integration

The MQTT publisher task reads from `genericSens_`:
- Binary payload: `mqtt_publish(..., genericSens_.data_payload, len)`
- JSON payload: Reads individual fields like `genericSens_.tempx`, `genericSens_.humidity`, etc.
- **Coordination**: Waits for all 1000ms sensors to complete via event group before publishing

### System Monitor Integration

The system monitor task can display sensor status:
- Sensor presence flags from each manager
- Last update timestamps
- Error counts per manager

### Error Recovery Integration

Each manager reports errors to the error recovery system:
- I2C timeouts
- Sensor not found errors
- Invalid data errors

## Adding New Sensors

To add a new sensor:

1. **Determine appropriate manager**:
   - Flow sensor → PCNT Flow Manager
   - ADC sensor → I2C ADC Manager
   - GPIO expander → I2C GPIO Manager
   - Discrete GPIO → GPIO Discrete Manager
   - Environmental sensor → I2C Environmental Manager
   - Temperature sensor → One-Wire Temperature Manager
   - Fan control → System Monitor Task (system-level control, not a sensor manager)

2. **Register sensor**:
   ```c
   // In app_main() or initialization function
   i2c_adc_manager_register_sensor(address, channel, type, name, scale_factor);
   ```

3. **Map to genericSens_ field**:
   - Update manager code to write to appropriate `genericSens_` field
   - Ensure field is documented in this architecture

4. **Update event group** (if 1000ms sensor):
   - Add event bit definition
   - Signal completion after reading
   - Update `SENSOR_EVENT_1000MS_MASK`

5. **Update MQTT publisher** (if needed):
   - Add field to JSON payload if not already included

## Benefits of This Architecture

1. **RTOS-Native**: Leverages FreeRTOS task scheduling and priorities
2. **Isolated**: Each sensor type runs in its own task, preventing interference
3. **Flexible**: Easy to add sensors to existing managers
4. **Maintainable**: Clear separation of concerns
5. **Efficient**: Each manager runs at its optimal interval
6. **Thread-Safe**: Mutex protection for shared data
7. **Organized**: Sensors grouped logically by interface/type
8. **Simple**: No complex abstraction layers
9. **Coordinated**: Event groups ensure data consistency before publishing

## Configuration

Sensor manager intervals and priorities are defined in `config.h`:

```c
// Sensor timing constants
#define PCNT_FLOW_TASK_INTERVAL_MS       1000  // Flow sensors (read every 1s, accumulate over 2s)
#define I2C_ADC_TASK_INTERVAL_MS         1000  // ADC sensors
#define I2C_GPIO_TASK_INTERVAL_MS        1000  // I2C GPIO expander
#define GPIO_DISCRETE_TASK_INTERVAL_MS   1000  // Discrete GPIO inputs
#define I2C_ENV_TASK_INTERVAL_MS         5000  // Environmental sensors
#define ONEWIRE_TEMP_TASK_INTERVAL_MS    5000  // Temperature sensors
#define MQTT_PUBLISH_INTERVAL            1000  // MQTT publishing interval
#define MONITOR_INTERVAL_MS              1000  // System monitor task (fan control + monitoring)
```

## Example Usage

```c
// In app_main()
void app_main(void) {
    // ... existing initialization ...
    
    // Initialize sensor data structure and event group
    sensor_data.mutex = xSemaphoreCreateMutex();
    sensor_event_group = xEventGroupCreate();
    
    // Initialize and start PCNT Flow Manager (reads every 1000ms, accumulates over 2000ms)
    pcnt_flow_manager_init();
    pcnt_flow_manager_register_sensor(GPIO_NUM_7, PCNT_UNIT_0, "Flow1");
    pcnt_flow_manager_register_sensor(GPIO_NUM_8, PCNT_UNIT_1, "Flow2");
    pcnt_flow_manager_register_sensor(GPIO_NUM_3, PCNT_UNIT_2, "Flow3");
    pcnt_flow_manager_start_task();
    
    // Initialize and start I2C ADC Manager
    i2c_adc_manager_init();
    i2c_adc_manager_register_sensor(0x48, 0, SENSOR_TYPE_PRESSURE, "Pressure", 100.0);
    i2c_adc_manager_register_sensor(0x48, 1, SENSOR_TYPE_CURRENT, "Current", 50.0);
    i2c_adc_manager_register_sensor(0x48, 2, SENSOR_TYPE_SOUND, "Sound", 1.0);
    i2c_adc_manager_start_task();
    
    // Initialize and start I2C GPIO Manager
    i2c_gpio_manager_init();
    i2c_gpio_manager_register_expander(0x20);  // MCP23X17
    i2c_gpio_manager_start_task();
    
    // Initialize and start GPIO Discrete Manager
    gpio_discrete_manager_init();
    gpio_discrete_manager_register_input(GPIO_NUM_X, "Config1");
    gpio_discrete_manager_register_input(GPIO_NUM_Y, "Config2");
    gpio_discrete_manager_start_task();
    
    // Initialize and start I2C Environmental Manager
    i2c_env_manager_init();
    i2c_env_manager_register_bme280(0x77);
    i2c_env_manager_start_task();
    
    // Initialize and start One-Wire Temperature Manager
    onewire_temp_manager_init(GPIO_NUM_6);
    onewire_temp_manager_scan_devices();
    onewire_temp_manager_start_task();
    
    // Fan control is handled by System Monitor Task (see system_init.c)
    // Initialize fan GPIO pin in System Monitor Task initialization
    // Fan control runs every 1000ms, system monitoring logs every 5000ms
    
    // ... rest of initialization ...
}
```

## Notes

- All sensor managers update the `genericSens_` structure to maintain compatibility with existing MQTT publisher and logging code
- The `data_payload` array shares memory with structured fields (union), allowing direct binary MQTT transmission
- Mutex protection ensures thread-safe concurrent access from multiple manager tasks
- Each manager handles its own error recovery and logging
- Watchdog registration ensures system stability if a manager task hangs
- **Flow sensors read every 1000ms but accumulate pulses over 2000ms for optimal accuracy** - the `newData` flag indicates when a complete 2000ms accumulation is ready
- **Publishing coordination**: MQTT publisher waits for all 1000ms sensors (Flow, ADC, GPIO I2C, GPIO Discrete) to complete before publishing
- Event groups provide coordination between sensor managers and publisher
- Publisher publishes every 1000ms regardless of flow sensor `newData` flag (ensures consistent publishing interval)
- **Fan Control**: Implemented in System Monitor Task, reads temperature from `genericSens_.tempx` every 1000ms (read-only consumer, doesn't participate in event group coordination)
- **System Monitor Task**: Runs every 1000ms for fan control, logs system status every 5000ms

## Deferred Features

The following features are documented in `FUTURE_UPGRADES.md`:
- **Time Management**: Built-in RTC with SNTP (replaces external I2C RTC) - *Note: Already implemented, but full SNTP integration deferred*
- **Data Logging**: HTTP POST to local server (recommended), MQTT logging, or W25Q128 SPI flash storage options
- **I2C Buzzer Manager**: Audio feedback and alerts for WiFi/MQTT status and system errors
