# Implementation Strategy: SensorX ESP32-C6 System

## Overview

This document provides a comprehensive, methodical implementation strategy for the SensorX ESP32-C6 system based on the design documents in `notes/Design Notes/`. The strategy is organized into phases, ensuring each component can be implemented, tested, and validated before moving to the next.

## Implementation Philosophy

**Slow and Methodical**: Each phase builds upon the previous, ensuring stability before adding complexity.

**Testable**: Each component includes unit tests and integration tests before proceeding.

**Incremental**: Implement one manager at a time, validate fully, then move to the next.

**Non-Breaking**: Existing functionality remains operational throughout implementation.

## Prerequisites and Foundation

### Current System State

**Already Implemented**:
- ✅ WiFi Manager (working)
- ✅ MQTT Manager (working)
- ✅ I2C Manager (working, STEMMA QT power enabled)
- ✅ Watchdog System (working)
- ✅ Error Recovery System (working)
- ✅ System Monitor Task (basic monitoring)
- ✅ Heartbeat Task (working)
- ✅ System Info Task (working)
- ✅ Sensor Acquisition Task (legacy, to be replaced)
- ✅ MQTT Publisher Task (working)
- ✅ GPIO pin definitions (`pins.h`)

**Infrastructure Ready**:
- ✅ FreeRTOS runtime statistics enabled
- ✅ Mutex system available
- ✅ Event groups available
- ✅ Task priorities defined
- ✅ Stack sizes defined

### Missing Infrastructure

**To Be Created**:
1. **Sensor Data Structure (`genericSens_`)**
   - Bit-packed FlowData structure
   - All sensor fields defined
   - Union with `data_payload` array
   - Mutex protection

2. **Sensor Coordination System**
   - Event group for sensor completion signaling
   - Event bit definitions
   - Coordination logic

3. **Common Sensor Manager Interface**
   - Initialization pattern
   - Task creation pattern
   - Error handling pattern
   - Watchdog registration pattern

## Phase-by-Phase Implementation Strategy

---

## Phase 0: Foundation Infrastructure (Prerequisites)

**Goal**: Establish the foundation that all sensor managers will depend on.

**Components**:
1. Sensor data structure (`genericSens_`)
2. Sensor coordination system (event groups)
3. Common headers and utilities

**Files to Create**:
- `components/sensor_system/include/sensor_coordination.h`
- `components/sensor_system/include/sensor_manager_types.h` (or extend `types.h`)
- `components/sensor_system/src/sensor_coordination.c`

**Files to Modify**:
- `components/sensor_system/include/types.h` - Add FlowData structure, extend SensorData
- `components/sensor_system/include/config.h` - Add sensor manager constants
- `main/main.c` - Initialize sensor data structure and event group

**Testing**:
- ✅ Verify `genericSens_` structure compiles
- ✅ Verify mutex creation succeeds
- ✅ Verify event group creation succeeds
- ✅ Verify structure size matches expected (for MQTT payload)
- ✅ Test mutex locking/unlocking
- ✅ Test event group signaling/waiting

**Success Criteria**:
- Sensor data structure compiles without errors
- Mutex and event group initialize successfully
- No memory leaks
- Structure size documented

**Estimated Time**: 1-2 days

---

## Phase 1: Fan Control Enhancement (System Monitor Task)

**Goal**: Add fan control logic to existing System Monitor Task.

**Why First**:
- Modifies existing task (low risk)
- No new tasks created
- Builds on existing infrastructure
- Provides immediate value
- Works with die temperature fallback (no BME280 required initially)
- Can be enhanced later when BME280 is available

**Components**:
1. Fan control GPIO initialization
2. Temperature reading logic (primary + fallback)
3. Fan control logic (threshold comparison)
4. System monitor output enhancement

**Files to Modify**:
- `components/sensor_system/src/system_init.c` - Add fan control to `vSystemMonitorTask`
- `components/sensor_system/include/config.h` - Add fan control constants

**Dependencies**:
- Phase 0 complete (sensor coordination structure available for future BME280 integration)

**Implementation Steps**:
1. Add fan control GPIO initialization in `vSystemMonitorTask`
2. Add temperature reading logic (BME280 primary - will be unavailable initially, die temp fallback)
3. Add fan control threshold logic
4. Add fan status to system monitor output
5. Add temperature validity checks
6. Add error handling

**Testing**:
- ✅ Unit test: GPIO configuration
- ✅ Unit test: Temperature reading (simulated)
- ✅ Unit test: Fallback logic (simulate BME280 unavailable - this will be the initial state)
- ✅ Unit test: Threshold comparison
- ✅ Integration test: Fan control with die temperature fallback (initial implementation)
- ✅ Integration test: Fan control with both sources unavailable
- ✅ Hardware test: Verify fan turns ON/OFF correctly using die temperature
- ✅ Hardware test: Measure fan control response time
- ✅ System test: Verify no impact on system monitoring
- ✅ Future test: Verify fan control with BME280 when Phase 6 is complete

**Hardware Required**:
- GPIO21 configured for fan control output (changed from GPIO16 to avoid UART conflict)
- Fan connected to GPIO21
- BME280 sensor (optional - will be added in Phase 6)

**Success Criteria**:
- Fan control runs every 1000ms
- Temperature read from die temperature (BME280 unavailable initially)
- Falls back gracefully when no temperature source available
- Fan turns ON/OFF based on threshold
- Fan status displayed in system monitor output
- No impact on system monitoring function
- Ready to integrate BME280 temperature when Phase 6 is complete

**Estimated Time**: 2-3 days

---

## Phase 2: One-Wire Temperature Manager (DS18B20)

**Goal**: Implement One-Wire temperature sensor manager.

**Why Second**:
- Different protocol (One-Wire, not I2C)
- Independent of I2C bus (doesn't conflict with later I2C managers)
- Long read times (750ms per sensor) - good to establish early
- Cooperative multitasking required
- 5000ms interval (different pattern from 1000ms sensors)

**Components**:
1. One-Wire temperature manager (`onewire_temp_manager.h/c`)
2. One-Wire protocol implementation (or library)
3. DS18B20 driver
4. Task implementation with cooperative multitasking
5. Integration with `genericSens_`

**Files to Create**:
- `components/sensor_system/include/onewire_temp_manager.h`
- `components/sensor_system/src/onewire_temp_manager.c`
- Possibly: `components/sensor_system/include/onewire.h` (if creating protocol)
- Possibly: `components/sensor_system/include/ds18b20.h` (if creating driver)

**Files to Modify**:
- `main/main.c` - Initialize and start One-Wire temperature manager
- `components/sensor_system/include/config.h` - Add One-Wire constants
- `components/sensor_system/src/mqtt_publisher.c` - Include One-Wire temperature data

**Dependencies**:
- Phase 0 complete (sensor coordination)
- GPIO6 available for One-Wire bus

**Implementation Steps**:
1. Research/create One-Wire protocol implementation
2. Research/create DS18B20 driver
3. Create header file with function prototypes
4. Implement initialization function (GPIO pin setup)
5. Implement device scanning function (ROM code discovery)
6. Implement task function:
   - Read all 4 sensors sequentially
   - Yield between sensors (20ms)
   - Update `genericSens_.temp1` through `temp4`
7. Add event group signaling (SENSOR_EVENT_TEMP_COMPLETE) - optional for 5000ms
8. Integrate into `main.c`
9. Update MQTT publisher

**Testing**:
- ✅ Unit test: One-Wire protocol (reset, write, read)
- ✅ Unit test: DS18B20 communication
- ✅ Unit test: Device scanning
- ✅ Unit test: Temperature conversion
- ✅ Unit test: CRC verification
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Task interval (5000ms)
- ✅ Integration test: All 4 sensors read in one cycle
- ✅ Integration test: Yield points work correctly
- ✅ Hardware test: Connect DS18B20 sensors, verify discovery
- ✅ Hardware test: Verify temperature readings
- ✅ Hardware test: Test with 1, 2, 3, 4 sensors
- ✅ System test: Verify no blocking of other tasks

**Hardware Required**:
- DS18B20 sensors on One-Wire bus (GPIO6)
- Pull-up resistor (4.7kΩ)

**Success Criteria**:
- One-Wire temperature manager task runs every 5000ms
- All sensors discovered and read correctly
- All 4 sensors read within 5000ms cycle
- `genericSens_.temp1` through `temp4` update correctly
- Cooperative multitasking works (other tasks not blocked)
- MQTT publisher includes temperature data

**Estimated Time**: 5-6 days (most complex due to One-Wire protocol)

---

## Phase 3: PCNT Flow Manager (Hardware-Based, Critical Priority)

**Goal**: Implement PCNT-based flow sensor manager.

**Why Third**:
- Hardware-intensive (PCNT configuration)
- Higher priority task (critical)
- Requires careful timing and pulse loss minimization
- Establishes pattern for hardware-intensive managers
- Independent of I2C bus (doesn't conflict with I2C managers)

**Components**:
1. PCNT flow manager (`pcnt_flow_manager.h/c`)
2. PCNT hardware configuration
3. Flow sensor registration
4. Task implementation with 2000ms accumulation logic
5. Integration with `genericSens_`

**Files to Create**:
- `components/sensor_system/include/pcnt_flow_manager.h`
- `components/sensor_system/src/pcnt_flow_manager.c`

**Files to Modify**:
- `main/main.c` - Initialize and start PCNT flow manager
- `components/sensor_system/include/config.h` - Add PCNT flow constants
- `components/sensor_system/src/mqtt_publisher.c` - Include flow data

**Dependencies**:
- Phase 0 complete (sensor coordination)

**Implementation Steps**:
1. Create header file with function prototypes
2. Implement PCNT initialization function
3. Implement sensor registration function (GPIO pin, PCNT unit)
4. Implement PCNT configuration (edge detection, filter, limits)
5. Implement task function:
   - Read PCNT counter (pause-based read)
   - Calculate elapsed time
   - Check 2000ms accumulation window
   - Update `genericSens_.flowData1/2/3`
   - Reset counter if window complete
6. Add event group signaling (SENSOR_EVENT_FLOW_COMPLETE)
7. Integrate into `main.c`
8. Update MQTT publisher

**Testing**:
- ✅ Unit test: PCNT initialization
- ✅ Unit test: PCNT configuration (edge, filter, limits)
- ✅ Unit test: Counter read/reset operations
- ✅ Unit test: 2000ms accumulation logic
- ✅ Unit test: Pulse loss minimization (timing test)
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Task interval (1000ms)
- ✅ Integration test: 2000ms accumulation window
- ✅ Hardware test: Connect flow sensor, verify pulse counting
- ✅ Hardware test: Verify pulse count accuracy
- ✅ Hardware test: Verify `newData` flag behavior
- ✅ Hardware test: Measure pulse loss during reads
- ✅ System test: Verify no interference with other tasks (critical priority)

**Hardware Required**:
- 3 flow sensors connected to GPIO7, GPIO8, GPIO3
- Flow sensor signal generation (or actual flow)

**Success Criteria**:
- PCNT flow manager task runs every 1000ms
- All 3 sensors read correctly
- Pulse counts accumulate over 2000ms window
- `newData` flag set correctly (every other read)
- `genericSens_.flowData1/2/3` update correctly
- Event group signals completion
- MQTT publisher includes flow data
- Pulse loss < 0.1% (negligible)
- No interference with other tasks

**Estimated Time**: 4-5 days

---

## Phase 4: I2C GPIO Manager (MCP23X17 GPIO Expander)

**Goal**: Implement I2C GPIO expander manager for reading MCP23X17.

**Why Fourth**:
- First I2C manager implementation
- Uses existing I2C infrastructure
- Establishes pattern for other I2C managers
- Useful for testing I2C bus before more complex I2C sensors

**Components**:
1. I2C GPIO manager (`i2c_gpio_manager.h/c`)
2. MCP23X17 driver/interface
3. Task implementation
4. Integration with `genericSens_`

**Files to Create**:
- `components/sensor_system/include/i2c_gpio_manager.h`
- `components/sensor_system/src/i2c_gpio_manager.c`
- Possibly: `components/sensor_system/include/mcp23x17.h` (if creating driver)

**Files to Modify**:
- `main/main.c` - Initialize and start I2C GPIO manager
- `components/sensor_system/include/config.h` - Add I2C GPIO constants
- `components/sensor_system/src/mqtt_publisher.c` - Include GPIO expander data

**Implementation Steps**:
1. Research/create MCP23X17 driver (or use existing library)
2. Create header file with function prototypes
3. Implement initialization function (I2C device setup)
4. Implement expander registration function
5. Implement task function (read Port A/B, update `genericSens_.GPIO_x1`, `GPIO_x2`)
6. Add event group signaling (SENSOR_EVENT_GPIO_I2C_COMPLETE)
7. Integrate into `main.c`
8. Update MQTT publisher

**Testing**:
- ✅ Unit test: MCP23X17 communication
- ✅ Unit test: Port A/B read operations
- ✅ Unit test: `genericSens_` update
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Task interval (1000ms)
- ✅ Hardware test: Read actual MCP23X17 device
- ✅ Hardware test: Verify Port A/B values
- ✅ System test: Verify no I2C bus conflicts

**Hardware Required**:
- MCP23X17 GPIO expander on I2C bus (address 0x20 typically)
- I2C bus operational (already working)

**Success Criteria**:
- I2C GPIO manager task runs every 1000ms
- MCP23X17 Port A/B read correctly
- `genericSens_.GPIO_x1` and `GPIO_x2` update correctly
- Event group signals completion
- MQTT publisher includes GPIO expander data
- No I2C bus conflicts

**Estimated Time**: 3-4 days

---

## Phase 5: I2C ADC Manager (ADS1015/ADS1115)

**Goal**: Implement I2C ADC manager for reading analog sensors.

**Why Fifth**:
- Second I2C manager implementation
- Uses I2C infrastructure
- Multiple sensors (pressure, current, sound)
- Builds on I2C GPIO manager experience (Phase 4)

**Components**:
1. I2C ADC manager (`i2c_adc_manager.h/c`)
2. ADS1015/ADS1115 driver/interface
3. Sensor registration system
4. Task implementation
5. Integration with `genericSens_`

**Files to Create**:
- `components/sensor_system/include/i2c_adc_manager.h`
- `components/sensor_system/src/i2c_adc_manager.c`
- Possibly: `components/sensor_system/include/ads1015.h` (if creating driver)

**Files to Modify**:
- `main/main.c` - Initialize and start I2C ADC manager
- `components/sensor_system/include/config.h` - Add I2C ADC constants
- `components/sensor_system/src/mqtt_publisher.c` - Include ADC data

**Dependencies**:
- Phase 0 complete (sensor coordination)
- Phase 4 complete (I2C GPIO manager - establishes I2C manager pattern)
- I2C bus operational

**Implementation Steps**:
1. Research/create ADS1015/ADS1115 driver
2. Create header file with function prototypes
3. Implement initialization function
4. Implement sensor registration function (address, channel, type, scale factor)
5. Implement task function:
   - Read all registered ADC sensors
   - Apply scale factors
   - Update `genericSens_.adc_x1` through `adc_x8`
6. Add event group signaling (SENSOR_EVENT_ADC_COMPLETE)
7. Integrate into `main.c`
8. Update MQTT publisher

**Testing**:
- ✅ Unit test: ADS1015/ADS1115 communication
- ✅ Unit test: Channel read operations
- ✅ Unit test: Scale factor application
- ✅ Unit test: Multi-sensor registration
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Task interval (1000ms)
- ✅ Hardware test: Read actual ADC channels
- ✅ Hardware test: Verify scale factor accuracy
- ✅ Hardware test: Test with multiple sensors

**Hardware Required**:
- ADS1015 or ADS1115 ADC on I2C bus
- Analog sensors connected to ADC channels

**Success Criteria**:
- I2C ADC manager task runs every 1000ms
- ADC channels read correctly
- Scale factors applied correctly
- `genericSens_.adc_x1` through `adc_x8` update correctly
- Event group signals completion
- MQTT publisher includes ADC data

**Estimated Time**: 3-4 days

---

## Phase 6: I2C Environmental Manager (BME280)

**Goal**: Implement I2C environmental sensor manager for BME280.

**Why Sixth**:
- Third I2C manager implementation
- Enhances fan control with ambient temperature (Phase 1 enhancement)
- Uses I2C infrastructure
- Multiple sensor values (temp, humidity, pressure)
- 5000ms interval (different from 1000ms sensors)
- Builds on I2C manager patterns from Phases 4 and 5

**Components**:
1. I2C environmental manager (`i2c_env_manager.h/c`)
2. BME280 driver/interface
3. Task implementation
4. Integration with `genericSens_`

**Files to Create**:
- `components/sensor_system/include/i2c_env_manager.h`
- `components/sensor_system/src/i2c_env_manager.c`
- Possibly: `components/sensor_system/include/bme280.h` (if creating driver)

**Files to Modify**:
- `main/main.c` - Initialize and start I2C environmental manager
- `components/sensor_system/include/config.h` - Add I2C env constants
- `components/sensor_system/src/mqtt_publisher.c` - Include environmental data

**Dependencies**:
- Phase 0 complete (sensor coordination)
- Phase 1 complete (fan control - will now use BME280 temperature as primary source)
- Phase 4 complete (I2C GPIO manager - establishes I2C manager pattern)
- Phase 5 complete (I2C ADC manager - establishes multi-sensor I2C pattern)
- I2C bus operational

**Implementation Steps**:
1. Research/create BME280 driver
2. Create header file with function prototypes
3. Implement initialization function
4. Implement BME280 registration function
5. Implement task function:
   - Read temperature, humidity, pressure
   - Convert units (Celsius to Fahrenheit, etc.)
   - Update `genericSens_.tempx`, `humidity`, `pressurex`
6. Add event group signaling (SENSOR_EVENT_ENV_COMPLETE) - optional for 5000ms
7. Integrate into `main.c`
8. Update MQTT publisher

**Testing**:
- ✅ Unit test: BME280 communication
- ✅ Unit test: Temperature read
- ✅ Unit test: Humidity read
- ✅ Unit test: Pressure read
- ✅ Unit test: Unit conversions
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Task interval (5000ms)
- ✅ Integration test: Fan control reads temperature correctly (enhances Phase 1)
- ✅ Hardware test: Read actual BME280 sensor
- ✅ Hardware test: Verify temperature accuracy
- ✅ Hardware test: Verify humidity accuracy
- ✅ Hardware test: Verify pressure accuracy
- ✅ Integration test: Fan control switches from die temp to BME280 temp

**Hardware Required**:
- BME280 sensor on I2C bus (address 0x77 typically)

**Success Criteria**:
- I2C environmental manager task runs every 5000ms
- Temperature, humidity, pressure read correctly
- `genericSens_.tempx`, `humidity`, `pressurex` update correctly
- Fan control (Phase 1) now uses BME280 temperature as primary source
- MQTT publisher includes environmental data

**Estimated Time**: 3-4 days

---

## Phase 7: GPIO Discrete Manager (Direct GPIO Inputs)

**Goal**: Implement the simplest sensor manager (direct GPIO reads) to establish the pattern.

**Why Last of I2C Sequence**:
- Simple implementation (direct GPIO, no I2C)
- Useful for discrete inputs and configuration switches
- Completes the sensor manager suite
- Establishes pattern for simple GPIO-based sensors

**Components**:
1. GPIO discrete manager (`gpio_discrete_manager.h/c`)
2. GPIO pin configuration
3. Task implementation
4. Integration with `genericSens_`

**Files to Create**:
- `components/sensor_system/include/gpio_discrete_manager.h`
- `components/sensor_system/src/gpio_discrete_manager.c`

**Files to Modify**:
- `main/main.c` - Initialize and start GPIO discrete manager
- `components/sensor_system/include/config.h` - Add GPIO discrete constants
- `components/sensor_system/src/mqtt_publisher.c` - Include GPIO discrete data in publish

**Dependencies**:
- Phase 0 complete (sensor coordination)

**Implementation Steps**:
1. Create header file with function prototypes
2. Implement initialization function (GPIO pin setup)
3. Implement sensor registration function
4. Implement task function (read GPIO pins, update `genericSens_.gpio_sensor`)
5. Implement task creation function
6. Add event group signaling (SENSOR_EVENT_GPIO_DISC_COMPLETE)
7. Integrate into `main.c`
8. Update MQTT publisher to include GPIO data

**Testing**:
- ✅ Unit test: GPIO pin configuration
- ✅ Unit test: GPIO read operation
- ✅ Unit test: `genericSens_` update
- ✅ Unit test: Mutex protection
- ✅ Unit test: Event group signaling
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Task interval (1000ms)
- ✅ Integration test: Watchdog registration
- ✅ Hardware test: Read actual GPIO pins (GPIO0, GPIO1, GPIO2)
- ✅ System test: Verify no interference with other tasks

**Hardware Required**:
- GPIO0, GPIO1, GPIO2 pins configured as inputs
- Pull-up resistors or external pull-ups

**Success Criteria**:
- GPIO discrete manager task runs every 1000ms
- GPIO values read correctly
- `genericSens_.gpio_sensor` updates correctly
- Event group signals completion
- MQTT publisher includes GPIO data
- No memory leaks
- Watchdog doesn't trigger

**Estimated Time**: 2-3 days

---

## Phase 8: Publishing Coordination Enhancement

**Goal**: Implement event group coordination for MQTT publishing.

**Why Eighth**:
- All 1000ms sensors must be implemented first
- Coordinates multiple sensor managers
- Ensures data consistency before publishing

**Components**:
1. Event group coordination logic
2. MQTT publisher enhancement
3. Sensor manager completion signaling

**Files to Modify**:
- `components/sensor_system/src/mqtt_publisher.c` - Add event group waiting
- `components/sensor_system/src/pcnt_flow_manager.c` - Add event signaling
- `components/sensor_system/src/i2c_adc_manager.c` - Add event signaling
- `components/sensor_system/src/i2c_gpio_manager.c` - Add event signaling
- `components/sensor_system/src/gpio_discrete_manager.c` - Add event signaling

**Dependencies**:
- Phase 0 complete (event group created)
- Phase 3 complete (PCNT flow manager)
- Phase 4 complete (I2C GPIO manager)
- Phase 5 complete (I2C ADC manager)
- Phase 7 complete (GPIO discrete manager)

**Implementation Steps**:
1. Review event group bit definitions
2. Add event signaling to each 1000ms sensor manager
3. Modify MQTT publisher to wait for all 1000ms sensors
4. Add timeout handling
5. Add logging for coordination debugging

**Testing**:
- ✅ Unit test: Event group signaling
- ✅ Unit test: Event group waiting (all bits)
- ✅ Unit test: Timeout handling
- ✅ Integration test: All sensors signal completion
- ✅ Integration test: Publisher waits correctly
- ✅ Integration test: Publisher timing (1000ms interval maintained)
- ✅ System test: Verify data consistency in MQTT messages

**Success Criteria**:
- All 1000ms sensors signal completion
- MQTT publisher waits for all sensors
- Data published is consistent (all sensors updated)
- Publishing interval maintained (1000ms)
- No missed publishes due to timeouts

**Estimated Time**: 2-3 days

**Note**: Phase 2 (One-Wire) and Phase 6 (BME280) are 5000ms sensors and don't participate in the 1000ms event group coordination, but their data is still included in MQTT publishes.

---

## Phase 9: Boot Button Task Monitor

**Goal**: Implement boot button-triggered task monitor.

**Why Ninth**:
- Non-critical feature
- Useful for debugging and monitoring
- Can be implemented independently
- No dependencies on sensor managers

**Components**:
1. Boot button task monitor (`task_monitor.h/c`)
2. GPIO interrupt or polling
3. Task statistics gathering
4. Custom timing tracking (if needed)
5. Formatted output

**Files to Create**:
- `components/sensor_system/include/task_monitor.h`
- `components/sensor_system/src/task_monitor.c`

**Files to Modify**:
- `main/main.c` - Initialize and start task monitor
- `components/sensor_system/include/config.h` - Add task monitor constants
- Possibly: `components/sensor_system/include/freertos/FreeRTOSConfig.h` - Enable hooks if needed

**Dependencies**:
- Phase 0 complete (for consistency)
- FreeRTOS runtime statistics enabled (already done)

**Implementation Steps**:
1. Create header file with function prototypes
2. Implement GPIO button configuration
3. Implement button debounce logic
4. Implement task statistics gathering function
5. Implement custom timing tracking (if using hooks)
6. Implement formatted output function
7. Implement task function (polling or interrupt-driven)
8. Integrate into `main.c`

**Testing**:
- ✅ Unit test: Button detection
- ✅ Unit test: Debounce logic
- ✅ Unit test: Task statistics gathering
- ✅ Unit test: Timing statistics calculation
- ✅ Unit test: Output formatting
- ✅ Integration test: Task creation and execution
- ✅ Integration test: Button press triggers display
- ✅ Hardware test: Press boot button, verify output
- ✅ System test: Verify no impact on other tasks (idle priority)

**Hardware Required**:
- Boot button (GPIO9, already available)

**Success Criteria**:
- Task monitor task runs at idle priority
- Button press triggers statistics display
- All task information displayed correctly
- Timing statistics accurate (min/nominal/max)
- Stack usage displayed correctly
- No interference with other tasks

**Estimated Time**: 4-5 days

---

## Phase 10: OTA Implementation

**Goal**: Implement HTTP OTA firmware update capability.

**Why Last**:
- Complex feature with system-wide impact
- Requires stable system to test updates
- Can be tested independently
- Optional feature (system works without it)

**Components**:
1. OTA manager (`ota_manager.h/c`)
2. HTTP client setup
3. OTA partition management
4. Background update checking task
5. Integration with error recovery

**Files to Create**:
- `components/sensor_system/include/ota_manager.h`
- `components/sensor_system/src/ota_manager.c`

**Files to Modify**:
- `main/main.c` - Initialize OTA manager
- `components/sensor_system/src/wifi_manager.c` - Initialize OTA after WiFi connection
- `sdkconfig` - Configure partition table for OTA
- `components/sensor_system/include/config.h` - Add OTA constants

**Dependencies**:
- Phase 0 complete (for consistency)
- WiFi manager operational
- Partition table configured for OTA

**Implementation Steps**:
1. Configure partition table (OTA_0, OTA_1 partitions)
2. Create header file with function prototypes
3. Implement initialization function
4. Implement update check function
5. Implement download function
6. Implement write function
7. Implement reboot function
8. Implement background task for periodic checking
9. Integrate into `main.c` and `wifi_manager.c`

**Testing**:
- ✅ Unit test: Partition table configuration
- ✅ Unit test: HTTP client setup
- ✅ Unit test: Version checking logic
- ✅ Unit test: Download progress tracking
- ✅ Unit test: Write validation
- ✅ Integration test: OTA initialization
- ✅ Integration test: Background update checking
- ✅ System test: Download and install test firmware
- ✅ System test: Verify rollback on failure
- ✅ System test: Verify no impact on sensor operations

**Hardware Required**:
- WiFi connection (already working)
- HTTP server for firmware hosting

**Success Criteria**:
- OTA manager initializes after WiFi connection
- Update checking works correctly
- Firmware download succeeds
- Firmware write succeeds
- System reboots to new firmware
- Rollback works on failure
- No impact on sensor operations during download

**Estimated Time**: 5-6 days

---

## Testing Strategy

### Unit Testing Approach

**For Each Manager**:
1. **Hardware Interface Tests**: Test hardware communication independently
2. **Data Structure Tests**: Test `genericSens_` updates
3. **Mutex Tests**: Test thread-safe access
4. **Event Group Tests**: Test completion signaling
5. **Error Handling Tests**: Test failure scenarios

### Integration Testing Approach

**For Each Manager**:
1. **Task Creation Test**: Verify task creates successfully
2. **Task Execution Test**: Verify task runs at correct interval
3. **Watchdog Test**: Verify watchdog registration
4. **Resource Test**: Verify no memory leaks
5. **Timing Test**: Verify task completes within interval

### Hardware Testing Approach

**For Each Manager**:
1. **Basic Functionality**: Verify sensors read correctly
2. **Accuracy Test**: Compare readings with reference
3. **Reliability Test**: Long-term operation
4. **Failure Test**: Remove sensor, verify error handling

### System Testing Approach

**After Each Phase**:
1. **Stability Test**: Run for extended period
2. **Resource Test**: Monitor memory usage
3. **Performance Test**: Verify no degradation
4. **Integration Test**: Verify all components work together

---

## Risk Mitigation

### Implementation Risks

1. **Risk**: Sensor manager breaks existing functionality
   - **Mitigation**: Implement one manager at a time, test thoroughly before next
   - **Mitigation**: Keep existing sensor acquisition task until all managers working

2. **Risk**: Event group coordination fails
   - **Mitigation**: Test event group independently before integrating
   - **Mitigation**: Add extensive logging for debugging

3. **Risk**: Mutex deadlock
   - **Mitigation**: Always use timeouts on mutex operations
   - **Mitigation**: Keep mutex hold times minimal
   - **Mitigation**: Test concurrent access scenarios

4. **Risk**: Task priority conflicts
   - **Mitigation**: Use defined priority constants
   - **Mitigation**: Test task preemption scenarios
   - **Mitigation**: Monitor task execution times

5. **Risk**: Memory leaks
   - **Mitigation**: Use static allocation where possible
   - **Mitigation**: Monitor heap usage over time
   - **Mitigation**: Test long-term operation

### Testing Risks

1. **Risk**: Hardware not available for testing
   - **Mitigation**: Use simulated sensors where possible
   - **Mitigation**: Test hardware interface independently
   - **Mitigation**: Document hardware requirements clearly

2. **Risk**: Integration issues not discovered until late
   - **Mitigation**: Integrate incrementally
   - **Mitigation**: Test integration after each phase
   - **Mitigation**: Use integration test checklist

---

## Implementation Checklist Template

For each phase, use this checklist:

### Pre-Implementation
- [ ] Review design document thoroughly
- [ ] Identify all dependencies
- [ ] Verify dependencies are complete
- [ ] Review existing code for integration points
- [ ] Plan file structure

### Implementation
- [ ] Create header file
- [ ] Implement initialization function
- [ ] Implement core functionality
- [ ] Implement task function
- [ ] Add error handling
- [ ] Add logging
- [ ] Add watchdog registration
- [ ] Integrate into main.c

### Testing
- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Hardware tests pass (if applicable)
- [ ] System tests pass
- [ ] No memory leaks
- [ ] No compiler warnings
- [ ] Documentation updated

### Post-Implementation
- [ ] Code review (self-review)
- [ ] Performance validated
- [ ] Resource usage validated
- [ ] Integration with other components validated
- [ ] Ready for next phase

---

## Dependencies Matrix

```
Phase 0: Foundation
  └─> Required by: All phases

Phase 1: Fan Control
  └─> Requires: Phase 0
  └─> Enhanced by: Phase 6 (BME280 temperature becomes primary source)

Phase 2: One-Wire Temperature Manager
  └─> Requires: Phase 0
  └─> Independent: No dependencies on other managers

Phase 3: PCNT Flow Manager
  └─> Requires: Phase 0
  └─> Required by: Phase 8 (Publishing Coordination)

Phase 4: I2C GPIO Manager
  └─> Requires: Phase 0, I2C Manager (already exists)
  └─> Required by: Phase 8 (Publishing Coordination)
  └─> Establishes pattern for: Phase 5, Phase 6

Phase 5: I2C ADC Manager
  └─> Requires: Phase 0, Phase 4 (I2C manager pattern), I2C Manager (already exists)
  └─> Required by: Phase 8 (Publishing Coordination)

Phase 6: I2C Environmental Manager
  └─> Requires: Phase 0, Phase 4 (I2C manager pattern), I2C Manager (already exists)
  └─> Enhances: Phase 1 (Fan Control - provides primary temperature source)

Phase 7: GPIO Discrete Manager
  └─> Requires: Phase 0
  └─> Required by: Phase 8 (Publishing Coordination)

Phase 8: Publishing Coordination
  └─> Requires: Phase 0, Phase 3 (PCNT), Phase 4 (I2C GPIO), Phase 5 (I2C ADC), Phase 7 (GPIO Discrete)
  └─> Modifies: MQTT Publisher (already exists)
  └─> Note: Phase 2 (One-Wire) and Phase 6 (BME280) are 5000ms sensors, don't participate in event group

Phase 9: Boot Button Task Monitor
  └─> Requires: Phase 0 (for consistency)
  └─> Independent: No dependencies on sensor managers

Phase 10: OTA Implementation
  └─> Requires: Phase 0, WiFi Manager (already exists)
  └─> Independent: No dependencies on sensor managers
```

---

## Recommended Implementation Order

Based on user preference and dependencies:

1. **Phase 0: Foundation** (Week 1)
2. **Phase 1: Fan Control** (Week 1-2)
   - Works with die temperature fallback initially
   - Will be enhanced when Phase 6 (BME280) is complete
3. **Phase 2: One-Wire Temperature Manager** (Week 2-3)
   - Independent protocol, doesn't conflict with I2C
   - Establishes cooperative multitasking pattern
4. **Phase 3: PCNT Flow Manager** (Week 3-4)
   - Hardware-intensive, critical priority
   - Independent of I2C bus
5. **Phase 4: I2C GPIO Manager** (Week 4-5)
   - First I2C manager
   - Establishes I2C manager pattern for Phases 5 and 6
6. **Phase 5: I2C ADC Manager** (Week 5-6)
   - Second I2C manager
   - Multi-sensor I2C pattern
7. **Phase 6: I2C Environmental Manager** (Week 6-7)
   - Third I2C manager
   - Enhances Phase 1 (Fan Control) with BME280 temperature
8. **Phase 7: GPIO Discrete Manager** (Week 7)
   - Completes sensor manager suite
   - Simple GPIO-based manager
9. **Phase 8: Publishing Coordination** (Week 7-8)
   - Coordinates all 1000ms sensors
   - Requires Phases 3, 4, 5, 7 complete
10. **Phase 9: Boot Button Task Monitor** (Week 8, can be parallel)
    - Debugging tool
    - Independent feature
11. **Phase 10: OTA Implementation** (Week 8-9, can be parallel)
    - Optional feature
    - Independent feature

**Total Estimated Time**: 8-9 weeks (assuming methodical pace with thorough testing)

**Key Benefits of This Order**:
- Fan control provides immediate value (works with die temp)
- One-Wire and PCNT don't interfere with I2C implementation
- I2C managers build on each other (pattern establishment)
- Fan control enhanced naturally when BME280 is added

---

## Key Success Factors

1. **Incremental Progress**: Complete one phase fully before starting the next
2. **Thorough Testing**: Test each component independently before integration
3. **Documentation**: Document any deviations from design documents
4. **Code Review**: Self-review code before moving to next phase
5. **Hardware Validation**: Test with actual hardware when available
6. **Performance Monitoring**: Monitor resource usage throughout
7. **Error Handling**: Implement robust error handling from the start
8. **Logging**: Add comprehensive logging for debugging

---

## Notes

- **Legacy Code**: The existing `sensor_acquisition.c` task can remain until all managers are implemented and tested
- **Parallel Development**: Phases 9 and 10 can be developed in parallel after Phase 8
- **Hardware Availability**: Some phases may be delayed if hardware is not available - use simulation where possible
- **Design Documents**: Refer to design documents for detailed implementation guidance
- **Configuration**: All timing constants and thresholds should be in `config.h` for easy adjustment

This strategy provides a clear, methodical path forward that ensures each component is thoroughly tested before moving to the next, minimizing risk and ensuring system stability throughout the implementation process.

