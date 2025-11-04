# ESP32 Migration Design Goals

## Overview
This document outlines the design goals for migrating the ESP8266-based sensor system to ESP32 with FreeRTOS architecture. The migration aims to leverage ESP32's superior processing power, memory, and FreeRTOS capabilities while maintaining all existing functionality.

**Language Specification**: This project uses classic C language within the Arduino ecosystem. All implementation files use .c/.h extensions with C-style programming constructs, avoiding C++ features like classes, templates, and operator overloading.

### C Language Programming Practices

**Header File Conventions:**
- Function prototypes with parameter names for documentation
- Extern declarations for global variables shared between modules
- Typedef structures for complex data types
- Include guards using `#ifndef/#define/#endif` pattern
- Header-only declarations, no function implementations

**Implementation Practices:**
- Static functions for module-private functions
- Global variables with `extern` declarations in headers
- Function pointers for callbacks and driver interfaces
- Struct-based data organization instead of classes
- Error codes as return values instead of exceptions

**Arduino C Integration:**
- `.ino` file serves as main entry point with Arduino-specific functions
- Standard Arduino libraries used through `#include <LibraryName.h>`
- ESP32 FreeRTOS API accessed through standard C function calls
- Hardware registers accessed through ESP32 SDK functions
- Memory management using standard C allocation functions

## Sensor and Task Priority Classification

### Sensor Priority Levels

**High Priority Sensors (Critical Real-time Data):**
- **ADC Sensors**: Flow rate measurements, pressure (PSI) sensors, amperage/current monitoring
- **PWM Sensors**: Pulse-width modulated sensors requiring precise timing
- **Characteristics**:
  - Must be sampled at consistent intervals (100ms frames)
  - Data loss is unacceptable
  - Require immediate processing and validation
  - Feed critical control systems and safety monitoring

**Medium Priority Sensors (Environmental Monitoring):**
- **Temperature Sensors**: DS18B20 one-wire temperature sensors
- **Environmental Sensors**: BME280 (temperature, humidity, barometric pressure)
- **Characteristics**:
  - Sampled in dedicated time frames (800-900ms slots)
  - Allow some data latency
  - Used for trending and environmental control

**Low Priority Sensors (Status Monitoring):**
- **GPIO Expanders**: MCP23X17 digital I/O status
- **RTC/Time Sensors**: RV1805 real-time clock synchronization
- **Characteristics**:
  - Sampled opportunistically
  - Used for system status and logging

### Task Priority Classification

**Critical Priority Tasks (Priority 5 - Real-time):**
- **Sensor Data Acquisition**: Must capture all sensor data without loss
- **Watchdog Monitoring**: Hardware protection against system hangs
- **ISR Handlers**: Flow sensor pulse counting (cannot be interrupted)

**Fixed Frequency Tasks (Priority 3 - Deterministic):**
- **MQTT Publishing**: Must occur at regular 1000ms intervals for data consistency
- **Data Processing**: Transform raw sensor data into final values
- **Flow Data Updates**: Process accumulated pulse counts at 2000ms intervals

**Background Tasks (Priority 1 - Best Effort):**
- **SD Card Logging**: Write buffered data when system load permits
- **System Monitoring**: Periodic health checks and status reporting
- **NTP Synchronization**: Time synchronization (hourly intervals)

**Idle Tasks (Priority 0 - Opportunistic):**
- **WiFi Connection Management**: Maintain connectivity when possible
- **OTA Updates**: Firmware updates during low-activity periods

### Data Handling Strategy

**Critical Data Capture (RAM Buffer with Acceptable Loss):**
- **Primary Buffer**: RAM-based circular buffer for immediate sensor data storage
- **Buffer Size**: Configurable for 5-15 seconds of sensor data (based on ESP32 RAM availability)
- **Data Loss Tolerance**: Acceptable to lose up to 15 seconds of data on system reset/crash
- **Buffer Management**: Automatic overflow handling with oldest data replacement
- **Validation**: CRC checks and timestamp verification for data integrity

**Background Data Persistence:**
- **SD Card Writing**: Low-priority task writes buffered data to SD card at regular intervals
- **Buffer Flushing**: Frequent flush intervals (2-5 seconds) to minimize RAM usage
- **Error Recovery**: Automatic retry logic for SD card write failures with backoff
- **Data Compression**: Optional compression for extended logging periods
- **Memory Efficiency**: Design prioritizes minimal RAM footprint over absolute data preservation

**Task Synchronization Requirements:**
- **Sensor Reading Frame Lock**: Prevents concurrent I2C access during critical reads
- **Data Publishing Lock**: Ensures complete data sets are published together
- **Buffer Access Mutex**: Protects shared RAM circular buffer operations
- **SD Card Access Semaphore**: Serializes SD card operations and prevents concurrent file writes

## Design Goal 1: FreeRTOS Task Architecture Migration

### Current ESP8266 Limitations
- Single-threaded Arduino loop() execution
- Blocking operations in main loop
- Limited real-time performance
- Difficulty managing concurrent operations

### Target ESP32 FreeRTOS Architecture

#### Task Definitions and Priorities

**Critical Priority Tasks (Priority 5 - Real-time):**
- **Sensor Acquisition Task** - 100ms execution cycle
  - Reads high-priority ADC/PWM sensors (flow, PSI, amperage) in framed intervals
  - Manages sensor timing to prevent I2C bus conflicts and data loss
  - Priority: 5 (highest - cannot be interrupted)
  - Stack size: 4096 bytes
  - Execution frequency: Every 100ms (guaranteed)
  - Data handling: Direct RAM buffer with no-loss guarantee

- **Watchdog Monitor Task** - 50ms execution cycle
  - Monitors system health and feeds hardware watchdog
  - Tracks critical task execution times for deadlock detection
  - Priority: 5
  - Stack size: 2048 bytes

**Fixed Frequency Tasks (Priority 3 - Deterministic):**
- **MQTT Publisher Task** - 1000ms execution cycle
  - Publishes binary and JSON data to MQTT servers at fixed intervals
  - Handles connection management and retry logic
  - Priority: 3 (lower than sensor acquisition but guaranteed execution)
  - Stack size: 6144 bytes (for JSON buffer)
  - Data handling: Reads from validated sensor buffers

- **Data Processing Task** - 1000ms execution cycle
  - Processes raw sensor data into final calibrated values
  - Applies filtering and validation to critical sensor data
  - Priority: 3
  - Stack size: 3072 bytes

- **Flow Sensor ISR Handler Task** - Event-driven (interrupt priority)
  - Processes pulse interrupts from flow sensors
  - Updates flow counters with microsecond timestamps
  - Priority: ISR priority (above FreeRTOS tasks)
  - Stack size: 2048 bytes
  - Data handling: Accumulates in protected counters

**Background Tasks (Priority 1 - Best Effort):**
- **SD Card Logger Task** - 2000-5000ms execution cycle
  - Writes buffered sensor data to SD card at regular intervals to minimize RAM usage
  - Manages log file rotation and error recovery with exponential backoff
  - Priority: 1 (runs when higher priority tasks are idle)
  - Stack size: 4096 bytes
  - Data handling: Flushes RAM circular buffer to SD card, maintains minimal memory footprint

- **System Monitor Task** - 30000ms execution cycle
  - Reports system status and performance metrics
  - Monitors memory usage and task health
  - Priority: 1
  - Stack size: 2048 bytes

- **NTP Sync Task** - 3600000ms execution cycle
  - Synchronizes RTC with NTP servers
  - Priority: 1
  - Stack size: 2048 bytes

**Idle Tasks (Priority 0 - Opportunistic):**
- **WiFi Connection Manager** - Event-driven
  - Maintains WiFi connectivity during low system load
  - Handles reconnection and network optimization
  - Priority: 0 (runs only when all other tasks are blocked)
  - Stack size: 3072 bytes

- **OTA Update Handler** - Event-driven
  - Manages over-the-air firmware updates during idle periods
  - Priority: 0
  - Stack size: 2048 bytes

#### Inter-Task Communication

**Semaphores:**
- `sensorDataMutex` - Protects shared sensor data structures
- `i2cBusMutex` - Serializes I2C bus access
- `logBufferMutex` - Protects logging buffer operations
- `mqttPublishSemaphore` - Signals when new data is ready for MQTT

**Queues:**
- `sensorDataQueue` - Passes processed sensor data (size: 10 items)
- `flowPulseQueue` - Passes flow sensor pulse events (size: 50 items)
- `logDataQueue` - Passes buffered sensor data to SD logger (size: 100 items, RAM-efficient)
- `commandQueue` - Passes serial commands to appropriate handlers (size: 5 items)

**Event Groups:**
- `systemStatusBits` - Tracks system state (WiFi connected, MQTT connected, SD ready, etc.)
- `sensorStatusBits` - Tracks individual sensor health status

#### Task Synchronization Strategy

**Sensor Reading Frame System (Maintained from ESP8266):**
- Distributed I2C reads across 100ms frames to prevent bus conflicts
- Frame 0: Temperature sensors (100-200ms)
- Frame 1: Skip (timing spacing)
- Frame 2: Basic I/O (analog/digital) (200-300ms)
- Frame 3: Skip (timing spacing)
- Frame 4: ADC sensors (400-500ms)
- Frame 5: Skip (timing spacing)
- Frame 6: GPIO expander (600-700ms)
- Frame 7: Skip (timing spacing)
- Frame 8: BME280 sensor (800-900ms)
- Frame 9: Skip (timing spacing)

**Execution Timeline:**
```
Time: 0ms    100ms   200ms   300ms   400ms   500ms   600ms   700ms   800ms   900ms   1000ms
Task: |Sensor |Temp   |I/O    |ADC    |GPIO   |BME280 |Process|MQTT   |Log    |Monitor|
Freq: |100ms  |100ms  |100ms  |100ms  |100ms  |100ms  |1000ms |1000ms |5000ms |30000ms|
```

## Design Goal 2: Modular Code Architecture

### Current ESP8266 Structure
- Single monolithic `GenericSensorX.ino` file (1669 lines)
- All functionality in one Arduino sketch
- Mixed concerns (setup, loop, sensor reading, MQTT, logging)
- Difficult to maintain and test individual components

### Target ESP32 Modular Structure

#### File Organization
```
GenericSensorX/
├── GenericSensorX.ino          # Main entry point and task orchestration
├── include/
│   ├── config.h                 # System configuration constants
│   ├── types.h                  # Data structures and enums
│   ├── globals.h                # Global variables and externs
│   └── pins.h                   # Pin definitions for Arduino Nano ESP32
├── tasks/
│   ├── sensor_acquisition.h     # Sensor reading task
│   ├── sensor_acquisition.c
│   ├── data_processor.h         # Data processing task
│   ├── data_processor.c
│   ├── mqtt_publisher.h         # MQTT communication task
│   ├── mqtt_publisher.c
│   ├── sd_logger.h              # SD card logging task
│   ├── sd_logger.c
│   ├── system_monitor.h         # System monitoring task
│   ├── system_monitor.c
│   ├── wifi_manager.h           # WiFi connection management
│   ├── wifi_manager.c
│   ├── flow_sensor.h            # Flow sensor ISR handling
│   ├── flow_sensor.c
│   └── watchdog.h               # Watchdog monitoring
├── drivers/
│   ├── i2c_manager.h            # I2C bus management
│   ├── i2c_manager.c
│   ├── ads1x15_driver.h         # ADS1015/ADS1115 driver
│   ├── ads1x15_driver.c
│   ├── bme280_driver.h          # BME280 environmental sensor
│   ├── bme280_driver.c
│   ├── ds18b20_driver.h         # DS18B20 temperature sensors
│   ├── ds18b20_driver.c
│   ├── mcp23x17_driver.h        # GPIO expander driver
│   ├── mcp23x17_driver.c
│   ├── rtc_driver.h             # RV1805 RTC driver
│   ├── rtc_driver.c
│   ├── openlog_driver.h         # OpenLog SD card driver
│   ├── openlog_driver.c
│   └── buzzer_driver.h          # Qwiic buzzer driver
├── utils/
│   ├── debug_utils.h            # Enhanced debugging utilities
│   ├── debug_utils.c
│   ├── time_utils.h             # Time and NTP utilities
│   ├── time_utils.c
│   ├── buffer_utils.h           # Data buffering utilities
│   └── buffer_utils.c
└── platform/
    ├── esp32_platform.h         # ESP32-specific platform code
    └── esp32_platform.c
```

#### Include Strategy
```c
// GenericSensorX.ino - Arduino C language with FreeRTOS
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <cJSON.h>

// Platform-specific includes (ESP32 Arduino)
#include "platform/esp32_platform.h"

// Core system includes
#include "include/config.h"
#include "include/types.h"
#include "include/globals.h"
#include "include/pins.h"

// FreeRTOS task includes
#include "tasks/sensor_acquisition.h"
#include "tasks/data_processor.h"
#include "tasks/mqtt_publisher.h"
#include "tasks/sd_logger.h"
#include "tasks/system_monitor.h"
#include "tasks/wifi_manager.h"
#include "tasks/flow_sensor.h"
#include "tasks/watchdog.h"

// Hardware driver includes
#include "drivers/i2c_manager.h"
#include "drivers/ads1x15_driver.h"
#include "drivers/bme280_driver.h"
#include "drivers/ds18b20_driver.h"
#include "drivers/mcp23x17_driver.h"
#include "drivers/rtc_driver.h"
#include "drivers/openlog_driver.h"
#include "drivers/buzzer_driver.h"

// Utility function includes
#include "utils/debug_utils.h"
#include "utils/time_utils.h"
#include "utils/buffer_utils.h"
```

#### Module Responsibilities

**Main Entry Point (`GenericSensorX.ino`):**
- Initialize Arduino framework and FreeRTOS
- Create and start FreeRTOS tasks with proper priorities
- Setup inter-task communication primitives (queues, semaphores, mutexes)
- Initialize ESP32-specific hardware (I2C, interrupts, timers)
- Handle system-level events and error recovery

**Task Modules (.c/.h pairs):**
- Each task implements a FreeRTOS task function with clear entry point
- Tasks communicate via queues, semaphores, and shared memory with mutex protection
- Error handling and recovery implemented within each task function
- Configurable execution parameters through header-defined constants
- C-style function interfaces with proper parameter passing

**Driver Modules (.c/.h pairs):**
- Hardware abstraction layer using C function interfaces
- Platform-independent APIs through function pointers and callbacks
- Error handling through return codes and status structures
- Optimized for ESP32 capabilities while maintaining C language constructs
- Static functions for internal implementation details

**Utility Modules (.c/.h pairs):**
- Common C functions used across multiple tasks
- Debugging and logging utilities with printf-style interfaces
- Time management functions using ESP32 hardware timers
- Data processing helpers with C-style data structures
- Buffer management with proper memory allocation/deallocation

## Migration Implementation Phases

### Phase 1: Core Framework Setup
- [ ] Create modular file structure
- [ ] Setup ESP32 platform code
- [ ] Implement basic FreeRTOS task skeleton
- [ ] Port pin definitions for Arduino Nano ESP32

### Phase 2: Driver Migration
- [ ] Migrate I2C bus management
- [ ] Port individual sensor drivers
- [ ] Test hardware interfaces
- [ ] Validate sensor readings

### Phase 3: Task Implementation
- [ ] Implement sensor acquisition task
- [ ] Add data processing task
- [ ] Setup MQTT publisher task
- [ ] Create SD logging task

### Phase 4: System Integration
- [ ] Integrate all tasks with proper synchronization
- [ ] Implement error handling and recovery
- [ ] Add system monitoring and diagnostics
- [ ] Performance optimization

### Phase 5: Advanced Features
- [ ] Implement OTA update capability
- [ ] Add low-power sleep modes
- [ ] Enhance debugging and monitoring
- [ ] Final testing and validation

## Benefits of New Architecture

1. **Improved Maintainability**: Modular code structure makes updates and debugging easier
2. **Better Real-time Performance**: FreeRTOS allows precise task scheduling and priorities
3. **Enhanced Reliability**: Isolated tasks prevent single-point failures
4. **Scalability**: Easy to add new sensors or features as separate tasks
5. **Resource Efficiency**: Better memory management and CPU utilization
6. **Future-Proof**: ESP32 platform provides room for expansion and advanced features

## Risk Mitigation

1. **Incremental Migration**: Test each module individually before integration
2. **Backward Compatibility**: Maintain existing data formats and protocols
3. **Comprehensive Testing**: Validate all functionality matches ESP8266 behavior
4. **Performance Monitoring**: Track execution times and resource usage
5. **Fallback Mechanisms**: Implement safe failure modes and recovery procedures

---

*Document Version: 1.0*
*Last Updated: October 28, 2025*
*Author: ESP32 Migration Team*
