# Codebase Documentation

**Firmware Version:** 2.1  
**Documentation Date:** November 21, 2025  
**Project:** SensorX ESP32-C6 Feather  
**Framework:** ESP-IDF v5.5.1  

## Overview

This document provides a comprehensive overview of the SensorX codebase, including file sizes, complexity metrics, and architectural information. Use this document to track changes across firmware versions.

---

## Version Information

- **Firmware Version:** 2.1
- **Firmware Name:** SensorX ESP32
- **Target Hardware:** ESP32-C6 Feather
- **ESP-IDF Version:** v5.5.1-dirty
- **FreeRTOS Version:** V10.5.1

---

## Source Files (Components)

### Core System Files

| File | Lines | Functions | Complexity | Description |
|------|-------|-----------|------------|-------------|
| `system_init.c` | 1009 | 5 | High | System initialization, monitoring, fan control, MQTT system monitor |
| `error_recovery.c` | 512 | 17 | Medium-High | Error recovery system with multiple strategies |
| `watchdog.c` | 381 | 14 | Medium | Watchdog timer management and task health monitoring |
| `panic_stats.c` | 266 | 9 | Low-Medium | Panic statistics tracking with NVS persistence |

### Sensor Management Files

| File | Lines | Functions | Description |
|------|-------|-----------|-------------|
| `sensor_coordination.c` | 140 | 7 | Shared sensor data structure management (genericSens_) |
| `sensor_acquisition.c` | 161 | 20 | Sensor data acquisition task coordination |
| `sensor.c` | 42 | 0 | JSON variable name definitions |

### Communication Files

| File | Lines | Functions | Description |
|------|-------|-----------|-------------|
| `wifi_manager.c` | 415 | 18 | WiFi connection management and status tracking |
| `mqtt_manager.c` | 324 | 14 | MQTT broker connection and message publishing |
| `mqtt_publisher.c` | 232 | 1 | MQTT message publishing task |

### Sensor Driver Files

| File | Lines | Functions | Description |
|------|-------|-----------|-------------|
| `onewire_temp_manager.c` | 381 | 6 | DS18B20 One-Wire temperature sensors (supports 5 sensors) |
| `pcnt_flow_manager.c` | 359 | 4 | PCNT-based flow sensor pulse counting (3 sensors) |
| `i2c_adc_manager.cpp` | 417 | 6 | I2C ADC sensors (ADS1015/ADS1115) |
| `i2c_gpio_manager.c` | 342 | 8 | I2C GPIO expander (MCP23017) |
| `i2c_manager.c` | 241 | 8 | I2C bus management and device scanning |
| `i2c_env_manager.c` | 597 | 13 | I2C environmental sensors (BME280 - REMOVED) |

### Monitoring Files

| File | Lines | Functions | Description |
|------|-------|-----------|-------------|
| `task_monitor.c` | 293 | 7 | FreeRTOS task statistics monitoring (periodic, 2 min interval) |

---

## Header Files (Components)

| File | Lines | Description |
|------|-------|-------------|
| `sensor.h` | 246 | Shared sensor data structure (genericSens_) - FROZEN interface |
| `pins.h` | 231 | GPIO pin definitions for ESP32-C6 Feather |
| `error_recovery.h` | 212 | Error recovery system API |
| `sensor_coordination.h` | 180 | Sensor coordination API |
| `watchdog.h` | 168 | Watchdog timer API |
| `config.h` | 161 | System-wide configuration constants |
| `types.h` | 149 | Common data structures and type definitions |
| `wifi_manager.h` | 140 | WiFi manager API |
| `mqtt_manager.h` | 134 | MQTT manager API |
| `i2c_manager.h` | 101 | I2C manager API |
| `pcnt_flow_manager.h` | 81 | PCNT flow manager API |
| `i2c_adc_manager.h` | 80 | I2C ADC manager API |
| `i2c_gpio_manager.h` | 77 | I2C GPIO manager API |
| `panic_stats.h` | 64 | Panic statistics API |
| `onewire_temp_manager.h` | 61 | One-Wire temperature manager API |
| `i2c_env_manager.h` | 57 | I2C environmental manager API (BME280 - REMOVED) |
| `task_monitor.h` | 54 | Task monitor API |
| `system_init.h` | 29 | System initialization API |
| `sensor_acquisition.h` | 20 | Sensor acquisition API |
| `mqtt_publisher.h` | 20 | MQTT publisher API |

---

## Main Application Files

| File | Lines | Functions | Description |
|------|-------|-----------|-------------|
| `main/main.c` | 358 | 1 | Application entry point, system initialization sequence |

---

## Total Statistics

### Source Code
- **Total Source Files:** 17 (.c/.cpp files)
- **Total Source Lines:** ~6,112 lines
- **Total Functions:** ~150+ functions
- **Average File Size:** ~360 lines
- **Largest File:** `system_init.c` (1,009 lines)
- **Smallest File:** `sensor.c` (42 lines)

### Header Files
- **Total Header Files:** 20 (.h files)
- **Total Header Lines:** ~2,265 lines
- **Average Header Size:** ~113 lines
- **Largest Header:** `sensor.h` (246 lines)
- **Smallest Header:** `mqtt_publisher.h` (20 lines)

### Main Application
- **Main Files:** 1
- **Main Lines:** 358 lines

---

## Complexity Assessment

### High Complexity Files (>500 lines)
- `system_init.c` (1,009 lines) - System monitoring, fan control, MQTT publishing
- `i2c_env_manager.c` (597 lines) - BME280 removed, but file retained
- `error_recovery.c` (512 lines) - Multiple error recovery strategies

### Medium Complexity Files (200-500 lines)
- `i2c_adc_manager.cpp` (417 lines)
- `wifi_manager.c` (415 lines)
- `onewire_temp_manager.c` (381 lines)
- `watchdog.c` (381 lines)
- `pcnt_flow_manager.c` (359 lines)
- `i2c_gpio_manager.c` (342 lines)
- `mqtt_manager.c` (324 lines)
- `task_monitor.c` (293 lines)
- `panic_stats.c` (266 lines)
- `i2c_manager.c` (241 lines)
- `mqtt_publisher.c` (232 lines)

### Low Complexity Files (<200 lines)
- `sensor_acquisition.c` (161 lines)
- `sensor_coordination.c` (140 lines)
- `sensor.c` (42 lines)

---

## Architecture Overview

### Component Structure
```
components/sensor_system/
├── include/          # Header files (20 files)
├── src/              # Source files (17 files)
└── CMakeLists.txt    # Build configuration
```

### Key Components

1. **System Core**
   - `system_init.c` - Main system monitoring and control
   - `error_recovery.c` - Error handling and recovery
   - `watchdog.c` - Watchdog timer management
   - `panic_stats.c` - Panic tracking and statistics

2. **Sensor Management**
   - `sensor_coordination.c` - Shared data structure (genericSens_)
   - `sensor_acquisition.c` - Sensor reading coordination
   - Individual sensor managers (One-Wire, PCNT, I2C)

3. **Communication**
   - `wifi_manager.c` - WiFi connectivity
   - `mqtt_manager.c` - MQTT broker connection
   - `mqtt_publisher.c` - Data publishing

4. **Monitoring**
   - `task_monitor.c` - FreeRTOS task statistics

---

## Key Features (Version 2.1)

### System Monitoring
- System monitor messages every 10 seconds
- Firmware version and build date/time display
- Idle time percentage calculation
- Panic count tracking (total and since clean boot)
- Reset reason reporting
- Production gates for verbose output

### Sensor Support
- 5 One-Wire DS18B20 temperature sensors (including internal temp)
- 3 PCNT flow sensors
- I2C ADC sensors (ADS1015/ADS1115)
- I2C GPIO expander (MCP23017)
- BME280 removed (replaced with One-Wire internal temp)

### Communication
- WiFi with auto-reconnect
- MQTT binary and JSON publishing
- System monitor MQTT messages
- Task monitor MQTT messages (optional)

### Reliability
- Watchdog timer (30 second timeout)
- Error recovery system (7 strategies)
- Panic statistics tracking with NVS persistence
- Production mode gates for logging

---

## Configuration Constants

### Key Configuration Files
- `config.h` - All system-wide constants
  - Task priorities and stack sizes
  - Sensor intervals
  - MQTT topics
  - Production mode flags

### Production Mode
- `PRODUCTION_MODE` - 0=Development, 1=Production
- `ENABLE_SYSTEM_MONITOR_OUTPUT` - Gate for system monitor serial output
- `ENABLE_TASK_MONITOR_OUTPUT` - Gate for task monitor serial output

---

## Build System

### CMakeLists.txt
- Component registration with all source files
- Dependencies: esp_hw_support, driver, esp_netif, esp_event, esp_wifi, nvs_flash, mqtt, json, onewire_bus, espp__ads1x15

### Component Structure
- Modular design with clear separation of concerns
- Each sensor type has its own manager
- Shared data structure (genericSens_) for all sensors

---

## Memory Usage

### Stack Sizes (from config.h)
- Critical tasks: 4096-6144 bytes
- Fixed frequency tasks: 3072-4096 bytes
- Background tasks: 2048-3072 bytes
- Task Monitor: 6144 bytes (increased for watchdog safety)

### Shared Data Structure
- `genericSens_` structure: 104 bytes (26 words × 4 bytes)
- Protected by mutex
- Used for both binary and JSON MQTT publishing

---

## Version History Tracking

### Version 2.1 (Current)
- Added panic statistics tracking
- Added firmware version and build info to monitor
- Added idle time percentage to monitor
- Changed monitor interval from 5s to 10s
- Fixed TaskMonitor watchdog panics
- Added production gates for monitor output
- BME280 removed, using One-Wire internal temp

### Previous Versions
- Version 2.0: Initial production release
- Version 1.x: Development versions

---

## Notes for Future Development

1. **File Size Monitoring**: `system_init.c` is approaching 1000 lines - consider splitting if it grows further
2. **BME280 Removal**: `i2c_env_manager.c` still exists but is commented out in build - consider removing entirely
3. **Production Gates**: Monitor output can be disabled in production mode while maintaining MQTT publishing
4. **Panic Tracking**: Panic statistics persist across power cycles using NVS
5. **Watchdog Safety**: All long-running operations include watchdog resets

---

## Maintenance Guidelines

### When Adding New Features
1. Update this document with new file information
2. Update version number in `config.h`
3. Update version history section
4. Document complexity changes

### When Modifying Files
1. Update line counts if significant changes
2. Update function counts if adding/removing functions
3. Note complexity changes if file grows significantly

### Version Tracking
- Increment minor version for feature additions
- Increment major version for breaking changes
- Document all changes in version history section

---

**Last Updated:** November 21, 2025  
**Documentation Version:** 1.0

