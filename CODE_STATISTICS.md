# Code Statistics Breakdown

## Overall Summary

**Total Lines of Code: 7,857 lines**

- **Source Files (.c/.cpp)**: 5,724 lines (73%)
- **Header Files (.h/.hpp)**: 2,133 lines (27%)

---

## Breakdown by Component

### Main Application
- **main/main.c**: 356 lines
- **Total**: 356 lines (4.5%)

### Sensor System Component
- **Source Files**: 5,368 lines
- **Header Files**: 2,133 lines
- **Total**: 7,501 lines (95.5%)

---

## Source Files Breakdown (5,724 lines)

### System & Infrastructure (1,728 lines - 30%)
| File | Lines | Description |
|------|-------|-------------|
| `system_init.c` | 834 | System monitor, fan control, MQTT system status |
| `error_recovery.c` | 512 | Error recovery strategies and monitoring |
| `watchdog.c` | 381 | Watchdog timer management |

### I2C Sensor Managers (1,356 lines - 24%)
| File | Lines | Description |
|------|-------|-------------|
| `i2c_env_manager.c` | 597 | BME280 environmental sensor (temp, humidity, pressure) |
| `i2c_adc_manager.cpp` | 417 | ADS1015/ADS1115 ADC sensors (C++ wrapper) |
| `i2c_gpio_manager.c` | 342 | MCP23017 GPIO expander |

### Communication & Networking (1,131 lines - 20%)
| File | Lines | Description |
|------|-------|-------------|
| `wifi_manager.c` | 415 | WiFi connection management |
| `mqtt_manager.c` | 324 | MQTT client connection and publishing |
| `mqtt_publisher.c` | 232 | MQTT message publishing task |
| `sensor_coordination.c` | 135 | Sensor task coordination (event groups, mutex) |
| `sensor_acquisition.c` | 161 | Sensor data acquisition coordination |

### Sensor Drivers (735 lines - 13%)
| File | Lines | Description |
|------|-------|-------------|
| `onewire_temp_manager.c` | 376 | DS18B20 One-Wire temperature sensors |
| `pcnt_flow_manager.c` | 359 | PCNT-based flow sensor pulse counting |

### Core Infrastructure (774 lines - 13%)
| File | Lines | Description |
|------|-------|-------------|
| `i2c_manager.c` | 241 | I2C bus manager (master driver) |
| `sensor.c` | 42 | Sensor data structure definitions |
| `main.c` | 356 | Application entry point, initialization |
| `sensor_coordination.c` | 135 | (already counted above) |

---

## Header Files Breakdown (2,133 lines)

### Data Structures & Configuration (623 lines - 29%)
| File | Lines | Description |
|------|-------|-------------|
| `sensor.h` | 246 | GenericSens data structure, MQTT topics |
| `pins.h` | 231 | GPIO pin definitions and assignments |
| `config.h` | 147 | System configuration constants |

### System Infrastructure Headers (380 lines - 18%)
| File | Lines | Description |
|------|-------|-------------|
| `error_recovery.h` | 212 | Error recovery API |
| `sensor_coordination.h` | 180 | Sensor coordination API (event groups, mutex) |

### Watchdog & Types (317 lines - 15%)
| File | Lines | Description |
|------|-------|-------------|
| `watchdog.h` | 168 | Watchdog timer API |
| `types.h` | 149 | Common type definitions |

### Communication Headers (274 lines - 13%)
| File | Lines | Description |
|------|-------|-------------|
| `wifi_manager.h` | 140 | WiFi manager API |
| `mqtt_manager.h` | 134 | MQTT manager API |

### Sensor Manager Headers (539 lines - 25%)
| File | Lines | Description |
|------|-------|-------------|
| `i2c_manager.h` | 101 | I2C bus manager API |
| `pcnt_flow_manager.h` | 81 | PCNT flow sensor API |
| `i2c_adc_manager.h` | 80 | I2C ADC manager API |
| `i2c_gpio_manager.h` | 77 | I2C GPIO expander API |
| `onewire_temp_manager.h` | 61 | One-Wire temperature sensor API |
| `i2c_env_manager.h` | 57 | I2C environmental sensor API |
| `system_init.h` | 29 | System monitor API |
| `sensor_acquisition.h` | 20 | Sensor acquisition API |
| `mqtt_publisher.h` | 20 | MQTT publisher API |

---

## Code Distribution by Functionality

### Sensor Management (2,091 lines - 27%)
- I2C sensor managers (BME280, ADC, GPIO expander)
- One-Wire temperature sensors
- PCNT flow sensors
- Sensor coordination

### System Infrastructure (1,893 lines - 24%)
- System monitor and fan control
- Error recovery system
- Watchdog timer
- Sensor data structures

### Communication (1,405 lines - 18%)
- WiFi manager
- MQTT client and publisher
- Sensor coordination

### Core Infrastructure (1,468 lines - 19%)
- I2C bus manager
- Main application initialization
- Configuration and pin definitions

### Data Structures & Headers (1,000 lines - 12%)
- Header files and API definitions
- Type definitions

---

## Largest Files (Top 10)

1. **system_init.c** - 834 lines (System monitor, fan control)
2. **i2c_env_manager.c** - 597 lines (BME280 environmental sensor)
3. **error_recovery.c** - 512 lines (Error recovery system)
4. **i2c_adc_manager.cpp** - 417 lines (ADS1015/ADS1115 ADC)
5. **wifi_manager.c** - 415 lines (WiFi connection management)
6. **watchdog.c** - 381 lines (Watchdog timer)
7. **onewire_temp_manager.c** - 376 lines (DS18B20 sensors)
8. **pcnt_flow_manager.c** - 359 lines (Flow sensors)
9. **main.c** - 356 lines (Application entry point)
10. **i2c_gpio_manager.c** - 342 lines (MCP23017 GPIO expander)

---

## Code Language Distribution

- **C Files**: ~5,307 lines (68%)
- **C++ Files**: ~417 lines (5%)
- **Header Files**: ~2,133 lines (27%)

---

## Module Complexity (Lines per Module)

### High Complexity (>500 lines)
- System Monitor (`system_init.c`) - 834 lines
- BME280 Manager (`i2c_env_manager.c`) - 597 lines
- Error Recovery (`error_recovery.c`) - 512 lines

### Medium Complexity (300-500 lines)
- ADC Manager (`i2c_adc_manager.cpp`) - 417 lines
- WiFi Manager (`wifi_manager.c`) - 415 lines
- Watchdog (`watchdog.c`) - 381 lines
- One-Wire Manager (`onewire_temp_manager.c`) - 376 lines
- PCNT Flow Manager (`pcnt_flow_manager.c`) - 359 lines
- Main (`main.c`) - 356 lines
- GPIO Manager (`i2c_gpio_manager.c`) - 342 lines

### Low Complexity (<300 lines)
- MQTT Manager (`mqtt_manager.c`) - 324 lines
- I2C Manager (`i2c_manager.c`) - 241 lines
- MQTT Publisher (`mqtt_publisher.c`) - 232 lines
- Sensor Acquisition (`sensor_acquisition.c`) - 161 lines
- Sensor Coordination (`sensor_coordination.c`) - 135 lines
- Sensor Data (`sensor.c`) - 42 lines

---

## Summary Statistics

- **Total Source Files**: 15 files
- **Total Header Files**: 18 files
- **Average Lines per Source File**: 382 lines
- **Average Lines per Header File**: 119 lines
- **Largest Module**: System Monitor (834 lines)
- **Smallest Module**: Sensor Data (42 lines)

---

## Code Organization

The codebase follows a modular architecture:

1. **Main Application** (`main/`) - Entry point and initialization
2. **Sensor System Component** (`components/sensor_system/`)
   - **src/** - Implementation files
   - **include/** - Header files and APIs
3. **Clear separation** between:
   - Sensor managers (I2C, One-Wire, PCNT)
   - Communication (WiFi, MQTT)
   - System infrastructure (Watchdog, Error Recovery)
   - Core utilities (I2C bus, Sensor coordination)

---

*Generated: 2025-11-12*
*Excludes: Build artifacts, CMake generated files, managed components*

