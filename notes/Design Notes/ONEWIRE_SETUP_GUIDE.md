# One-Wire Temperature Manager - Setup Guide

## Overview

The One-Wire Temperature Manager uses the official Espressif `onewire_bus` component, which provides hardware-based timing via the RMT peripheral. This is much more reliable than bit-banging.

## Setup Steps

### 1. Add Component Dependency

Before building, you must add the One-Wire component to your project:

```bash
cd /Volumes/iHome/Users/dub/esp/SensorX
idf.py add-dependency "espressif/onewire_bus^1.0.4"
```

This will:
- Download the component from ESP Component Registry
- Add it to your `managed_components` directory
- Make it available to your project

### 2. Build the Project

After adding the dependency, build normally:

```bash
idf.py build
```

The component will be automatically linked via the `REQUIRES onewire_bus` in `CMakeLists.txt`.

## Implementation Details

### Files Created

1. **`components/sensor_system/include/onewire_temp_manager.h`**
   - Header file with function declarations
   - Task handle declaration

2. **`components/sensor_system/src/onewire_temp_manager.c`**
   - Complete implementation using official component
   - ~300 lines (vs ~400+ with bit-banging)

### Files Modified

1. **`components/sensor_system/include/config.h`**
   - Added One-Wire timing constants

2. **`components/sensor_system/CMakeLists.txt`**
   - Added `onewire_temp_manager.c` to sources
   - Added `onewire_bus` to REQUIRES

3. **`main/main.c`**
   - Added initialization call after sensor coordination
   - Includes onewire_temp_manager.h

### Features

- **Hardware-Based Timing**: Uses RMT peripheral (no CPU load)
- **Up to 4 Sensors**: Supports 4 DS18B20 sensors on GPIO6
- **5000ms Interval**: Reads all sensors every 5 seconds
- **Cooperative Multitasking**: 20ms yields between sensors
- **Error Handling**: CRC verification, retry logic
- **Watchdog Protection**: 6000ms timeout
- **Thread-Safe**: Mutex-protected updates to `genericSens_`

### Data Updates

- Sensor 0 → `genericSens_.temp1` (int) and `temp1_f` (float)
- Sensor 1 → `genericSens_.temp2` (float)
- Sensor 2 → `genericSens_.temp3` (float)
- Sensor 3 → `genericSens_.temp4` (float)
- `genericSens_.tempSensorcount` → Number of sensors found

### Task Configuration

- **Priority**: `TASK_PRIORITY_BACKGROUND` (1) - Lowest priority
- **Stack**: `TASK_STACK_SIZE_BACKGROUND` (2048 bytes)
- **Interval**: 5000ms (all sensors read per cycle)
- **Watchdog**: 6000ms timeout

## API Usage Example

```c
// Initialize (called from main.c)
esp_err_t ret = onewire_temp_manager_init();

// Get sensor count
uint8_t count = onewire_temp_manager_get_sensor_count();
```

## Troubleshooting

### Component Not Found

If you get "Component onewire_bus not found":
1. Run: `idf.py add-dependency "espressif/onewire_bus^1.0.4"`
2. Verify ESP-IDF version is 5.0+ (you have 5.5.1 ✓)
3. Check `managed_components` directory exists

### No Sensors Found

If no sensors are detected:
1. Check GPIO6 connection
2. Verify 4.7kΩ pull-up resistor
3. Check sensor power (parasitic or VCC)
4. Review serial output for error messages

### Build Errors

If build fails:
1. Ensure `idf.py add-dependency` was run successfully
2. Check `managed_components/espressif__onewire_bus` exists
3. Verify CMakeLists.txt has `onewire_bus` in REQUIRES

## Comparison: Official Component vs Bit-Banging

| Feature | Official Component | Bit-Banging |
|---------|-------------------|-------------|
| Code Size | ~300 lines | ~400+ lines |
| Timing | Hardware (RMT) | Software delays |
| CPU Load | Low | High |
| Interrupts | Never disabled | Disabled briefly |
| Reliability | High | Medium |
| Maintenance | Espressif | You |

## Next Steps

After implementation:
1. Add dependency: `idf.py add-dependency "espressif/onewire_bus^1.0.4"`
2. Build: `idf.py build`
3. Flash and test with DS18B20 sensors
4. Verify sensor readings in verbose output
5. Check MQTT publishes temperature data

