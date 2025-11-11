# Documentation Updates Required

This document outlines the changes needed to remove references to deferred features (NTP/RTC, Logging, Buzzer) from existing documentation and move them to FUTURE_UPGRADES.md.

## Files to Update

### 1. `notes/ESP32_Migration_Design.md`

#### Change 1: Remove OpenLog driver references (lines 270-271)
**Current:**
```
│   ├── openlog_driver.h         # OpenLog SD card driver
│   ├── openlog_driver.c
```

**Change to:**
```
│   ├── (OpenLog driver removed - see FUTURE_UPGRADES.md)
```

#### Change 2: Remove Buzzer driver references (line 272)
**Current:**
```
│   └── buzzer_driver.h          # Qwiic buzzer driver
```

**Change to:**
```
│   └── (Buzzer driver removed - see FUTURE_UPGRADES.md)
```

#### Change 3: Remove OpenLog include (line 320)
**Current:**
```
#include "drivers/openlog_driver.h"
```

**Change to:**
```
// OpenLog driver removed - see FUTURE_UPGRADES.md
```

#### Change 4: Remove Buzzer include (line 321)
**Current:**
```
#include "drivers/buzzer_driver.h"
```

**Change to:**
```
// Buzzer driver removed - see FUTURE_UPGRADES.md
```

#### Change 5: Update SD Card Logger Task description (line 152)
**Current:**
```
- **SD Card Logger Task** - 2000-5000ms execution cycle
  - Writes buffered sensor data to SD card at regular intervals to minimize RAM usage
  - Manages log file rotation and error recovery with exponential backoff
  - Priority: 1 (runs when higher priority tasks are idle)
  - Stack size: 4096 bytes
  - Data handling: Flushes RAM circular buffer to SD card, maintains minimal memory footprint
```

**Change to:**
```
- **SD Card Logger Task** - DEFERRED (see FUTURE_UPGRADES.md)
  - Logging system implementation deferred to future upgrade
  - Options include: HTTP POST to server, MQTT logging, or W25Q128 SPI flash
  - Priority: 1 (runs when higher priority tasks are idle)
  - Stack size: 4096 bytes (when implemented)
```

---

### 2. `notes/Design Notes/SYSTEM_MONITOR_OUTPUT_EXAMPLES.md`

#### Change 1: Remove OpenLog and Buzzer from I2C device list (lines 112-119)
**Current:**
```
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 5 [0x20, 0x2A, 0x34, 0x48, 0x77]
```

**Change to:**
```
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 3 [0x20, 0x48, 0x77]
```

**Current:**
```
**Device Addresses:**
- `0x20` - GPIO Expander (MCP23X17)
- `0x2A` - OpenLog datalogger
- `0x34` - Qwiic Buzzer
- `0x48` - ADS1115 (16-bit ADC)
- `0x77` - BME280 Environmental Sensor
```

**Change to:**
```
**Device Addresses:**
- `0x20` - GPIO Expander (MCP23X17)
- `0x48` - ADS1015/ADS1115 (12/16-bit ADC)
- `0x77` - BME280 Environmental Sensor

**Note**: OpenLog (0x2A) and Buzzer (0x34) are deferred - see FUTURE_UPGRADES.md
```

---

### 3. `notes/ESP32_Implementation_Plan.md`

#### Change 1: Remove OpenLog driver section (around line 750)
**Current:**
```
**8.1.1 drivers/openlog_driver.h**
- OpenLog SD card interface
- Include guards: `#ifndef OPENLOG_DRIVER_H` / `#define OPENLOG_DRIVER_H` / `#endif`

**8.1.2 drivers/openlog_driver.c**
```

**Change to:**
```
**8.1.1 Logging System - DEFERRED**
- Logging system implementation deferred to future upgrade
- See FUTURE_UPGRADES.md for implementation options:
  - HTTP POST to local server (recommended)
  - MQTT-based logging
  - W25Q128 SPI flash storage
```

#### Change 2: Update Phase 9 description (if exists)
**Current:**
```
Phase 9: System Utilities
- RTC driver (RV1805)
- Time utilities
- OpenLog driver
```

**Change to:**
```
Phase 9: System Utilities
- Time utilities (built-in RTC with SNTP - see FUTURE_UPGRADES.md)
- Logging system (deferred - see FUTURE_UPGRADES.md)
- Buzzer manager (deferred - see FUTURE_UPGRADES.md)
```

---

### 4. `notes/Design Notes/SENSOR_MANAGER_ARCHITECTURE.md`

#### Change 1: Add note about deferred features (if mentioned)
**Add at end of document:**
```
## Deferred Features

The following features are documented in FUTURE_UPGRADES.md:
- **Time Management**: Built-in RTC with SNTP (replaces external I2C RTC)
- **Data Logging**: HTTP POST, MQTT logging, or W25Q128 SPI flash options
- **I2C Buzzer Manager**: Audio feedback and alerts
```

---

### 5. `notes/Design Notes/GenericSensor.ino`

#### Change 1: Add note at top of file (after revision log)
**Add after line 50:**
```
 * NOTE: ESP-IDF implementation defers the following features to FUTURE_UPGRADES.md:
 *   - OpenLog SD card logging (replaced with HTTP POST or MQTT logging options)
 *   - I2C RTC (RV1805) - replaced with built-in RTC + SNTP
 *   - Qwiic Buzzer - deferred to future upgrade
```

---

## Summary of Changes

### Files Modified:
1. `notes/ESP32_Migration_Design.md` - Remove OpenLog and Buzzer driver references
2. `notes/Design Notes/SYSTEM_MONITOR_OUTPUT_EXAMPLES.md` - Remove from I2C device list
3. `notes/ESP32_Implementation_Plan.md` - Remove OpenLog driver section, update Phase 9
4. `notes/Design Notes/SENSOR_MANAGER_ARCHITECTURE.md` - Add deferred features note
5. `notes/Design Notes/GenericSensor.ino` - Add note about deferred features

### Key Changes:
- **OpenLog (0x2A)**: Removed from all documentation, moved to FUTURE_UPGRADES.md
- **Buzzer (0x34)**: Removed from all documentation, moved to FUTURE_UPGRADES.md
- **RTC (RV1805, 0x69)**: Already documented as replaced with built-in RTC + SNTP
- **Logging System**: References updated to point to FUTURE_UPGRADES.md

### New File Created:
- `notes/FUTURE_UPGRADES.md` - Comprehensive documentation of deferred features

---

## Implementation Notes

1. **No Code Changes Required**: These are documentation-only updates
2. **Backward Compatibility**: Existing code references remain in GenericSensor.ino (Arduino reference)
3. **Future Reference**: All deferred features are fully documented in FUTURE_UPGRADES.md
4. **Clear Migration Path**: FUTURE_UPGRADES.md provides implementation details for when these features are added

