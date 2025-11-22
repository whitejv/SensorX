# BME280 Removal and One-Wire Internal Temperature Plan

## Overview

**Goal:** Remove BME280 environmental sensor and replace with 5th One-Wire sensor (DS18B20) for internal enclosure temperature measurement.

**Date:** 2025-01-XX
**Status:** Planned - Ready for Implementation

---

## Rationale

- **Simplification:** Eliminates complex BME280 calibration code (~598 lines)
- **Cost Reduction:** No BME280 hardware needed
- **Reliability:** One-Wire sensors already proven reliable in field deployment
- **Interface Compatibility:** Maintains `genericSens_` structure (humidity/pressure always zero)

---

## Key Decisions

1. **Remove BME280 completely** - No longer initialize or use I2C environmental manager
2. **Add 5th One-Wire sensor** - Designate sensor index 4 as internal enclosure temperature
3. **Map sensor 4 → `tempx`** - Use existing `tempx` field for internal temperature
4. **Keep interface unchanged** - `humidity` and `pressurex` fields remain but always zero
5. **Update fan control** - Use `tempx` from One-Wire instead of BME280

---

## Sensor Mapping

### One-Wire Sensors (5 total):
- **Sensor 0** → `temp1` (int) and `temp1_f` (float)
- **Sensor 1** → `temp2` (float)
- **Sensor 2** → `temp3` (float)
- **Sensor 3** → `temp4` (float)
- **Sensor 4** → `tempx` (float) - **Internal enclosure temperature** ⭐ NEW

### Environmental Fields (always zero):
- `humidity` → 0.0 (no sensor)
- `pressurex` → 0.0 (no sensor)
- `tempx` → Updated by One-Wire sensor 4 (internal temp)

---

## Files to Modify

### 1. `components/sensor_system/include/config.h`

**Changes:**
- `ONEWIRE_MAX_SENSORS`: 4 → 5
- `MAX_TEMPERATURE_SENSORS`: 4 → 5
- Update `FAN_CONTROL_THRESHOLD_TEMP_F` comment

**Lines to modify:**
- Line 78: `#define ONEWIRE_MAX_SENSORS 5`
- Line 95: `#define MAX_TEMPERATURE_SENSORS 5`
- Line 64: Update comment to reference One-Wire internal temp

---

### 2. `components/sensor_system/src/onewire_temp_manager.c`

**Changes:**
- Add `case 4:` to switch statement
- Map sensor 4 to `genericSens_.generic.tempx`

**Location:** Around line 197-212

**Code to add:**
```c
case 4:
    // 5th sensor (internal enclosure temperature) → tempx
    genericSens_.generic.tempx = temp_fahrenheit;
    ESP_LOGD(TAG, "Internal temp (sensor 4): %.1f°F", temp_fahrenheit);
    break;
```

---

### 3. `components/sensor_system/src/sensor_coordination.c`

**Changes:**
- Initialize `humidity = 0.0f`
- Initialize `pressurex = 0.0f`
- Initialize `tempx = 0.0f` (will be updated by One-Wire)

**Location:** Around line 65-70

**Code to add:**
```c
// Initialize environmental fields (BME280 removed, always zero)
genericSens_.generic.humidity = 0.0f;      // No humidity sensor (BME280 removed)
genericSens_.generic.pressurex = 0.0f;    // No pressure sensor (BME280 removed)
genericSens_.generic.tempx = 0.0f;        // Will be updated by One-Wire sensor 4 (internal temp)
```

---

### 4. `components/sensor_system/src/system_init.c`

**Changes:**
- Update fan control logic (lines 295-335)
- Change variable names: `bme280_temp` → `internal_temp`
- Change variable names: `bme280_valid` → `internal_temp_valid`
- Update log messages to reference "internal temp" instead of "BME280"
- Update verbose output section (lines 209-218)

**Key changes:**
- Fan control reads `genericSens_.generic.tempx` (from One-Wire sensor 4)
- Validation logic remains the same (range check, non-zero check)
- Error messages updated to reference "internal temp"

**Verbose output changes:**
- Change "TempX (BME280)" → "TempX (Internal)"
- Change "ENVIRONMENTAL (BME280)" → "ENVIRONMENTAL:"
- Add notes: "(not available)" for pressure and humidity

---

### 5. `main/main.c`

**Changes:**
- Remove BME280 initialization block (lines 205-232)
- Remove `#include "i2c_env_manager.h"`

**Code to remove:**
```c
// Initialize I2C Environmental Manager (BME280)
ret = i2c_env_manager_init();
// ... entire BME280 registration block ...
```

---

### 6. `components/sensor_system/CMakeLists.txt` (Optional)

**Decision:** Keep BME280 files for reference or remove from build?

**Option A - Keep files but don't compile:**
```cmake
# "src/i2c_env_manager.c"  # BME280 removed - using One-Wire internal temp instead
```

**Option B - Remove completely:**
- Remove `"src/i2c_env_manager.c"` from SRCS list

---

## Implementation Steps

1. ✅ Update `config.h` - Increase sensor limits
2. ✅ Update `onewire_temp_manager.c` - Add sensor 4 mapping
3. ✅ Update `sensor_coordination.c` - Initialize environmental fields
4. ✅ Update `system_init.c` - Fan control and verbose output
5. ✅ Update `main.c` - Remove BME280 initialization
6. ✅ Update `CMakeLists.txt` - Remove or comment BME280 source

---

## Testing Checklist

After implementation:
- [ ] Verify 5 One-Wire sensors detected during initialization
- [ ] Verify sensor 4 (5th sensor) updates `tempx` field
- [ ] Verify `humidity` and `pressurex` remain 0.0
- [ ] Verify fan control reads `tempx` from One-Wire
- [ ] Verify fan turns ON when `tempx >= 70.0°F`
- [ ] Verify fan turns OFF when `tempx < 70.0°F`
- [ ] Verify MQTT binary message includes correct `tempx` value
- [ ] Verify MQTT JSON message includes correct `tempx` value
- [ ] Verify system monitor output shows correct values
- [ ] Verify verbose sensor data output shows correct mapping

---

## Hardware Requirements

**One-Wire Bus:**
- GPIO6 (D6/A2) - One-Wire data line
- 4.7kΩ pull-up resistor (GPIO6 → 3.3V) ✅ Already added
- 5 DS18B20 sensors total:
  - 4 external sensors (temp1-temp4)
  - 1 internal sensor (sensor 4 → tempx)

**Internal Sensor Placement:**
- Mount DS18B20 inside ESP32 enclosure
- Connect to same One-Wire bus (GPIO6)
- Will be detected as sensor index 4 during initialization

---

## Code Statistics

**Removed:**
- ~598 lines (BME280 manager code)
- ~25 lines (BME280 initialization)
- 1 I2C device (BME280)

**Added:**
- ~10 lines (sensor 4 mapping)
- ~5 lines (environmental field initialization)

**Modified:**
- ~50 lines (fan control, verbose output, comments)

**Net Change:** ~560 lines removed, ~15 lines added/modified

---

## Interface Compatibility

**Maintained:**
- `genericSens_` structure unchanged
- Field order unchanged
- Field types unchanged
- MQTT message format unchanged

**Changed:**
- `tempx` source: BME280 → One-Wire sensor 4
- `humidity`: BME280 value → Always 0.0
- `pressurex`: BME280 value → Always 0.0

**Server Impact:**
- Server should handle `humidity = 0.0` and `pressurex = 0.0` gracefully
- `tempx` will still contain valid temperature (from One-Wire instead of BME280)

---

## Rollback Plan

If issues arise:
1. Revert code changes (git)
2. Re-enable BME280 initialization in `main.c`
3. Revert sensor limit back to 4
4. Revert fan control to use BME280

**Note:** BME280 hardware would need to be reconnected if rolling back.

---

## Related Files (Reference Only)

**BME280 Files (to be removed from build or kept for reference):**
- `components/sensor_system/src/i2c_env_manager.c` (598 lines)
- `components/sensor_system/include/i2c_env_manager.h` (~60 lines)

**Documentation to Update:**
- `WIRING_GUIDE.md` - Remove BME280 wiring section
- `CODE_STATISTICS.md` - Update line counts
- Design notes - Update sensor manager architecture docs

---

## Notes

- **One-Wire Pull-Up:** External 4.7kΩ resistor already added and verified working ✅
- **Sensor Detection:** Ensure 5th sensor is physically connected before testing
- **Sensor Order:** One-Wire sensors are detected in order - sensor 4 may not always be the internal one if sensors are added/removed
- **Future Consideration:** Could add sensor address tracking to ensure internal sensor is always mapped correctly

---

## Status

- [x] Plan documented
- [ ] Code changes implemented
- [ ] Testing completed
- [ ] Documentation updated

---

**Last Updated:** 2025-01-XX
**Next Steps:** Implement code changes as outlined above


