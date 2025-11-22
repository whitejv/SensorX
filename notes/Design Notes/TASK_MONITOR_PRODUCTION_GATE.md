# Task Monitor Production Gate Design

## Overview

This document prototypes changes to:
1. Remove boot button trigger for task monitor
2. Make task monitor output periodic (every 2 minutes)
3. Add production gate to control task monitor and system monitor messages

## Design Goals

- **Simplification**: Remove button hardware dependency
- **Automated Monitoring**: Periodic task statistics without user interaction
- **Production Control**: Gate messages behind compile-time or runtime flag
- **Backward Compatible**: Easy to enable/disable for development vs production

---

## Proposed Changes

### 1. Configuration Flags (`config.h`)

```c
// Production/Development Mode Control
#define PRODUCTION_MODE                0      // 0 = Development (verbose), 1 = Production (minimal)
#define ENABLE_TASK_MONITOR_OUTPUT     1      // Enable task monitor output (gated by PRODUCTION_MODE)
#define ENABLE_SYSTEM_MONITOR_OUTPUT   1      // Enable system monitor output (gated by PRODUCTION_MODE)

// Task Monitor Configuration
#define TASK_MONITOR_ENABLED           1      // Enable task monitor task
#define TASK_MONITOR_INTERVAL_MS       120000 // 2 minutes (120 seconds)
#define TASK_MONITOR_STACK_SIZE         6144   // Stack size (unchanged)
#define TASK_MONITOR_PRIORITY          0      // Lowest priority (unchanged)

// Remove button-related config (no longer needed)
// #define TASK_MONITOR_CHECK_INTERVAL_MS  (removed)
// #define TASK_MONITOR_DEBOUNCE_MS        (removed)
// #define TASK_MONITOR_BUTTON_PIN         (removed)
```

### 2. Task Monitor Changes (`task_monitor.c`)

**Remove:**
- Boot button GPIO configuration
- Button polling logic
- Button debouncing code
- `task_monitor_is_button_pressed()` function
- Button state tracking variables

**Add:**
- Periodic timer-based output
- Production gate check
- Simplified task loop (just periodic delay)

**Prototype Code:**

```c
// Remove button-related includes and constants
// #include <driver/gpio.h>  // No longer needed
// #define TASK_MONITOR_BUTTON_PIN          (removed)
// #define TASK_MONITOR_CHECK_INTERVAL_MS   (removed)
// #define TASK_MONITOR_DEBOUNCE_MS         (removed)

// Remove button state tracking
// static bool button_initialized = false;
// static bool last_button_state = false;
// static uint32_t last_press_time = 0;

/**
 * Check if task monitor output is enabled (production gate)
 */
static bool task_monitor_output_enabled(void) {
#if PRODUCTION_MODE
    return (ENABLE_TASK_MONITOR_OUTPUT == 1);
#else
    return true;  // Always enabled in development mode
#endif
}

/**
 * Task monitor task - outputs task statistics periodically
 */
static void vTaskMonitorTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Task Monitor Task started (interval: %d ms)", TASK_MONITOR_INTERVAL_MS);
    watchdog_register_current_task("TaskMonitor", TASK_MONITOR_INTERVAL_MS * 2);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        watchdog_task_heartbeat();
        
        // Check production gate
        if (task_monitor_output_enabled()) {
            ESP_LOGI(TAG, "Periodic task statistics (every %d ms)", TASK_MONITOR_INTERVAL_MS);
            task_monitor_print_task_info();
        } else {
            ESP_LOGD(TAG, "Task monitor output disabled (production mode)");
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_MONITOR_INTERVAL_MS));
    }
}

/**
 * Initialize task monitor (simplified - no GPIO)
 */
esp_err_t task_monitor_init(void) {
    // No GPIO initialization needed
    ESP_LOGI(TAG, "Task Monitor initialized (periodic mode, %d ms interval)", TASK_MONITOR_INTERVAL_MS);
    return ESP_OK;
}

// Remove button-related functions:
// - task_monitor_is_button_pressed() - REMOVED
```

### 3. System Monitor Changes (`system_init.c`)

**Add production gate to system monitor output:**

```c
/**
 * Check if system monitor output is enabled (production gate)
 */
static bool system_monitor_output_enabled(void) {
#if PRODUCTION_MODE
    return (ENABLE_SYSTEM_MONITOR_OUTPUT == 1);
#else
    return true;  // Always enabled in development mode
#endif
}

void vSystemMonitorTask(void *pvParameters) {
    // ... existing code ...
    
    // In the monitoring loop, gate the output:
    
    log_counter++;
    if (log_counter >= (MONITOR_LOG_INTERVAL_MS / MONITOR_INTERVAL_MS)) {
        log_counter = 0;

        // Check production gate before displaying monitoring info
        if (system_monitor_output_enabled()) {
            // ... existing monitoring output code ...
            ESP_LOGI(TAG, "--- System Monitor ---");
            ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu (HH:MM:SS)", ...);
            // ... rest of monitoring output ...
        } else {
            ESP_LOGD(TAG, "System monitor output disabled (production mode)");
        }

        // MQTT publishing (always enabled, but could be gated separately)
        if (mqtt_connected && wifi_connected) {
            // ... existing MQTT code ...
            // Note: MQTT might want separate gate: ENABLE_SYSTEM_MONITOR_MQTT
        }
    }
    
    // ... rest of task ...
}
```

### 4. Header File Changes (`task_monitor.h`)

**Remove button-related functions:**

```c
// Remove:
// bool task_monitor_is_button_pressed(void);

// Keep:
esp_err_t task_monitor_init(void);
BaseType_t task_monitor_start_task(void);
void task_monitor_print_task_info(void);
```

### 5. Main Initialization Changes (`main.c`)

**No changes needed** - initialization remains the same, just no GPIO setup.

---

## Production Gate Options

### Option A: Compile-Time Gate (Recommended)

**Pros:**
- Zero runtime overhead
- Can't accidentally enable in production
- Clear separation of dev/prod builds

**Cons:**
- Requires rebuild to change
- Can't toggle at runtime

**Implementation:**
```c
#if PRODUCTION_MODE
    #if ENABLE_TASK_MONITOR_OUTPUT
        // Output enabled
    #else
        // Output disabled
    #endif
#else
    // Development mode - always enabled
#endif
```

### Option B: Runtime Gate (Alternative)

**Pros:**
- Can toggle without rebuild
- Useful for field debugging

**Cons:**
- Small runtime overhead
- Could accidentally enable in production

**Implementation:**
```c
static bool task_monitor_enabled = true;  // Set via config or NVS

bool task_monitor_output_enabled(void) {
    if (PRODUCTION_MODE && !ENABLE_TASK_MONITOR_OUTPUT) {
        return false;
    }
    return task_monitor_enabled;
}
```

### Option C: Hybrid (Recommended for Flexibility)

**Compile-time default, runtime override:**

```c
static bool task_monitor_runtime_override = false;  // Set via command/config

bool task_monitor_output_enabled(void) {
    // Runtime override takes precedence
    if (task_monitor_runtime_override) {
        return true;
    }
    
    // Production gate
#if PRODUCTION_MODE
    return (ENABLE_TASK_MONITOR_OUTPUT == 1);
#else
    return true;  // Development mode
#endif
}
```

---

## Configuration Examples

### Development Build (`config.h`)
```c
#define PRODUCTION_MODE                0      // Development mode
#define ENABLE_TASK_MONITOR_OUTPUT     1      // Enabled
#define ENABLE_SYSTEM_MONITOR_OUTPUT   1      // Enabled
```

### Production Build (`config.h`)
```c
#define PRODUCTION_MODE                1      // Production mode
#define ENABLE_TASK_MONITOR_OUTPUT     0      // Disabled (reduce serial spam)
#define ENABLE_SYSTEM_MONITOR_OUTPUT   0      // Disabled (reduce serial spam)
```

### Production Build with Monitoring (`config.h`)
```c
#define PRODUCTION_MODE                1      // Production mode
#define ENABLE_TASK_MONITOR_OUTPUT     1      // Enabled (for debugging)
#define ENABLE_SYSTEM_MONITOR_OUTPUT   1      // Enabled (for monitoring)
```

---

## File Changes Summary

### Files to Modify:

1. **`components/sensor_system/include/config.h`**
   - Add production gate defines
   - Update task monitor interval to 120000ms
   - Remove button-related defines

2. **`components/sensor_system/src/task_monitor.c`**
   - Remove GPIO button code (~50 lines)
   - Remove button polling logic (~30 lines)
   - Add periodic timer logic (~10 lines)
   - Add production gate check (~10 lines)
   - Simplify initialization (~20 lines removed)

3. **`components/sensor_system/include/task_monitor.h`**
   - Remove `task_monitor_is_button_pressed()` declaration

4. **`components/sensor_system/src/system_init.c`**
   - Add production gate check for system monitor output
   - Gate the 5-second monitoring messages

5. **`main/main.c`**
   - No changes needed (initialization same)

### Files Unchanged:
- `components/sensor_system/CMakeLists.txt` (no changes)
- Other sensor managers (no changes)

---

## Code Size Impact

**Removed:**
- Button GPIO code: ~50 lines
- Button polling/debouncing: ~30 lines
- Button state tracking: ~10 lines
- Total removed: ~90 lines

**Added:**
- Production gate checks: ~20 lines
- Periodic timer logic: ~10 lines
- Total added: ~30 lines

**Net Change:** ~60 lines removed (simpler)

---

## Testing Checklist

- [ ] Task monitor outputs every 2 minutes (not on button press)
- [ ] No GPIO errors (button code removed)
- [ ] Production gate disables output when `PRODUCTION_MODE=1` and `ENABLE_*=0`
- [ ] Development mode always shows output
- [ ] System monitor respects production gate
- [ ] MQTT continues to work (separate gate if needed)
- [ ] Watchdog still works
- [ ] No memory leaks
- [ ] Stack usage acceptable

---

## Migration Path

1. **Phase 1**: Add production gate defines (no behavior change)
2. **Phase 2**: Remove button code, add periodic timer
3. **Phase 3**: Add gate checks to output functions
4. **Phase 4**: Test in development mode
5. **Phase 5**: Test with production gate enabled
6. **Phase 6**: Deploy to production

---

## Questions to Consider

1. **MQTT Messages**: Should MQTT system monitor messages also be gated, or always enabled?
   - Recommendation: Separate gate `ENABLE_SYSTEM_MONITOR_MQTT`

2. **Error Messages**: Should error/warning logs be gated?
   - Recommendation: Never gate errors/warnings, only info/debug

3. **Serial Bandwidth**: In production, do you want any serial output?
   - Recommendation: Keep errors/warnings, gate info/debug

4. **Runtime Control**: Do you need to toggle monitoring at runtime?
   - Recommendation: Compile-time gate is simpler and safer

---

## Recommended Implementation

**Use Option A (Compile-Time Gate)** with these settings:

**Development:**
```c
#define PRODUCTION_MODE                0
#define ENABLE_TASK_MONITOR_OUTPUT     1
#define ENABLE_SYSTEM_MONITOR_OUTPUT   1
```

**Production:**
```c
#define PRODUCTION_MODE                1
#define ENABLE_TASK_MONITOR_OUTPUT     0
#define ENABLE_SYSTEM_MONITOR_OUTPUT   0
```

This provides:
- Simple implementation
- Zero runtime overhead
- Clear dev/prod separation
- Easy to enable for debugging if needed

