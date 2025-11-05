# Boot Button Task Monitor Design

## Overview

This document describes the design for a boot button-triggered task monitor that displays FreeRTOS task statistics in a format similar to the Unix `top` command. The monitor provides detailed per-task information including task name, execution timing (min/nominal/max), and memory usage, triggered by pressing the boot button (GPIO9).

## Design Philosophy

**Non-Intrusive**: The task monitor runs at the lowest priority and only activates when the boot button is pressed, ensuring it never interferes with critical system operations.

**Comprehensive Statistics**: Provides detailed task information including execution timing statistics and memory usage to help diagnose performance issues and identify resource bottlenecks.

**On-Demand**: Statistics are only displayed when requested (button press), avoiding constant serial output that could interfere with normal operation.

## Hardware Requirements

### Boot Button

**GPIO Pin**: GPIO9 (`PIN_BOOT0`)
- Shared with NeoPixel and Digital IO9
- Hardware pull-up resistor (typically 10kΩ)
- Active LOW (button press pulls GPIO LOW)

**Button Characteristics**:
- Mechanical debounce: ~10-50ms typical
- Software debounce required: 50ms minimum
- Edge detection: Falling edge (HIGH → LOW)

**GPIO Configuration**:
```c
gpio_config_t boot_button_config = {
    .pin_bit_mask = (1ULL << PIN_BOOT0),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,   // Enable internal pull-up
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE     // Interrupt on falling edge
};
```

## Task Design

### Task Configuration

**Priority**: `TASK_PRIORITY_IDLE` (0) - Lowest possible priority
- Ensures all other tasks can preempt the monitor
- Only executes when no other tasks are ready
- Completely non-interfering with system operation

**Stack Size**: `TASK_STACK_SIZE_IDLE` (1536 bytes)
- Task statistics gathering requires minimal stack
- Room for formatting buffers and string operations
- Conservative size to minimize memory footprint

**Execution Model**: Event-driven (button press) + periodic check
- Button press triggers immediate statistics display
- Task polls button state periodically (low frequency)
- No continuous execution overhead

**Watchdog Timeout**: 10000ms (10 seconds)
- Very long timeout since task only runs briefly
- Protects against hung operations
- Allows time for statistics gathering and formatting

### Task Lifecycle

```
1. Task initialization
   a. Configure GPIO9 as input with pull-up
   b. Configure GPIO interrupt for falling edge
   c. Initialize button debounce state
   d. Initialize task statistics tracking
   e. Register with watchdog system
   f. Enable FreeRTOS runtime statistics (if not already enabled)

2. Main task loop:
   a. Wait for button press event (GPIO interrupt or polling)
   b. Debounce button press (50ms delay)
   c. Verify button still pressed after debounce
   d. Gather task statistics:
      - Iterate through all tasks
      - Get task name, state, priority
      - Get execution time statistics (min/nominal/max)
      - Get stack usage (high water mark)
      - Get stack size (allocated)
   e. Format and display statistics table
   f. Wait for button release (optional - prevents multiple triggers)
   g. Reset debounce state
   h. Delay until next check interval (1000ms)
```

## Statistics Collection

### FreeRTOS Runtime Statistics

**Configuration Requirements** (already enabled in `sdkconfig`):
```c
// In FreeRTOSConfig.h or sdkconfig:
CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
CONFIG_FREERTOS_USE_TRACE_FACILITY=y
CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U32=y
```

**Runtime Counter Setup**:
```c
// Requires configuration of timer for runtime stats
// ESP-IDF provides this automatically via systimer
// Counter resolution: 1 tick = 1/1000000 seconds (1μs) at 80MHz
```

### Task Information Collection

**Task Enumeration**:
- Use `uxTaskGetNumberOfTasks()` to get total task count
- Iterate through tasks using `uxTaskGetSystemState()` or task list
- Note: Some tasks may be created dynamically

**Per-Task Statistics**:

1. **Task Name**:
   ```c
   char task_name[configMAX_TASK_NAME_LEN];
   pcTaskGetName(task_handle, task_name, configMAX_TASK_NAME_LEN);
   ```

2. **Task State**:
   ```c
   eTaskState state = eTaskGetState(task_handle);
   // States: Running, Ready, Blocked, Suspended, Deleted
   ```

3. **Task Priority**:
   ```c
   UBaseType_t priority = uxTaskPriorityGet(task_handle);
   ```

4. **Execution Time Statistics**:
   ```c
   TaskStatus_t task_status;
   // Get runtime statistics for the task
   // Requires runtime stats enabled
   TaskStatus_t *task_status_array = malloc(sizeof(TaskStatus_t) * num_tasks);
   uint32_t total_runtime;
   UBaseType_t num_tasks_found = uxTaskGetSystemState(
       task_status_array,
       num_tasks,
       &total_runtime
   );
   
   // For each task:
   uint32_t task_runtime = task_status_array[i].ulRunTimeCounter;
   float cpu_percent = (task_runtime * 100.0) / total_runtime;
   ```

5. **Stack Usage**:
   ```c
   UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(task_handle);
   UBaseType_t stack_size = task_status_array[i].usStackHighWaterMark;
   // Calculate used stack: stack_size - stack_high_water
   ```

6. **Execution Timing** (min/nominal/max):
   - **Minimum**: Track minimum execution time per task cycle
   - **Nominal**: Average execution time or typical execution time
   - **Maximum**: Track maximum execution time per task cycle
   - **Note**: Requires custom tracking as FreeRTOS doesn't provide per-task cycle timing

## Custom Timing Statistics

### Problem: FreeRTOS Runtime Stats Limitations

**FreeRTOS Runtime Statistics**:
- Provides total CPU time per task (cumulative)
- Does not provide per-cycle execution times
- Does not provide min/nominal/max per cycle

**Solution**: Custom timing tracking per task

### Implementation Approach

**Option 1: Task Hook-Based Tracking** (Recommended)
- Use `vTaskSwitchContext` hook to track task switches
- Measure time between task start and task yield/suspend
- Store min/nominal/max per task in task-local storage or global array

**Option 2: Task Wrapper Function**
- Wrap each task function with timing code
- Measure execution time at start and end of each task cycle
- Update statistics after each execution

**Option 3: Periodic Sampling**
- Sample task execution time periodically
- Use `xTaskGetTickCount()` or `esp_timer_get_time()` for timing
- Store samples and calculate min/nominal/max

**Recommended: Option 1 with Task Local Storage**

```c
typedef struct {
    uint32_t min_exec_time_us;     // Minimum execution time (microseconds)
    uint32_t nominal_exec_time_us; // Nominal/average execution time
    uint32_t max_exec_time_us;     // Maximum execution time
    uint32_t execution_count;      // Number of executions tracked
    uint32_t last_start_time_us;   // Start time of current execution
} TaskTimingStats_t;

// Per-task timing statistics (indexed by task handle)
static TaskTimingStats_t task_timing_stats[MAX_TASKS];
```

## Statistics Display Format

### Output Format (Similar to `top`)

```
================================================================================
Task Statistics (Boot Button Pressed) - Uptime: 00:15:32
================================================================================
Task Name          | Priority | State   | CPU% | Time(us) Min/Nom/Max | Stack
-------------------|----------|---------|------|----------------------|--------
SysMonitor         |    1     | Blocked |  2.5 |   45 /  125 /  890   | 512/2048
PcntFlowMgr        |    5     | Running | 15.3 |  120 /  340 /  2100  | 768/4096
MqttPub            |    3     | Blocked |  8.2 |   89 /  245 /  1200  | 456/3072
WatchdogTask       |    1     | Blocked |  0.5 |   12 /   25 /   45   | 128/2048
Heartbeat          |    1     | Blocked |  0.1 |    5 /   10 /   20   | 256/2048
WiFiManager        |    2     | Blocked |  1.2 |   34 /   67 /  156   | 512/3072
SensorAcq          |    5     | Blocked |  3.8 |   78 /  198 /  567   | 1024/4096
I2C_ADC_Mgr        |    3     | Blocked |  4.5 |   95 /  234 /  678   | 768/3072
I2C_GPIO_Mgr       |    3     | Blocked |  2.1 |   45 /  123 /  345   | 512/3072
GPIO_Disc_Mgr      |    3     | Blocked |  0.8 |   18 /   45 /   89   | 384/3072
I2C_Env_Mgr        |    1     | Blocked |  1.5 |   123 /  456 /  1234  | 1024/2048
OneWireTempMgr     |    1     | Blocked |  0.2 |   34 /  890 /  3540  | 1536/2048
TaskMonitor        |    0     | Running |  0.1 |    5 /   12 /   25   | 512/1536
idle               |    0     | Running | 60.1 | 1200 / 5600 / 8900  | 256/1536
Tmr Svc            |    1     | Blocked |  0.0 |    0 /    0 /    0    | 128/2048
================================================================================
Total Tasks: 15 | Total CPU: 100.0% | Free Heap: 280376 bytes
================================================================================
```

### Column Descriptions

1. **Task Name**: Task name (max 16 characters, configurable)
2. **Priority**: Task priority (0 = idle, higher = more priority)
3. **State**: Current task state (Running, Ready, Blocked, Suspended)
4. **CPU%**: Percentage of total CPU time used by this task
5. **Time(us) Min/Nom/Max**: Execution time statistics in microseconds
   - **Min**: Minimum execution time per cycle
   - **Nom**: Nominal (average) execution time per cycle
   - **Max**: Maximum execution time per cycle
6. **Stack**: Stack usage / Stack size (bytes)

### Additional Information

**Header Section**:
- Timestamp or uptime
- Trigger source (boot button)
- System uptime

**Footer Section**:
- Total task count
- Total CPU usage (should be ~100%)
- Free heap memory
- Minimum free heap since boot

## Implementation Details

### Button Detection

**GPIO Interrupt Approach** (Recommended):
```c
static void IRAM_ATTR boot_button_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Signal the task monitor task
    xTaskNotifyFromISR(task_monitor_handle, BOOT_BUTTON_PRESSED, 
                       eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// In task initialization:
gpio_install_isr_service(0);
gpio_isr_handler_add(PIN_BOOT0, boot_button_isr_handler, NULL);
```

**Polling Approach** (Alternative):
```c
// Simpler, no ISR required
static bool is_button_pressed(void) {
    return gpio_get_level(PIN_BOOT0) == 0;  // Active LOW
}

// In task loop:
if (is_button_pressed()) {
    vTaskDelay(pdMS_TO_TICKS(50));  // Debounce delay
    if (is_button_pressed()) {
        // Button still pressed - trigger display
        display_task_statistics();
    }
}
```

### Debounce Logic

**Simple Debounce**:
```c
#define BUTTON_DEBOUNCE_MS 50

static bool button_pressed = false;
static uint32_t last_press_time = 0;

bool check_button_press(void) {
    bool current_state = (gpio_get_level(PIN_BOOT0) == 0);
    uint32_t current_time = xTaskGetTickCount();
    
    if (current_state && !button_pressed) {
        // Button just pressed
        if ((current_time - last_press_time) > pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            button_pressed = true;
            last_press_time = current_time;
            return true;
        }
    } else if (!current_state && button_pressed) {
        // Button released
        button_pressed = false;
    }
    
    return false;
}
```

### Statistics Gathering Function

```c
void display_task_statistics(void) {
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    
    // Allocate array for task status
    TaskStatus_t *task_status_array = malloc(sizeof(TaskStatus_t) * num_tasks);
    if (task_status_array == NULL) {
        ESP_LOGE(TAG, "Failed to allocate task status array");
        return;
    }
    
    // Get system state
    uint32_t total_runtime;
    UBaseType_t num_tasks_found = uxTaskGetSystemState(
        task_status_array,
        num_tasks,
        &total_runtime
    );
    
    // Print header
    print_statistics_header();
    
    // Print task statistics
    for (UBaseType_t i = 0; i < num_tasks_found; i++) {
        TaskStatus_t *task = &task_status_array[i];
        
        // Get task name
        char task_name[configMAX_TASK_NAME_LEN];
        pcTaskGetName(task->xHandle, task_name, configMAX_TASK_NAME_LEN);
        
        // Get task state
        eTaskState state = eTaskGetState(task->xHandle);
        const char *state_str = task_state_to_string(state);
        
        // Calculate CPU percentage
        float cpu_percent = (task->ulRunTimeCounter * 100.0) / total_runtime;
        
        // Get stack usage
        UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(task->xHandle);
        UBaseType_t stack_size = task->usStackHighWaterMark;
        UBaseType_t stack_used = stack_size - stack_high_water;
        
        // Get timing statistics (from custom tracking)
        TaskTimingStats_t *timing = get_task_timing_stats(task->xHandle);
        
        // Print task row
        printf("%-16s | %8d | %-8s | %5.1f | %5lu / %5lu / %5lu | %4lu/%4lu\n",
               task_name,
               task->uxCurrentPriority,
               state_str,
               cpu_percent,
               timing->min_exec_time_us,
               timing->nominal_exec_time_us,
               timing->max_exec_time_us,
               stack_used,
               stack_size);
    }
    
    // Print footer
    print_statistics_footer();
    
    free(task_status_array);
}
```

### Task Timing Hook Implementation

```c
// Task switch hook (called on every task switch)
void vApplicationTaskSwitchHook(void) {
    static TaskHandle_t last_task = NULL;
    static uint32_t last_switch_time = 0;
    
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    uint32_t current_time = esp_timer_get_time();  // Microseconds
    
    if (last_task != NULL && last_task != current_task) {
        // Calculate execution time for last task
        uint32_t exec_time = current_time - last_switch_time;
        
        // Update timing statistics for last task
        update_task_timing_stats(last_task, exec_time);
    }
    
    last_task = current_task;
    last_switch_time = current_time;
}

// Update timing statistics
void update_task_timing_stats(TaskHandle_t task_handle, uint32_t exec_time_us) {
    TaskTimingStats_t *stats = get_task_timing_stats(task_handle);
    if (stats == NULL) {
        // Task not in tracking array - add it
        stats = add_task_timing_stats(task_handle);
    }
    
    if (stats != NULL) {
        stats->execution_count++;
        
        // Update minimum
        if (exec_time_us < stats->min_exec_time_us || stats->execution_count == 1) {
            stats->min_exec_time_us = exec_time_us;
        }
        
        // Update maximum
        if (exec_time_us > stats->max_exec_time_us) {
            stats->max_exec_time_us = exec_time_us;
        }
        
        // Update nominal (exponential moving average)
        if (stats->execution_count == 1) {
            stats->nominal_exec_time_us = exec_time_us;
        } else {
            // EMA: new = alpha * current + (1-alpha) * old
            float alpha = 0.1;  // Smoothing factor
            stats->nominal_exec_time_us = 
                (uint32_t)(alpha * exec_time_us + (1.0 - alpha) * stats->nominal_exec_time_us);
        }
    }
}
```

## Configuration

### FreeRTOS Configuration

**Required in `sdkconfig`** (already enabled):
```c
CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
CONFIG_FREERTOS_USE_TRACE_FACILITY=y
CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U32=y
```

**Required in `FreeRTOSConfig.h`** (if using hooks):
```c
#define configUSE_TRACE_FACILITY 1
#define configUSE_APPLICATION_TASK_TAG 0  // Not needed for basic hook
// Note: Task switch hook needs to be enabled in FreeRTOS port
```

### Task Monitor Configuration

**Configuration in `config.h`**:
```c
// Task Monitor Configuration
#define TASK_MONITOR_PRIORITY           0      // Lowest priority (idle priority)
#define TASK_MONITOR_STACK_SIZE         1536   // Stack size (bytes)
#define TASK_MONITOR_CHECK_INTERVAL_MS  1000   // Button check interval
#define TASK_MONITOR_DEBOUNCE_MS        50     // Button debounce time
#define TASK_MONITOR_MAX_TASKS          32     // Maximum tasks to track
#define TASK_MONITOR_TIMING_SMOOTHING   0.1    // EMA smoothing factor (0.0-1.0)
```

### GPIO Configuration

**Button Pin**:
```c
#define TASK_MONITOR_BUTTON_PIN  PIN_BOOT0  // GPIO9
```

## Integration Points

### Dependencies

1. **FreeRTOS**: Task statistics and runtime stats
2. **GPIO Driver**: Button input
3. **ESP Timer**: High-resolution timing (optional, for microsecond precision)
4. **ESP Log**: Serial output formatting

### Initialization Order

```
1. Initialize GPIO system
2. Initialize task monitor module
3. Configure boot button GPIO
4. Create task monitor task (lowest priority)
5. Task registers with watchdog
6. Task configures button interrupt (if using interrupt approach)
```

### Task Creation

```c
// In app_main() or system initialization
TaskHandle_t task_monitor_handle = NULL;

xTaskCreate(
    vTaskMonitorTask,
    "TaskMonitor",
    TASK_MONITOR_STACK_SIZE,
    NULL,
    TASK_MONITOR_PRIORITY,
    &task_monitor_handle
);
```

## Error Handling

### Button Detection Failures

**GPIO Configuration Error**:
- Log error and continue without button monitoring
- Task still runs but button won't trigger display
- System continues normal operation

**Interrupt Registration Error**:
- Fall back to polling mode
- Log warning
- Continue operation

### Statistics Gathering Failures

**Memory Allocation Failure**:
- Log error
- Skip statistics display
- Continue task operation

**Task Status Array Too Small**:
- Log warning
- Display statistics for available tasks
- Note that some tasks may be missing

**Invalid Task Handle**:
- Skip invalid tasks
- Continue with remaining tasks
- Log warning if many invalid handles

## Performance Considerations

### CPU Overhead

**Task Switch Hook**:
- Called on every task switch
- Minimal overhead (~1-2μs per switch)
- Negligible impact on system performance

**Statistics Gathering**:
- Only executes on button press
- Brief execution (~10-50ms)
- No continuous overhead

**Button Polling**:
- Very low frequency (1000ms interval)
- Minimal CPU usage (~0.001%)

### Memory Overhead

**Task Status Array**:
- Temporary allocation during statistics display
- Freed immediately after display
- Size: `sizeof(TaskStatus_t) * num_tasks`

**Timing Statistics Array**:
- Persistent allocation per task
- Size: `sizeof(TaskTimingStats_t) * MAX_TASKS`
- Typical: ~32 tasks × 24 bytes = 768 bytes

**Total Memory**: ~1KB for timing statistics + temporary arrays

## Testing Considerations

### Unit Testing

1. **Button Detection**:
   - Test button press detection
   - Test debounce logic
   - Test multiple rapid presses

2. **Statistics Gathering**:
   - Test with various task counts
   - Test with dynamically created/deleted tasks
   - Test timing statistics accuracy

3. **Display Formatting**:
   - Test with long task names
   - Test with large timing values
   - Test with various stack sizes

### Integration Testing

1. **System Impact**:
   - Verify no impact on other tasks
   - Verify statistics accuracy during normal operation
   - Verify button press doesn't interfere with critical tasks

2. **Long-Term Operation**:
   - Run for extended period
   - Verify no memory leaks
   - Verify timing statistics remain accurate

### Hardware Testing

1. **Button Operation**:
   - Test button press detection
   - Test debounce behavior
   - Test multiple presses

2. **Serial Output**:
   - Verify output format is readable
   - Verify all columns display correctly
   - Verify scrolling works (if many tasks)

## Future Enhancements

### Additional Statistics

1. **Wait Time Statistics**:
   - Time spent blocked/waiting
   - Wait reason (semaphore, queue, delay, etc.)

2. **Context Switch Count**:
   - Number of times task was switched in
   - Number of times task was switched out

3. **Queue Usage**:
   - Queue depth statistics
   - Queue send/receive counts

### Enhanced Display

1. **Sortable Columns**:
   - Sort by CPU%, execution time, stack usage, etc.

2. **Filtering**:
   - Filter by priority, state, etc.

3. **Historical Data**:
   - Store statistics over time
   - Display trends

### Alternative Triggers

1. **Serial Command**:
   - Trigger via serial command (e.g., "stats")
   - More convenient than button press

2. **Periodic Display**:
   - Option to display statistics periodically
   - Configurable interval

3. **MQTT Command**:
   - Trigger via MQTT command
   - Remote statistics access

## Summary

The Boot Button Task Monitor design:

- ✅ **Non-Intrusive**: Lowest priority, event-driven, minimal overhead
- ✅ **Comprehensive**: Task name, priority, state, CPU%, timing stats, memory usage
- ✅ **Custom Timing**: Min/nominal/max execution times per task
- ✅ **On-Demand**: Only displays when boot button pressed
- ✅ **Robust**: Error handling, debouncing, memory management
- ✅ **Configurable**: Timing, formatting, display options

**Key Design Decisions**:
1. **Lowest Priority**: Ensures zero interference with system operation
2. **Button-Triggered**: On-demand display avoids continuous serial output
3. **Custom Timing Tracking**: Task switch hook provides min/nominal/max timing
4. **Debounced Input**: Prevents false triggers from button bounce
5. **Temporary Allocation**: Statistics arrays allocated only when needed

This design provides comprehensive task monitoring capabilities while maintaining complete non-interference with system operation.

