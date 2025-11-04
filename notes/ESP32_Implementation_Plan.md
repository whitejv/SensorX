# ESP32 FreeRTOS Migration - ESP-IDF Implementation Plan

## Overview
This document outlines the step-by-step implementation plan for migrating the ESP8266 sensor system to ESP32 using the ESP-IDF (Espressif IoT Development Framework). The implementation follows a modular approach with small, testable chunks that build sequentially upon each other, providing greater control, reduced overhead, and enhanced task monitoring capabilities.

**Base Directory**: `/Volumes/iHome/Users/dub/esp/SensorZ`

**Development Environment**: ESP-IDF v5.x with CMake build system
- Native ESP32 development framework
- Direct FreeRTOS integration
- Professional-grade development tools
- Component-based architecture

**Language**: C language only (ESP-IDF C API)
- .c files for all implementation code
- .h files for header declarations
- Proper header guards and ESP-IDF logging
- No C++ features or object-oriented programming

**Debug and Logging System**:
- **ESP_LOG**: Native ESP-IDF logging with levels (ERROR, WARN, INFO, DEBUG, VERBOSE)
- **VERBOSE Mode** (0=off, 1=on): Controls detailed runtime output
- **DEBUG Mode** (0=off, 1-3=levels): Controls debug code compilation
- **Task Monitoring**: Enhanced FreeRTOS task introspection and statistics

**Architecture**: Native FreeRTOS with ESP-IDF drivers, priority-based sensor handling, and RAM-efficient buffering
- Direct hardware access without Arduino abstraction layer
- Advanced FreeRTOS task management and monitoring
- ESP-IDF component architecture for modularity

## Implementation Phases

### Phase 1: Core Framework Setup
**Goal**: Establish the ESP-IDF component framework, FreeRTOS tasks, and logging system

#### 1.1 ESP-IDF Project Structure
```
SensorZ/
├── CMakeLists.txt               # Root project CMakeLists.txt
├── sdkconfig                     # ESP-IDF configuration (generated)
├── main/
│   ├── CMakeLists.txt           # Main component CMakeLists.txt
│   └── main.c                   # ESP-IDF application entry point
├── components/                  # ESP-IDF components
│   └── sensor_system/          # Main sensor system component
│       ├── CMakeLists.txt      # Component CMakeLists.txt
│       ├── include/
│       │   ├── config.h        # System configuration constants
│       │   ├── types.h         # Data structures and enums
│       │   └── pins.h          # ESP32 pin definitions
│       ├── src/
│       │   ├── system_init.c   # System initialization tasks
│       │   └── system_monitor.c # System monitoring tasks
│       └── Kconfig             # Component configuration (optional)
├── docs/
└── notes/
```

#### 1.2 Core Component Implementation

**1.2.1 main/main.c - ESP-IDF Application Entry Point**
- ESP-IDF `app_main()` function as entry point
- Native FreeRTOS task creation using `xTaskCreate()`
- ESP_LOG system initialization
- Component initialization calls
- Task handle declarations and management
- No Arduino dependencies

**1.2.2 components/sensor_system/include/config.h**
- ESP-IDF style configuration constants
- FreeRTOS task configuration with ESP-IDF conventions
- ESP_LOG level definitions (ESP_LOG_ERROR, ESP_LOG_WARN, etc.)
- Task priorities and stack sizes optimized for ESP32
- Menuconfig integration with Kconfig

**1.2.3 components/sensor_system/include/types.h**
- Common data structures (sensor data, flow data, etc.)
- Enum definitions for sensor types and states
- Typedef declarations with ESP-IDF naming conventions
- Include guards and ESP-IDF component structure

**1.2.4 components/sensor_system/include/pins.h**
- ESP32 GPIO pin definitions (not Arduino Nano ESP32)
- Direct GPIO assignments for sensors
- I2C and SPI pin definitions using ESP-IDF GPIO numbers
- Include guards and ESP-IDF component structure

**1.2.5 components/sensor_system/src/system_init.c**
- Implementation of system initialization tasks
- Native FreeRTOS task functions
- ESP_LOG usage for all output
- ESP-IDF driver initialization
- Pure C functions with proper error handling

#### 1.3 Task Prototypes

**1.3.1 System Information Task (Setup Task)**
Runs once during system initialization to display comprehensive system information using ESP-IDF logging.

```c
// Function prototype
void vSystemInfoTask(void *pvParameters);

// Task characteristics:
// - Priority: TASK_PRIORITY_BACKGROUND (1)
// - Stack size: TASK_STACK_SIZE_BACKGROUND (2048)
// - Execution: One-time during setup
// - Purpose: Display firmware, hardware, and software versions

void vSystemInfoTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    ESP_LOGI(TAG, "=== GenericSensorX System Information ===");
    ESP_LOGI(TAG, "Firmware Version: %s v%d.%d",
             FIRMWARE_NAME, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);

    // Get ESP32 chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 Chip: %s, Revision %d, Cores: %d",
             CONFIG_IDF_TARGET, chip_info.revision, chip_info.cores);

    ESP_LOGI(TAG, "Flash Size: %d MB", spi_flash_get_chip_size() / (1024 * 1024));
    ESP_LOGI(TAG, "FreeRTOS Kernel Version: %s", tskKERNEL_VERSION_NUMBER);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());

    // CPU frequency
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", esp_clk_cpu_freq() / 1000000);

    // PSRAM information
    size_t psram_size = 0;
#if CONFIG_SPIRAM
    psram_size = esp_spiram_get_size();
    ESP_LOGI(TAG, "PSRAM Available: Yes, Size: %d MB", psram_size / (1024 * 1024));
#else
    ESP_LOGI(TAG, "PSRAM Available: No");
#endif

    ESP_LOGI(TAG, "=== System Information Complete ===\n");

    // Task completes and deletes itself
    vTaskDelete(NULL);
}
```

**1.3.2 System Monitoring Task (Continuous Task)**
Runs continuously at low priority to monitor system health and resource usage using ESP-IDF APIs.

```c
// Function prototype
void vSystemMonitorTask(void *pvParameters);

// Task characteristics:
// - Priority: TASK_PRIORITY_BACKGROUND (1)
// - Stack size: TASK_STACK_SIZE_BACKGROUND (2048)
// - Execution: Every 5 seconds continuously
// - Purpose: Monitor memory, heap, uptime, task status

void vSystemMonitorTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter

    ESP_LOGI(TAG, "System Monitor Task: Started successfully");

    // Initialize uptime tracking
    static uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Initialize monitoring variables
    static size_t lastHeapFree = esp_get_free_heap_size();
    static size_t minHeapFree = esp_get_free_heap_size();
    static UBaseType_t lastTaskCount = uxTaskGetNumberOfTasks();

    const TickType_t xMonitorInterval = pdMS_TO_TICKS(5000); // 5 seconds

    for (;;) {
        // Calculate uptime
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t uptimeMs = currentTime - startTime;
        uint32_t uptimeSeconds = uptimeMs / 1000;
        uint32_t uptimeHours = uptimeSeconds / 3600;
        uint32_t uptimeMinutes = (uptimeSeconds % 3600) / 60;
        uint32_t uptimeSecs = uptimeSeconds % 60;

        // Get current memory stats
        size_t currentHeapFree = esp_get_free_heap_size();

        // Update minimum heap tracking
        if (currentHeapFree < minHeapFree) {
            minHeapFree = currentHeapFree;
        }

        // Get task information
        UBaseType_t currentTaskCount = uxTaskGetNumberOfTasks();

        // Display monitoring information
        ESP_LOGI(TAG, "--- System Monitor ---");
        ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu (HH:MM:SS)",
                uptimeHours, uptimeMinutes, uptimeSecs);
        ESP_LOGI(TAG, "Free Heap: %zu bytes (Min: %zu bytes)",
                currentHeapFree, minHeapFree);
        ESP_LOGI(TAG, "Active Tasks: %d", currentTaskCount);

        // Show task count changes
        if (currentTaskCount != lastTaskCount) {
            ESP_LOGW(TAG, "Task count changed from %d to %d",
                    lastTaskCount, currentTaskCount);
            lastTaskCount = currentTaskCount;
        }

        // Show heap usage trend
        if (currentHeapFree != lastHeapFree) {
            int32_t heapChange = (int32_t)currentHeapFree - (int32_t)lastHeapFree;
            ESP_LOGD(TAG, "Heap change: %ld bytes", heapChange);
            lastHeapFree = currentHeapFree;
        }

        // Internal temperature sensor (if available on ESP32 variant)
#if CONFIG_IDF_TARGET_ESP32
        // ESP32 has internal temperature sensor
        // Note: Requires calibration and may not be accurate
        // ESP_LOGD(TAG, "CPU Temperature: %.1f°C", temperatureRead());
#endif

        ESP_LOGI(TAG, "--- Monitor Complete ---");

        // Wait for next monitoring interval
        vTaskDelay(xMonitorInterval);
    }
}
```

**Task Integration in main.c:**
```c
// In app_main() function
// Initialize ESP_LOG system
esp_log_level_set(TAG, ESP_LOG_INFO);

// Create system information task (runs once)
BaseType_t xResult = xTaskCreate(
    vSystemInfoTask,
    "SystemInfo",
    TASK_STACK_SIZE_BACKGROUND,
    NULL,
    TASK_PRIORITY_BACKGROUND,
    &xSystemInfoTaskHandle
);

if (xResult != pdPASS) {
    ESP_LOGE(TAG, "Failed to create system information task");
}

// Create continuous monitoring task
xResult = xTaskCreate(
    vSystemMonitorTask,
    "SysMonitor",
    TASK_STACK_SIZE_BACKGROUND,
    NULL,
    TASK_PRIORITY_BACKGROUND,
    &xSystemMonitorTaskHandle
);

if (xResult != pdPASS) {
    ESP_LOGE(TAG, "Failed to create system monitoring task");
    // Critical error - halt system
    abort();
}
```

**Testing Criteria Phase 1 (Updated):**
- Project compiles without errors
- FreeRTOS starts successfully
- System information displays correctly during startup (firmware version, ESP/FreeRTOS versions, etc.)
- Continuous monitoring task runs every 5 seconds with verbose output (uptime, heap, tasks)
- Command processor accepts serial commands (info, status, memory, tasks, reset)
- Info command spawns response task and displays comprehensive system information
- No runtime crashes or memory leaks

### Command and Response Framework ✅ IMPLEMENTED

#### Overview
Implement a background command processor using ESP-IDF console component that receives serial commands and spawns one-time response tasks for system analysis.

**Status: To be implemented in Phase 1 for ESP-IDF**
- ESP-IDF console component integration
- UART command parsing and dispatch
- Info response task with comprehensive system information
- Basic command support: info, status, memory, tasks, reset
- Enhanced task monitoring capabilities

#### 1.4.1 Command Processor Task (Background)

**Task Characteristics:**
- Priority: `TASK_PRIORITY_IDLE` (0) - Lowest priority
- Stack Size: `TASK_STACK_SIZE_IDLE` (2048)
- Execution: Continuous background processing
- Purpose: Monitor UART input and dispatch commands using ESP-IDF console

**ESP-IDF Console Integration:**
```c
// ESP-IDF console requires these includes
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"

// Command registration structure
typedef struct {
    const char *command;
    const char *help;
    esp_console_cmd_func_t func;
} command_entry_t;

// Available commands
typedef enum {
    CMD_INFO = 0,
    CMD_STATUS,
    CMD_MEMORY,
    CMD_TASKS,
    CMD_RESET,
    CMD_UNKNOWN
} CommandType_t;

// Command parsing function
CommandType_t parseCommand(const char* cmd) {
    if (strcmp(cmd, "info") == 0) return CMD_INFO;
    if (strcmp(cmd, "status") == 0) return CMD_STATUS;
    if (strcmp(cmd, "memory") == 0) return CMD_MEMORY;
    if (strcmp(cmd, "tasks") == 0) return CMD_TASKS;
    if (strcmp(cmd, "reset") == 0) return CMD_RESET;
    return CMD_UNKNOWN;
}
```

**ESP-IDF Console Setup:**
```c
// Initialize ESP-IDF console in app_main()
esp_console_repl_t *repl = NULL;
esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

// Configure UART for console
ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

// Register commands
esp_console_cmd_t cmd_info = {
    .command = "info",
    .help = "Display system information",
    .hint = NULL,
    .func = &cmd_info_func,
    .argtable = NULL
};
ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_info));

// Similar registration for other commands...

// Start console REPL
ESP_ERROR_CHECK(esp_console_start_repl(repl));
```

**Command Handler Functions:**
```c
// Info command handler
static int cmd_info_func(int argc, char **argv) {
    // Spawn info response task
    xTaskCreate(vInfoResponseTask, "InfoResp",
                TASK_STACK_SIZE_BACKGROUND,
                NULL, TASK_PRIORITY_BACKGROUND, NULL);
    return 0;
}

// Memory command handler
static int cmd_memory_func(int argc, char **argv) {
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "Free Heap: %zu bytes, Min Free: %zu bytes",
             free_heap, min_free_heap);
    return 0;
}

// Tasks command handler
static int cmd_tasks_func(int argc, char **argv) {
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    ESP_LOGI(TAG, "Active Tasks: %d", task_count);

    // Enhanced task information (ESP-IDF FreeRTOS)
    char *task_list = malloc(1024);
    if (task_list) {
        vTaskList(task_list);
        ESP_LOGI(TAG, "Task List:\n%s", task_list);
        free(task_list);
    }
    return 0;
}

// Reset command handler
static int cmd_reset_func(int argc, char **argv) {
    ESP_LOGW(TAG, "System reset requested...");
    esp_restart();
    return 0;
}
```

#### 1.4.2 Info Command Response Task

**Purpose:** Provide comprehensive system information using ESP-IDF APIs and enhanced FreeRTOS task monitoring

**Implementation:**
```c
void vInfoResponseTask(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "=== System Information ===");

    // Basic system info
    ESP_LOGI(TAG, "Firmware: %s v%d.%d", FIRMWARE_NAME,
             FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    ESP_LOGI(TAG, "Build Date: %s %s", __DATE__, __TIME__);

    // ESP32 chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 Chip: %s, Revision %d, Cores: %d",
             CONFIG_IDF_TARGET, chip_info.revision, chip_info.cores);

    // Flash size
    ESP_LOGI(TAG, "Flash Size: %d MB", spi_flash_get_chip_size() / (1024*1024));

    // ESP-IDF and FreeRTOS versions
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "FreeRTOS Kernel: %s", tskKERNEL_VERSION_NUMBER);

    // Task count and states
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    ESP_LOGI(TAG, "Active Tasks: %d", taskCount);

    // Memory information
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "Free Heap: %zu bytes", free_heap);
    ESP_LOGI(TAG, "Min Free Heap: %zu bytes", min_free_heap);

    // PSRAM information
#if CONFIG_SPIRAM
    size_t psram_size = esp_spiram_get_size();
    size_t psram_free = esp_spiram_get_free_size();
    ESP_LOGI(TAG, "PSRAM: %zu bytes total, %zu bytes free", psram_size, psram_free);
#endif

    // CPU frequency
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", esp_clk_cpu_freq() / 1000000);

    // Uptime calculation
    uint32_t uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t uptime_sec = uptime_ms / 1000;
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    uint32_t seconds = uptime_sec % 60;
    ESP_LOGI(TAG, "Uptime: %02lu:%02lu:%02lu", hours, minutes, seconds);

    // Enhanced task list (if enabled in FreeRTOS config)
    ESP_LOGI(TAG, "=== Task Details ===");
    char *task_list = malloc(1024);
    if (task_list) {
        vTaskList(task_list);
        ESP_LOGI(TAG, "Task List:\n%s", task_list);
        free(task_list);
    }

    ESP_LOGI(TAG, "=== End System Information ===");

    // Task completes and deletes itself
    vTaskDelete(NULL);
}
```

#### 1.4.3 Advanced Task Analysis (Future Enhancement)

**Runtime Statistics (requires FreeRTOS config):**
```cpp
// In FreeRTOSConfig.h, enable:
// #define configGENERATE_RUN_TIME_STATS 1
// #define configUSE_TRACE_FACILITY 1
// #define configUSE_STATS_FORMATTING_FUNCTIONS 1
// #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() configureTimerForRunTimeStats()
// #define portGET_RUN_TIME_COUNTER_VALUE() getRunTimeCounterValue()

void vAdvancedTaskInfo(void *pvParameters) {
    char taskListBuffer[1024];

    // Get task list
    vTaskList(taskListBuffer);
    Serial.println("Task List:");
    Serial.println(taskListBuffer);

    // Get runtime statistics
    vTaskGetRunTimeStats(taskListBuffer);
    Serial.println("Runtime Statistics:");
    Serial.println(taskListBuffer);

    vTaskDelete(NULL);
}
```

#### 1.4.4 Integration Points

**Task Creation in setup():**
```cpp
// Create command processor (lowest priority)
xResult = xTaskCreate(
    vCommandProcessorTask,
    "CmdProc",
    TASK_STACK_SIZE_IDLE,
    NULL,
    TASK_PRIORITY_IDLE,  // Priority 0 - lowest
    NULL
);
```

**Command Interface:**
- **Input:** Serial commands terminated by newline
- **Output:** Formatted responses with clear headers
- **Error Handling:** Unknown command feedback
- **Non-blocking:** Commands spawn separate tasks

**Design Benefits:**
- **Background processing** doesn't interfere with sensor tasks
- **One-time response tasks** minimize resource usage
- **Extensible** command system for future features
- **Priority isolation** prevents command processing from affecting real-time operations

---

### Phase 2: Platform Abstraction Layer
**Goal**: Create ESP32-specific platform code and basic I/O

**Dependencies**: Phase 1 complete

#### 2.1 Platform Files

**2.1.1 platform/esp32_platform.h**
- ESP32-specific function declarations
- Hardware initialization prototypes
- Include guards: `#ifndef ESP32_PLATFORM_H` / `#define ESP32_PLATFORM_H` / `#endif`

**2.1.2 platform/esp32_platform.c**
- ESP32 hardware initialization
- I2C bus setup with optimized timing
- Basic GPIO configuration
- Serial debugging setup

**2.1.3 utils/debug_utils.h**
- Debug function prototypes
- Logging macros
- Include guards: `#ifndef DEBUG_UTILS_H` / `#define DEBUG_UTILS_H` / `#endif`

**2.1.4 utils/debug_utils.c**
- Serial debugging functions
- printf-style logging
- System status reporting

**Testing Criteria Phase 2:**
- I2C bus initializes correctly
- Serial debugging works
- Basic GPIO operations function
- Platform code compiles and links
- Debug output shows hardware status

---

### Phase 3: Core Task Framework
**Goal**: Implement basic FreeRTOS task infrastructure

**Dependencies**: Phase 1 & 2 complete

#### 3.1 Task Management Files

**3.1.1 tasks/task_manager.h**
- Task creation and management functions
- Task synchronization primitives
- Include guards: `#ifndef TASK_MANAGER_H` / `#define TASK_MANAGER_H` / `#endif`

**3.1.2 tasks/task_manager.c**
- FreeRTOS task creation wrapper functions
- Queue and semaphore initialization
- Task monitoring and health checks

**3.1.3 tasks/watchdog.h**
- Watchdog task declarations
- Include guards: `#ifndef WATCHDOG_H` / `#define WATCHDOG_H` / `#endif`

**3.1.4 tasks/watchdog.c**
- Hardware watchdog feeding task
- System health monitoring
- Deadlock detection

**Testing Criteria Phase 3:**
- FreeRTOS tasks create successfully
- Watchdog prevents system hangs
- Task synchronization works
- Serial debug shows task status
- No memory leaks in basic operation

---

### Phase 4: Basic Sensor Framework
**Goal**: Implement core sensor data structures and basic acquisition framework

**Dependencies**: Phase 1-3 complete

#### 4.1 Sensor Data Management

**4.1.1 utils/buffer_utils.h**
- Circular buffer management
- Data validation functions
- Include guards: `#ifndef BUFFER_UTILS_H` / `#define BUFFER_UTILS_H` / `#endif`

**4.1.2 utils/buffer_utils.c**
- RAM circular buffer implementation
- CRC validation for data integrity
- Buffer overflow handling

**4.1.3 tasks/sensor_acquisition.h**
- Sensor acquisition task framework
- Include guards: `#ifndef SENSOR_ACQUISITION_H` / `#define SENSOR_ACQUISITION_H` / `#endif`

**4.1.4 tasks/sensor_acquisition.c**
- Basic sensor reading framework
- Timed sensor acquisition (100ms frames)
- Data buffering to RAM

**Testing Criteria Phase 4:**
- RAM buffer allocates correctly
- Sensor acquisition timing works
- Data validation passes
- Buffer overflow handled gracefully
- Memory usage stays within bounds

---

### Phase 5: High Priority Sensors (ADC/PWM)
**Goal**: Implement critical sensor drivers (flow, pressure, current)

**Dependencies**: Phase 1-4 complete

#### 5.1 ADC/PWM Driver Implementation

**5.1.1 drivers/ads1x15_driver.h**
- ADS1015/ADS1115 driver interface
- Include guards: `#ifndef ADS1X15_DRIVER_H` / `#define ADS1X15_DRIVER_H` / `#endif`

**5.1.2 drivers/ads1x15_driver.c**
- ADC initialization and reading
- Voltage conversion functions
- Error handling

**5.1.3 drivers/flow_sensor.h**
- Flow sensor interrupt handling
- Include guards: `#ifndef FLOW_SENSOR_H` / `#define FLOW_SENSOR_H` / `#endif`

**5.1.4 drivers/flow_sensor.c**
- Pulse counting ISR
- Flow rate calculations
- Timestamp accuracy

**Integration**: Update sensor_acquisition.c to include high-priority sensor readings

**Testing Criteria Phase 5:**
- ADC readings accurate and stable
- Flow sensor pulse counting works
- Interrupt handling doesn't block
- Sensor data appears in debug output
- High-priority timing maintained

---

### Phase 6: Medium Priority Sensors (Environmental)
**Goal**: Add temperature, humidity, and pressure sensors

**Dependencies**: Phase 1-5 complete

#### 6.1 Environmental Sensor Implementation

**6.1.1 drivers/ds18b20_driver.h**
- One-wire temperature sensor interface
- Include guards: `#ifndef DS18B20_DRIVER_H` / `#define DS18B20_DRIVER_H` / `#endif`

**6.1.2 drivers/ds18b20_driver.c**
- DS18B20 initialization and reading
- Multi-sensor support

**6.1.3 drivers/bme280_driver.h**
- Environmental sensor interface
- Include guards: `#ifndef BME280_DRIVER_H` / `#define BME280_DRIVER_H` / `#endif`

**6.1.4 drivers/bme280_driver.c**
- BME280 temperature, humidity, pressure
- Calibration and compensation

**Integration**: Extend sensor_acquisition.c for environmental readings

**Testing Criteria Phase 6:**
- Temperature readings accurate
- Humidity and pressure data valid
- Multiple DS18B20 sensors supported
- Environmental data in debug output
- Timing doesn't interfere with high-priority sensors

---

### Phase 7: Data Processing and Publishing
**Goal**: Implement data processing and MQTT communication

**Dependencies**: Phase 1-6 complete

#### 7.1 Data Processing Pipeline

**7.1.1 tasks/data_processor.h**
- Data processing task interface
- Include guards: `#ifndef DATA_PROCESSOR_H` / `#define DATA_PROCESSOR_H` / `#endif`

**7.1.2 tasks/data_processor.c**
- Raw data to final values conversion
- Filtering and validation
- Calibration application

**7.1.3 tasks/mqtt_publisher.h**
- MQTT communication interface
- Include guards: `#ifndef MQTT_PUBLISHER_H` / `#define MQTT_PUBLISHER_H` / `#endif`

**7.1.4 tasks/mqtt_publisher.c**
- WiFi and MQTT connection management
- Binary and JSON message publishing
- Connection retry logic

**Testing Criteria Phase 7:**
- Data processing produces correct values
- MQTT connects and publishes successfully
- Both binary and JSON formats work
- Publishing timing is consistent
- Network reconnection works

---

### Phase 8: SD Card Logging System
**Goal**: Implement background logging with RAM-efficient buffering

**Dependencies**: Phase 1-7 complete

#### 8.1 Logging Implementation

**8.1.1 drivers/openlog_driver.h**
- OpenLog SD card interface
- Include guards: `#ifndef OPENLOG_DRIVER_H` / `#define OPENLOG_DRIVER_H` / `#endif`

**8.1.2 drivers/openlog_driver.c**
- SD card initialization
- File creation and management
- Error handling and recovery

**8.1.3 tasks/sd_logger.h**
- SD logging task interface
- Include guards: `#ifndef SD_LOGGER_H` / `#define SD_LOGGER_H` / `#endif`

**8.1.4 tasks/sd_logger.c**
- RAM buffer to SD card flushing
- 2-5 second flush intervals
- Log file rotation

**Testing Criteria Phase 8:**
- SD card initializes correctly
- Data flushes to SD card successfully
- RAM buffer stays within limits
- Log files contain correct data
- SD card errors handled gracefully

---

### Phase 9: System Integration and Monitoring
**Goal**: Add system monitoring, NTP sync, and final integration

**Dependencies**: Phase 1-8 complete

#### 9.1 System Services

**9.1.1 tasks/system_monitor.h**
- System monitoring interface
- Include guards: `#ifndef SYSTEM_MONITOR_H` / `#define SYSTEM_MONITOR_H` / `#endif`

**9.1.2 tasks/system_monitor.c**
- Memory usage tracking
- Task health monitoring
- Performance metrics

**9.1.3 utils/time_utils.h**
- NTP and RTC utilities
- Include guards: `#ifndef TIME_UTILS_H` / `#define TIME_UTILS_H` / `#endif`

**9.1.4 utils/time_utils.c**
- NTP synchronization
- RTC management
- Timestamp generation

**9.1.5 drivers/rtc_driver.h**
- RTC interface
- Include guards: `#ifndef RTC_DRIVER_H` / `#define RTC_DRIVER_H` / `#endif`

**9.1.6 drivers/rtc_driver.c**
- RV1805 RTC driver
- Time setting and reading

**Testing Criteria Phase 9:**
- System monitoring reports accurate data
- NTP sync works correctly
- RTC maintains accurate time
- All tasks run stably
- Memory usage optimized

---

### Phase 10: Advanced Features and Optimization
**Goal**: Add WiFi management, OTA updates, and final optimizations

**Dependencies**: Phase 1-9 complete

#### 10.1 Advanced Features

**10.1.1 tasks/wifi_manager.h**
- WiFi connection management
- Include guards: `#ifndef WIFI_MANAGER_H` / `#define WIFI_MANAGER_H` / `#endif`

**10.1.2 tasks/wifi_manager.c**
- WiFi reconnection logic
- Network optimization

**10.1.3 tasks/ota_handler.h**
- OTA update interface
- Include guards: `#ifndef OTA_HANDLER_H` / `#define OTA_HANDLER_H` / `#endif`

**10.1.4 tasks/ota_handler.c**
- Over-the-air firmware updates
- Update safety checks

**Testing Criteria Phase 10:**
- WiFi reconnection works reliably
- OTA updates function correctly
- System remains stable under load
- All features integrate properly

## ESP-IDF Logging and Debug Implementation

### ESP_LOG System (Native ESP-IDF Logging)
- **Levels**: `ESP_LOG_NONE`, `ESP_LOG_ERROR`, `ESP_LOG_WARN`, `ESP_LOG_INFO`, `ESP_LOG_DEBUG`, `ESP_LOG_VERBOSE`
- **Runtime Control**: Can be set per component using `esp_log_level_set()`
- **Usage**: Provides structured logging with timestamps and component tags
- **Example**:
  ```c
  #define TAG "SENSOR_SYSTEM"
  ESP_LOGI(TAG, "Sensor value: %.2f", value);
  ESP_LOGD(TAG, "Raw ADC reading: %d", adc_raw);
  ```

### VERBOSE Mode (Runtime Control)
- **Setting**: `VERBOSE 0` (quiet) or `VERBOSE 1` (verbose)
- **Behavior**: Code is always compiled in, but execution depends on runtime flag
- **Usage**: Controls verbosity of status messages and operational output
- **Example**:
  ```c
  if (verbose_mode) {
      ESP_LOGI(TAG, "Detailed operation status");
  }
  ```

### DEBUG Mode (Compile-time Control)
- **Setting**: `DEBUG 0` (none), `1` (basic), `2` (intermediate), `3` (detailed)
- **Behavior**: Code is only compiled when DEBUG level is set
- **Cumulative**: Higher levels include all lower level debug code
- **Usage**: Controls inclusion of debugging infrastructure and detailed diagnostics

### DEBUG Level Definitions
- **DEBUG 1**: Basic debug output, sensor validation, task status
- **DEBUG 2**: Includes DEBUG 1 + performance monitoring, timing analysis
- **DEBUG 3**: Includes DEBUG 2 + detailed memory analysis, protocol dumps

### ESP-IDF Implementation Patterns
```c
// Verbose mode - always compiled, conditionally executed
void sensorRead() {
    // Always executed sensor reading code
    float value = readSensor();

    if (verbose_mode) {
        ESP_LOGI(TAG, "Sensor value: %.2f", value);
    }
}

// Debug mode - conditionally compiled
void debugSensorDiagnostics() {
    #if DEBUG >= 1
        // Basic debug code
        validateSensorData();
        ESP_LOGD(TAG, "Sensor validation passed");

        #if DEBUG >= 2
            // Intermediate debug code
            uint32_t start_time = esp_timer_get_time();
            measureTiming();
            uint32_t elapsed = esp_timer_get_time() - start_time;
            ESP_LOGD(TAG, "Timing measurement took %lu us", elapsed);

            #if DEBUG >= 3
                // Advanced debug code
                dumpRawData();
                ESP_LOGV(TAG, "Raw protocol dump: %s", protocol_buffer);
            #endif
        #endif
    #endif
}
```

### Configuration Methods
- **Menuconfig**: Configure logging levels via `idf.py menuconfig`
- **Runtime**: Set log levels with `esp_log_level_set(TAG, ESP_LOG_DEBUG)`
- **Build Flags**: `-D VERBOSE=1 -D DEBUG=2`
- **Kconfig**: Component-specific configuration options

### Enhanced Task Monitoring
ESP-IDF provides advanced FreeRTOS monitoring:
- **Task Statistics**: Runtime statistics when `configGENERATE_RUN_TIME_STATS` enabled
- **Stack Watermarks**: High watermark tracking for stack usage
- **Heap Tracing**: Detailed heap allocation tracking
- **Performance Monitoring**: CPU usage and task switching statistics

## ESP-IDF Implementation Guidelines

### Component Architecture
1. **ESP-IDF Components**: Each major subsystem is an ESP-IDF component
2. **CMakeLists.txt**: Every component has its own CMakeLists.txt
3. **Public API**: Use `include/` directory for public headers
4. **Private Implementation**: Use `src/` directory for implementation files
5. **Kconfig**: Optional configuration file for menuconfig integration

### File Structure Rules
1. **Header Guards**: Every .h file uses `#ifndef COMPONENT_FILENAME_H` / `#define COMPONENT_FILENAME_H` / `#endif`
2. **Include Order**: ESP-IDF headers first, then component public headers, then private headers
3. **Component Isolation**: Components should be self-contained with minimal external dependencies
4. **Dependencies**: Each phase depends only on previous phases

### Code Quality Standards
1. **C Language Only**: Pure C99/C11 implementation, no C++ features
2. **ESP-IDF Conventions**: Follow ESP-IDF coding standards and naming conventions
3. **Error Handling**: Use `esp_err_t` return codes and ESP_ERROR_CHECK macros
4. **Memory Management**: Prefer static allocation, use heap only when necessary
5. **Modular Design**: Function-based design with clear interfaces
6. **Documentation**: Doxygen-style comments for functions and components
7. **Logging**: Use ESP_LOG with appropriate TAG definitions

### Build and Development Process
1. **ESP-IDF Build System**: Use `idf.py build`, `idf.py flash`, `idf.py monitor`
2. **Menuconfig**: Configure project options with `idf.py menuconfig`
3. **Incremental Development**: Add one component at a time
4. **Testing**: Compile, flash, and test after each addition
5. **Integration**: Verify all existing functionality still works
6. **Debugging**: Use ESP-IDF monitor for serial output and debugging

### FreeRTOS Best Practices
1. **Task Design**: Keep tasks focused on single responsibilities using pure C functions
2. **Stack Sizing**: Monitor stack usage with uxTaskGetStackHighWaterMark()
3. **Priority Assignment**: Use clear priority hierarchy (0=highest, configMAX_PRIORITIES-1=lowest)
4. **Synchronization**: Use appropriate primitives (queues, semaphores, mutexes)
5. **Resource Management**: Avoid blocking operations in high-priority tasks
6. **C Function Pointers**: Use function pointers for callbacks and task functions
7. **Static Allocation**: Prefer static allocation over dynamic memory where possible

### Risk Management
1. **Component Isolation**: Each component fails gracefully without affecting others
2. **Error Recovery**: Implement watchdog and recovery mechanisms
3. **Validation**: Comprehensive testing before phase completion
4. **Documentation**: Update design document as implementation progresses
5. **Version Control**: Use git branches for experimental features
6. **Fallback**: Each phase maintains system in working state

## ESP-IDF Phase Completion Checklist

For each phase, verify:
- [ ] `idf.py build` completes without errors or warnings
- [ ] `idf.py flash` successfully flashes to ESP32
- [ ] `idf.py monitor` shows expected ESP_LOG output
- [ ] Basic functionality works as expected
- [ ] Memory usage stays within bounds (check with heap tracing)
- [ ] FreeRTOS tasks are running (verify with task list)
- [ ] System remains stable under test conditions
- [ ] ESP_LOG output shows expected behavior at appropriate levels
- [ ] No regressions in previous functionality
- [ ] Component isolation maintained (components can be tested independently)
- [ ] Menuconfig options work correctly (if applicable)

---

*Implementation Plan Version: 2.1 - C Language Only*
*Last Updated: October 29, 2025*
*ESP32 FreeRTOS ESP-IDF C Implementation*
