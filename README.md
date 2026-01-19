# SensorX

**Multi-Sensor Data Acquisition System for ESP32-C6**

SensorX is an ESP-IDF based sensor monitoring system designed for industrial and well monitoring applications. It collects data from multiple sensor types and publishes to MQTT brokers for remote monitoring and analysis.

---

## Hardware Platform

- **Board**: Adafruit ESP32-C6 Feather
- **MCU**: ESP32-C6 (RISC-V, WiFi 6, Bluetooth LE)
- **Framework**: ESP-IDF v5.x (Espressif IoT Development Framework)
- **Language**: C (with C++ component for ADC drivers)

---

## Features

### Sensor Support
- **Flow Sensors** (3x): PCNT-based pulse counting for Hall effect flow meters
- **Temperature Sensors** (5x): One-Wire DS18B20 sensors on shared bus
- **ADC Sensors** (8 channels): I2C ADS1015/ADS1115 analog-to-digital converters
- **GPIO Expansion**: MCP23017 I2C GPIO expander (16 additional I/O)
- **Environmental**: Internal temperature monitoring (One-Wire sensor 4)

### Communication
- **WiFi**: Auto-connect with reconnection logic
- **MQTT**: Dual publishing (binary QoS 2 + JSON QoS 0)
- **Topics**: Production and development broker support

### System Features
- **FreeRTOS**: Task-based architecture with priority scheduling
- **Watchdog**: Hardware watchdog with task monitoring
- **Error Recovery**: Automatic error detection and recovery strategies
- **Fan Control**: Temperature-based cooling (GPIO2, threshold: 70°F)
- **Task Monitoring**: Runtime statistics and health monitoring
- **Panic Statistics**: Reset reason tracking and crash analysis

---

## Architecture

### Sensor Manager System
Sensors are organized into manager groups by interface type and read interval:

| Manager | Interface | Interval | Priority |
|---------|-----------|----------|----------|
| PCNT Flow Manager | PCNT (hardware) | 1000ms | Critical (5) |
| I2C ADC Manager | I2C (ADS1x15) | 1000ms | Fixed Freq (3) |
| I2C GPIO Manager | I2C (MCP23017) | 1000ms | Fixed Freq (3) |
| One-Wire Temp Manager | One-Wire (DS18B20) | 5000ms | Background (1) |

### Data Structure
- **`genericSens_`**: 104-byte union structure (26 words × 4 bytes)
- **Thread-safe**: Mutex-protected concurrent access
- **Dual format**: Structured fields + raw binary array for MQTT
- **Frozen interface**: Compatible with Raspberry Pi receiving applications

### Task Coordination
- **Event Groups**: Synchronize sensor completion with MQTT publishing
- **Mutex Protection**: Thread-safe sensor data updates
- **Priority-based**: Critical sensors prioritized over background tasks

---

## Pin Assignments

| Function | GPIO | Pin | Notes |
|----------|------|-----|-------|
| **I2C Bus** | | | |
| SDA | GPIO19 | D19 | Shared by all I2C devices |
| SCL | GPIO18 | D18 | Shared by all I2C devices |
| Power Enable | GPIO20 | D20 | STEMMA QT power (auto-controlled) |
| **Flow Sensors** | | | |
| Flow Sensor 1 | GPIO7 | D7 | PCNT Unit 0 |
| Flow Sensor 2 | GPIO8 | D8 | PCNT Unit 1 |
| Flow Sensor 3 | GPIO3 | D3/A4 | PCNT Unit 2 |
| **Temperature** | | | |
| One-Wire Bus | GPIO6 | D6/A2 | Shared DS18B20 bus |
| **Control** | | | |
| Fan Control | GPIO2 | D2/A5 | HIGH = ON, LOW = OFF |

See [WIRING_GUIDE.md](WIRING_GUIDE.md) for complete wiring details.

---

## I2C Devices

| Device | Address | Function |
|--------|---------|----------|
| ADS1015 | 0x48 | 12-bit ADC (4 channels) |
| ADS1115 | 0x49 | 16-bit ADC (4 channels) |
| MCP23017 | 0x20 | GPIO Expander (16 I/O) |

---

## Configuration

Edit `components/sensor_system/include/config.h`:

```c
// WiFi Configuration
#define WIFI_SSID              "YourWiFiNetwork"
#define WIFI_PASSWORD          "YourPassword"
#define WIFI_AUTO_CONNECT      1

// MQTT Configuration
#define MQTT_PROD_SERVER_IP   "192.168.1.250"
#define MQTT_DEV_SERVER_IP    "192.168.1.249"
#define MQTT_PORT             1883

// Sensor Timing
#define SENSOR_ACQUISITION_INTERVAL  100   // ms
#define MQTT_PUBLISH_INTERVAL        1000  // ms

// Fan Control
#define FAN_CONTROL_THRESHOLD_TEMP_F  70.0  // °F
```

---

## Build & Flash

### Prerequisites
- ESP-IDF v5.x installed and configured
- USB cable for programming
- Serial monitor access

### Build Commands
```bash
# Build project
idf.py build

# Flash to device
idf.py flash

# Monitor serial output
idf.py monitor

# Build + Flash + Monitor (combined)
idf.py flash monitor
```

### Configuration Menu
```bash
idf.py menuconfig
```

---

## Project Structure

```
SensorX/
├── main/
│   └── main.c                 # Application entry point
├── components/
│   └── sensor_system/         # Main sensor system component
│       ├── include/           # Public headers
│       │   ├── config.h      # System configuration
│       │   ├── pins.h         # GPIO pin definitions
│       │   ├── sensor.h       # Data structure definitions
│       │   └── ...
│       └── src/               # Implementation files
│           ├── pcnt_flow_manager.c
│           ├── onewire_temp_manager.c
│           ├── i2c_adc_manager.cpp
│           ├── wifi_manager.c
│           ├── mqtt_manager.c
│           └── ...
├── notes/                     # Design documentation
├── WIRING_GUIDE.md           # Hardware wiring guide
└── CMakeLists.txt            # Root build configuration
```

---

## MQTT Topics

### Binary Data (QoS 2)
- **Topic**: `mwp/data/sensor/well/S005D`
- **Format**: Binary (104 bytes)
- **QoS**: 2 (exactly-once delivery)

### JSON Data (QoS 0)
- **Topic**: `mwp/json/sensor/well/S005D`
- **Format**: JSON
- **QoS**: 0 (best-effort delivery)

### System Monitor
- **Topic**: `mwp/json/sensor/well/monitor`
- **Format**: JSON system status

---

## Code Statistics

- **Total Lines**: ~7,857 lines
- **Source Files**: 5,724 lines (73%)
- **Header Files**: 2,133 lines (27%)
- **Components**: 15 source files, 18 header files

See [CODE_STATISTICS.md](CODE_STATISTICS.md) for detailed breakdown.

---

## Documentation

- **[WIRING_GUIDE.md](WIRING_GUIDE.md)**: Complete hardware wiring instructions
- **[notes/](notes/)**: Design documentation and implementation plans
- **[CODE_STATISTICS.md](CODE_STATISTICS.md)**: Code metrics and organization

---

## Key Design Principles

1. **Modular Architecture**: Component-based design with clear interfaces
2. **Thread Safety**: Mutex-protected shared data structures
3. **Error Resilience**: Watchdog, error recovery, and graceful degradation
4. **Priority Scheduling**: Critical sensors prioritized over background tasks
5. **Frozen Interface**: `genericSens_` structure maintains compatibility with receiving applications

---

## License

[Add license information if applicable]

---

## Version

**Firmware Version**: 1.3  
**ESP-IDF Version**: v5.x  
**Last Updated**: 2025-01-15
