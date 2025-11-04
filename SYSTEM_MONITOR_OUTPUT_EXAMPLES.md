# System Monitor Output Examples

## Example 1: System Fully Operational

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connected | Broker: 192.168.1.250 (PROD) | Pub: 3600/0 (ok/fail)
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 4 [0x20, 0x48, 0x77, 0x69]
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Breakdown:**
- **WiFi**: Shows IP address, signal strength (RSSI), and connection uptime
- **MQTT**: Shows broker IP, server type (PROD/DEV), and publish statistics (successful/failed)
- **I2C**: Shows initialization status and list of all detected device addresses

## Example 2: MQTT Connected to Development Server

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connected | Broker: 192.168.1.249 (DEV) | Pub: 3600/0 (ok/fail)
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 4 [0x20, 0x48, 0x77, 0x69]
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Key Difference:** MQTT shows "(DEV)" instead of "(PROD)" when connected to development server

## Example 3: MQTT Not Connected

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connecting
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 4 [0x20, 0x48, 0x77, 0x69]
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Key Difference:** MQTT shows status text instead of connection details when not connected

## Example 4: No I2C Devices Found

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connected | Broker: 192.168.1.250 (PROD) | Pub: 3600/0 (ok/fail)
I (5000) SYS_MONITOR: I2C: Initialized | No devices found
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Key Difference:** I2C shows "No devices found" when bus is initialized but no devices are detected

## Example 5: I2C Not Initialized

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connected | Broker: 192.168.1.250 (PROD) | Pub: 3600/0 (ok/fail)
I (5000) SYS_MONITOR: I2C: Not initialized
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Key Difference:** I2C shows "Not initialized" when initialization failed or hasn't occurred

## Example 6: MQTT with Publish Failures

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connected | Broker: 192.168.1.250 (PROD) | Pub: 3500/100 (ok/fail)
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 4 [0x20, 0x48, 0x77, 0x69]
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Key Difference:** MQTT shows 3500 successful publishes and 100 failures, indicating some connectivity issues

## Example 7: Multiple I2C Devices (Typical Sensor Setup)

```
I (5000) SYS_MONITOR: === System Monitor (5000ms interval) ===
I (5000) SYS_MONITOR: Free Heap: 234567 bytes
I (5000) SYS_MONITOR: Min Free Heap: 223456 bytes
I (5000) SYS_MONITOR: Active Tasks: 8
I (5000) SYS_MONITOR: Watchdog: feeds=1234, timeouts=0, tasks=6
I (5000) SYS_MONITOR: WiFi: Connected | IP: 192.168.1.100 | RSSI: -45 dBm | Uptime: 3600 sec
I (5000) SYS_MONITOR: MQTT: Connected | Broker: 192.168.1.250 (PROD) | Pub: 3600/0 (ok/fail)
I (5000) SYS_MONITOR: I2C: Initialized | Devices: 6 [0x20, 0x2A, 0x34, 0x48, 0x69, 0x77]
I (5000) SYS_MONITOR: --- Monitor Complete ---
```

**Device Addresses:**
- `0x20` - GPIO Expander (MCP23X17)
- `0x2A` - OpenLog datalogger
- `0x34` - Qwiic Buzzer
- `0x48` - ADS1115 (16-bit ADC)
- `0x69` - RV1805 RTC
- `0x77` - BME280 Environmental Sensor

## Notes

- **MQTT Status Values:**
  - `Connected` - Successfully connected to broker
  - `Disconnected` - Not connected
  - `Connecting` - Attempting to connect
  - `Reconnecting` - Attempting to reconnect after disconnect
  - `Error` - Connection error occurred

- **I2C Device Addresses:**
  - Addresses are shown in hexadecimal format (0xXX)
  - Maximum of 16 devices will be displayed
  - Scan occurs every monitor interval (5 seconds)
  - Addresses are scanned in order (0x08-0x77)

- **MQTT Publish Statistics:**
  - Shows successful publishes / failed publishes
  - Resets can be performed via `mqtt_manager_reset_stats()`
  - Useful for monitoring MQTT connection quality

