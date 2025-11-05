# GenericSensor Interface Specification (X001D)

## Overview

This document defines the **genericSensor** (`genericSens_`) interface specification for the SensorX ESP32-C6 system. This is a **critical interface** that must match exactly as receiving applications on Raspberry Pi depend on this exact structure.

**⚠️ IMPORTANT**: This interface specification is **frozen** and must not be modified without updating all receiving applications. Any changes to field order, types, or sizes will break compatibility with existing Raspberry Pi applications.

---

## Block Information

- **Block ID**: `X001D`
- **Block Name**: `genericSens`
- **Description**: Generic Sensor Data
- **From**: `generic`
- **Category**: `sensor`
- **Type**: `data`
- **MQTT Client ID**: `Generic Flow Client`
- **MQTT Topic ID**: `mwp/data/sensor/generic/X001D`
- **MQTT JSON Topic**: `mwp/json/data/sensor/generic/X001D`
- **Message Length**: `26` words (104 bytes total)

---

## Data Structure Definition

### C Union Structure

The `genericSens_` structure is implemented as a union to allow both structured field access and raw binary array access for MQTT transmission:

```c
#define GENERICSENS_LEN 26

union GENERICSENS_ {
    int data_payload[GENERICSENS_LEN];  // Raw array for binary MQTT transmission
    
    struct {
        int   flowData1;         // Word 0: Flow sensor 1
        int   flowData2;         // Word 1: Flow sensor 2
        int   flowData3;         // Word 2: Flow sensor 3
        int   adc_sensor;        // Word 3: ADC Raw Sensor value
        int   gpio_sensor;       // Word 4: GPIO Sensor Data
        int   temp1;             // Word 5: Temp f (int)
        int   temp1_f;           // Word 6: Temperature in F Float Bytes 1&2
        int   tempSensorcount;   // Word 7: Number of Temp Sensors Detected
        int   cycle_count;       // Word 8: Cycle Counter
        int   fw_version;        // Word 9: FW Version (4 Hex)
        int   GPIO_x1;           // Word 10: Extended sensor GPIO - 1
        int   GPIO_x2;           // Word 11: Extended sensor GPIO - 2
        float adc_x1;            // Word 12: Extended sensor ADC - 1
        float adc_x2;            // Word 13: Extended sensor ADC - 2
        float adc_x3;            // Word 14: Extended sensor ADC - 3
        float adc_x4;            // Word 15: Extended sensor ADC - 4
        float adc_x5;            // Word 16: Extended sensor ADC - 5
        float adc_x6;            // Word 17: Extended sensor ADC - 6
        float adc_x7;            // Word 18: Extended sensor ADC - 7
        float adc_x8;            // Word 19: Extended sensor ADC - 8
        float tempx;             // Word 20: System Temp (extended sensor)
        float pressurex;         // Word 21: Atmospheric Pressure
        float humidity;           // Word 22: Humidity (%)
        float temp2;             // Word 23: Temp sensor 2
        float temp3;             // Word 24: Temp sensor 3
        float temp4;             // Word 25: Temp sensor 4
    } generic;
};

union GENERICSENS_ genericSens_;
```

### Complete Field Specification

| Word # | Data Type | Variable Name | Description | Min | Max | Nominal | Notes |
|--------|-----------|---------------|-------------|-----|-----|---------|-------|
| 0 | `int` | `flowData1` | Flow sensor 1 | 0 | 0 | 0 | Bit-packed: pulses (12), milliseconds (19), newData (1) |
| 1 | `int` | `flowData2` | Flow sensor 2 | 0 | 0 | 0 | Bit-packed: pulses (12), milliseconds (19), newData (1) |
| 2 | `int` | `flowData3` | Flow sensor 3 | 0 | 0 | 0 | Bit-packed: pulses (12), milliseconds (19), newData (1) |
| 3 | `int` | `adc_sensor` | ADC Raw Sensor value | 0 | 1023 | 512 | Bit 0-9 (0-1023) - Water pressure (psi) |
| 4 | `int` | `gpio_sensor` | GPIO Sensor Data | 0 | 3 | 3 | Bit 1: GPIO 4, Bit 2: GPIO 5 - GPIO discretes |
| 5 | `int` | `temp1` | Temp f (int) | -32 | 150 | 80 | Integer temperature in Fahrenheit |
| 6 | `int` | `temp1_f` | Temperature in F Float Bytes 1&2 | - | - | - | Float representation stored as int (for binary compatibility) |
| 7 | `int` | `tempSensorcount` | Number of Temp Sensors Detected | 0 | 4 | - | One-Wire sensor count |
| 8 | `int` | `cycle_count` | Cycle Counter | 0 | 28800 | - | Main loop cycle counter |
| 9 | `int` | `fw_version` | FW Version | - | - | - | 4 Hex digits - Firmware version |
| 10 | `int` | `GPIO_x1` | Extended sensor GPIO - 1 | - | - | - | MCP23X17 Port A - Discrete IN |
| 11 | `int` | `GPIO_x2` | Extended sensor GPIO - 2 | - | - | - | MCP23X17 Port B - Discrete OUT |
| 12 | `float` | `adc_x1` | Extended sensor ADC - 1 | - | - | - | Pump amperage on/off |
| 13 | `float` | `adc_x2` | Extended sensor ADC - 2 | - | - | - | Pump amperage on/off |
| 14 | `float` | `adc_x3` | Extended sensor ADC - 3 | - | - | - | Pump amperage on/off |
| 15 | `float` | `adc_x4` | Extended sensor ADC - 4 | - | - | - | Pump amperage on/off |
| 16 | `float` | `adc_x5` | Extended sensor ADC - 5 | - | - | - | Tank Hydrostatic Pressure |
| 17 | `float` | `adc_x6` | Extended sensor ADC - 6 | - | - | - | Water Pressure |
| 18 | `float` | `adc_x7` | Extended sensor ADC - 7 | - | - | - | Water Pressure |
| 19 | `float` | `adc_x8` | Extended sensor ADC - 8 | - | - | - | Water Pressure |
| 20 | `float` | `tempx` | System Temp (extended sensor) | - | - | - | BME280 temperature (F) |
| 21 | `float` | `pressurex` | Atmospheric Pressure | - | - | - | BME280 pressure (PSI) |
| 22 | `float` | `humidity` | Humidity (%) | 0 | 100 | - | BME280 humidity (%) |
| 23 | `float` | `temp2` | Temp sensor 2 | -32 | 125 | 99 | One-Wire DS18B20 sensor 2 |
| 24 | `float` | `temp3` | Temp sensor 3 | -32 | 125 | 99 | One-Wire DS18B20 sensor 3 |
| 25 | `float` | `temp4` | Temp sensor 4 | -32 | 125 | 99 | One-Wire DS18B20 sensor 4 |

---

## FlowData Bit-Packed Structure

The `flowData1`, `flowData2`, and `flowData3` fields are bit-packed integers containing flow sensor information:

```c
typedef struct {
    uint32_t pulses : 12;       // 12 bits for pulse count (0-4095)
    uint32_t milliseconds : 19;  // 19 bits for milliseconds (0-524287)
    uint32_t newData : 1;       // 1 bit for new data flag
} FlowData_t;
```

**FlowData Access Pattern**:
```c
// Extract FlowData from int
FlowData_t flow1;
memcpy(&flow1, &genericSens_.generic.flowData1, sizeof(int));

// Access fields
uint32_t pulses = flow1.pulses;           // 0-4095
uint32_t milliseconds = flow1.milliseconds; // 0-524287 (max ~8.7 minutes)
bool newData = flow1.newData;             // 0 or 1
```

---

## MQTT Configuration

### Constants

```c
const char GENERICSENS_CLIENTID[] = "Generic Flow Client";
const char GENERICSENS_TOPICID[] = "mwp/data/sensor/generic/X001D";
const char GENERICSENS_JSONID[] = "mwp/json/data/sensor/generic/X001D";
#define GENERICSENS_LEN 26
```

### Binary Transmission

For binary MQTT transmission, use the `data_payload` array directly:

```c
// Publish binary payload
mqtt_manager_publish_binary(
    GENERICSENS_TOPICID,
    (uint8_t*)genericSens_.data_payload,
    GENERICSENS_LEN * sizeof(int),  // 26 * 4 = 104 bytes
    0,  // QoS
    false  // Retain
);
```

### JSON Transmission

For JSON transmission, use the structured fields with the JSON variable names:

```c
const char* genericsens_ClientData_var_name[] = {
    "X001D:flowData1",
    "X001D:flowData2",
    "X001D:flowData3",
    "X001D:adc_sensor",
    "X001D:gpio_sensor",
    "X001D:temp1",
    "X001D:temp1_f",
    "X001D:tempSensorcount",
    "X001D:cycle_count",
    "X001D:fw_version",
    "X001D:GPIO_x1",
    "X001D:GPIO_x2",
    "X001D:adc_x1",
    "X001D:adc_x2",
    "X001D:adc_x3",
    "X001D:adc_x4",
    "X001D:adc_x5",
    "X001D:adc_x6",
    "X001D:adc_x7",
    "X001D:adc_x8",
    "X001D:tempx",
    "X001D:pressurex",
    "X001D:humidity",
    "X001D:temp2",
    "X001D:temp3",
    "X001D:temp4"
};
```

**JSON Publishing Example**:
```c
cJSON *json = cJSON_CreateObject();

// Flow data (bit-packed, extract first)
FlowData_t flow1, flow2, flow3;
memcpy(&flow1, &genericSens_.generic.flowData1, sizeof(int));
memcpy(&flow2, &genericSens_.generic.flowData2, sizeof(int));
memcpy(&flow3, &genericSens_.generic.flowData3, sizeof(int));

cJSON_AddNumberToObject(json, "X001D:W1_NewData", flow1.newData);
cJSON_AddNumberToObject(json, "X001D:W1_mSeconds", flow1.milliseconds);
cJSON_AddNumberToObject(json, "X001D:W1_Pulses", flow1.pulses);
cJSON_AddNumberToObject(json, "X001D:W2_NewData", flow2.newData);
cJSON_AddNumberToObject(json, "X001D:W2_mSeconds", flow2.milliseconds);
cJSON_AddNumberToObject(json, "X001D:W2_Pulses", flow2.pulses);
cJSON_AddNumberToObject(json, "X001D:W3_NewData", flow3.newData);
cJSON_AddNumberToObject(json, "X001D:W3_mSeconds", flow3.milliseconds);
cJSON_AddNumberToObject(json, "X001D:W3_Pulses", flow3.pulses);

// Integer fields (words 3-11)
for (int i = 3; i < 12; i++) {
    cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[i], 
                            genericSens_.data_payload[i]);
}

// Float fields (words 12-25)
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[12], genericSens_.generic.adc_x1);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[13], genericSens_.generic.adc_x2);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[14], genericSens_.generic.adc_x3);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[15], genericSens_.generic.adc_x4);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[16], genericSens_.generic.adc_x5);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[17], genericSens_.generic.adc_x6);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[18], genericSens_.generic.adc_x7);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[19], genericSens_.generic.adc_x8);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[20], genericSens_.generic.tempx);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[21], genericSens_.generic.pressurex);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[22], genericSens_.generic.humidity);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[23], genericSens_.generic.temp2);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[24], genericSens_.generic.temp3);
cJSON_AddNumberToObject(json, genericsens_ClientData_var_name[25], genericSens_.generic.temp4);

char *json_string = cJSON_Print(json);
mqtt_manager_publish_json(GENERICSENS_JSONID, json_string, 0, false);
free(json_string);
cJSON_Delete(json);
```

---

## Memory Layout

### Structure Size

- **Total Size**: 104 bytes (26 words × 4 bytes per word)
- **Integer Fields**: 12 words (48 bytes) - words 0-11
- **Float Fields**: 14 words (56 bytes) - words 12-25

### Memory Layout (Little-Endian)

```
Offset  Field              Type      Size
------- ------------------ --------- ----
0x00    flowData1          int       4
0x04    flowData2          int       4
0x08    flowData3          int       4
0x0C    adc_sensor         int       4
0x10    gpio_sensor        int       4
0x14    temp1              int       4
0x18    temp1_f            int       4
0x1C    tempSensorcount    int       4
0x20    cycle_count        int       4
0x24    fw_version         int       4
0x28    GPIO_x1            int       4
0x2C    GPIO_x2            int       4
0x30    adc_x1             float     4
0x34    adc_x2             float     4
0x38    adc_x3             float     4
0x3C    adc_x4             float     4
0x40    adc_x5             float     4
0x44    adc_x6             float     4
0x48    adc_x7             float     4
0x4C    adc_x8             float     4
0x50    tempx              float     4
0x54    pressurex          float     4
0x58    humidity           float     4
0x5C    temp2              float     4
0x60    temp3              float     4
0x64    temp4              float     4
```

---

## Field Usage by Sensor Manager

### PCNT Flow Manager
- **Updates**: `flowData1`, `flowData2`, `flowData3`
- **Frequency**: Every 1000ms
- **Note**: `newData` flag set every other read (2000ms accumulation)

### One-Wire Temperature Manager
- **Updates**: `temp1`, `temp1_f`, `temp2`, `temp3`, `temp4`, `tempSensorcount`
- **Frequency**: Every 5000ms

### I2C ADC Manager
- **Updates**: `adc_sensor`, `adc_x1` through `adc_x8`
- **Frequency**: Every 1000ms

### I2C GPIO Manager
- **Updates**: `GPIO_x1`, `GPIO_x2`
- **Frequency**: Every 1000ms

### GPIO Discrete Manager
- **Updates**: `gpio_sensor`
- **Frequency**: Every 1000ms

### I2C Environmental Manager
- **Updates**: `tempx`, `humidity`, `pressurex`
- **Frequency**: Every 5000ms

### System Monitor Task
- **Updates**: `cycle_count`, `fw_version`
- **Frequency**: As needed

---

## Thread Safety

The `genericSens_` structure must be protected with a mutex for concurrent access:

```c
// Take mutex before accessing
if (xSemaphoreTake(sensor_data.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Read or write to genericSens_
    genericSens_.generic.temp1 = new_temp;
    
    // Release mutex
    xSemaphoreGive(sensor_data.mutex);
} else {
    ESP_LOGW(TAG, "Failed to acquire mutex for sensor data access");
}
```

---

## Compatibility Notes

### Platform Considerations

1. **Endianness**: Structure assumes little-endian byte order (ESP32-C6 default)
2. **Integer Size**: `int` is 32-bit (4 bytes) on ESP32-C6
3. **Float Format**: IEEE 754 single-precision (32-bit) float
4. **Structure Packing**: Union ensures no padding between fields

### Raspberry Pi Compatibility

Receiving applications on Raspberry Pi must:
- Use the same byte order (little-endian)
- Interpret `int` as 32-bit signed integer
- Interpret `float` as IEEE 754 single-precision
- Handle `temp1_f` as float bytes stored in int (for binary compatibility)

### Version Compatibility

- **Current Version**: 1.0
- **Interface Stability**: Frozen (no changes allowed without updating all receivers)
- **Future Changes**: Must maintain backward compatibility or increment version

---

## Implementation Checklist

When implementing this interface:

- [ ] Structure matches exactly (field order, types, sizes)
- [ ] Union correctly implemented (`data_payload` and `generic` struct)
- [ ] All 26 fields present and correctly typed
- [ ] FlowData bit-packing matches specification
- [ ] MQTT topics match exactly
- [ ] JSON variable names match exactly
- [ ] Mutex protection implemented for thread-safe access
- [ ] Binary transmission uses `data_payload` array
- [ ] JSON transmission uses structured fields with correct names
- [ ] Field initialization/default values set appropriately
- [ ] Receiving applications validated against this specification

---

## References

- **MQTT Binary Topic**: `mwp/data/sensor/generic/X001D`
- **MQTT JSON Topic**: `mwp/json/data/sensor/generic/X001D`
- **MQTT Client ID**: `Generic Flow Client`
- **Block ID**: `X001D`
- **Related Documents**: 
  - `SENSOR_MANAGER_ARCHITECTURE.md` - Sensor manager architecture
  - `PCNT_FLOW_MANAGER_DESIGN.md` - Flow sensor implementation
  - `ONEWIRE_TEMP_MANAGER_DESIGN.md` - Temperature sensor implementation

---

## Change Log

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-01-11 | Initial specification - frozen interface | - |

---

**⚠️ CRITICAL**: This interface specification must remain unchanged to maintain compatibility with existing Raspberry Pi receiving applications. Any modifications require coordination with all receiving application developers.

