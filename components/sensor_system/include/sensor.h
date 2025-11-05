/*
 * sensor.h - Shared Sensor Data Structure
 * 
 * This header defines the sensor data structure used for communication
 * between ESP32-C6 sensor nodes and Raspberry Pi receiving applications.
 * 
 * This file is shared between:
 * - ESP32-C6 sensor application (SensorX)
 * - Raspberry Pi receiving applications
 * 
 * Block ID: X001D
 * Block Name: genericSens
 * 
 * ⚠️ CRITICAL: This interface specification is FROZEN and must not be
 * modified without updating all receiving applications. Any changes to
 * field order, types, or sizes will break compatibility.
 * 
 * Version: 1.0
 * Last Updated: 2025-01-11
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>  // For memcpy in utility macros

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration Constants
// ============================================================================

#define GENERICSENS_LEN 26              // Number of words in structure
#define GENERICSENS_SIZE_BYTES 104      // Total size: 26 words × 4 bytes
#define GENERICSENS_VERSION 1           // Protocol version

// Block Information
#define GENERICSENS_BLOCK_ID "X001D"
#define GENERICSENS_BLOCK_NAME "genericSens"

// MQTT Configuration
#define GENERICSENS_CLIENTID "Generic Flow Client"
#define GENERICSENS_TOPICID "mwp/data/sensor/generic/X001D"
#define GENERICSENS_JSONID "mwp/json/data/sensor/generic/X001D"

// ============================================================================
// FlowData Bit-Packed Structure
// ============================================================================

/**
 * FlowData Structure
 * 
 * Bit-packed structure containing flow sensor information.
 * Stored as a 32-bit integer (int) in the genericSens_ union.
 * 
 * Bit layout:
 * - Bits 0-11:   pulses (0-4095)
 * - Bits 12-30:  milliseconds (0-524287, max ~8.7 minutes)
 * - Bit 31:      newData flag (0 or 1)
 */
typedef struct {
    uint32_t pulses : 12;       // 12 bits for pulse count (0-4095)
    uint32_t milliseconds : 19; // 19 bits for milliseconds (0-524287)
    uint32_t newData : 1;       // 1 bit for new data flag
} FlowData_t;

// ============================================================================
// GenericSensor Data Structure
// ============================================================================

/**
 * GENERICSENS_ Union Structure
 * 
 * This union allows both structured field access and raw binary array
 * access for MQTT transmission. The structure must match exactly on both
 * ESP32-C6 and Raspberry Pi platforms.
 * 
 * Memory Layout:
 * - Total size: 104 bytes (26 words × 4 bytes per word)
 * - Integer fields: words 0-11 (48 bytes)
 * - Float fields: words 12-25 (56 bytes)
 * 
 * Structure Packing:
 * - Uses #pragma pack(1) to ensure no padding between fields
 * - Assumes little-endian byte order
 * - int is 32-bit (4 bytes)
 * - float is IEEE 754 single-precision (32-bit)
 */
#pragma pack(push, 1)

union GENERICSENS_ {
    int32_t data_payload[GENERICSENS_LEN];  // Raw array for binary MQTT transmission
    
    struct {
        // Flow sensors (PCNT) - words 0-2
        int32_t flowData1;         // Word 0: Flow sensor 1 (bit-packed FlowData_t)
        int32_t flowData2;         // Word 1: Flow sensor 2 (bit-packed FlowData_t)
        int32_t flowData3;         // Word 2: Flow sensor 3 (bit-packed FlowData_t)
        
        // ADC and GPIO sensors - words 3-4
        int32_t adc_sensor;        // Word 3: ADC Raw Sensor value (bits 0-9: 0-1023)
        int32_t gpio_sensor;       // Word 4: GPIO Sensor Data (bits 1: GPIO 4, bit 2: GPIO 5)
        
        // Temperature sensors - words 5-7
        int32_t temp1;             // Word 5: Temp f (int) - Integer temperature in Fahrenheit
        int32_t temp1_f;           // Word 6: Temperature in F Float Bytes 1&2 (float as int)
        int32_t tempSensorcount;   // Word 7: Number of Temp Sensors Detected (0-4)
        
        // System data - words 8-9
        int32_t cycle_count;       // Word 8: Cycle Counter (0-28800)
        int32_t fw_version;        // Word 9: FW Version (4 Hex digits)
        
        // Extended GPIO sensors - words 10-11
        int32_t GPIO_x1;           // Word 10: Extended sensor GPIO - 1 (MCP23X17 Port A)
        int32_t GPIO_x2;           // Word 11: Extended sensor GPIO - 2 (MCP23X17 Port B)
        
        // Extended ADC sensors - words 12-19
        float adc_x1;              // Word 12: Extended sensor ADC - 1 (Pump amperage)
        float adc_x2;              // Word 13: Extended sensor ADC - 2 (Pump amperage)
        float adc_x3;              // Word 14: Extended sensor ADC - 3 (Pump amperage)
        float adc_x4;              // Word 15: Extended sensor ADC - 4 (Pump amperage)
        float adc_x5;              // Word 16: Extended sensor ADC - 5 (Tank Hydrostatic Pressure)
        float adc_x6;              // Word 17: Extended sensor ADC - 6 (Water Pressure)
        float adc_x7;              // Word 18: Extended sensor ADC - 7 (Water Pressure)
        float adc_x8;              // Word 19: Extended sensor ADC - 8 (Water Pressure)
        
        // Environmental sensors - words 20-22
        float tempx;               // Word 20: System Temp (BME280 temperature in F)
        float pressurex;            // Word 21: Atmospheric Pressure (BME280 pressure in PSI)
        float humidity;             // Word 22: Humidity (%) (BME280 humidity %)
        
        // Additional temperature sensors - words 23-25
        float temp2;               // Word 23: Temp sensor 2 (One-Wire DS18B20 sensor 2)
        float temp3;               // Word 24: Temp sensor 3 (One-Wire DS18B20 sensor 3)
        float temp4;               // Word 25: Temp sensor 4 (One-Wire DS18B20 sensor 4)
    } generic;
};

#pragma pack(pop)

// ============================================================================
// JSON Variable Names
// ============================================================================

/**
 * JSON Variable Names Array
 * 
 * Array of JSON field names for MQTT JSON transmission.
 * Index corresponds to word number in the structure.
 * 
 * Implementation:
 * - Declared here as extern
 * - Defined in sensor.c (ESP32 project)
 * - Should be defined in corresponding .c file for Raspberry Pi projects
 */
extern const char* genericsens_ClientData_var_name[GENERICSENS_LEN];

// ============================================================================
// Utility Functions/Macros
// ============================================================================

/**
 * Extract FlowData from int32_t
 * 
 * Usage:
 *   FlowData_t flow1;
 *   GENERICSENS_EXTRACT_FLOWDATA(genericSens_.generic.flowData1, flow1);
 */
#define GENERICSENS_EXTRACT_FLOWDATA(src_int, dest_flowdata) \
    do { \
        memcpy(&(dest_flowdata), &(src_int), sizeof(int32_t)); \
    } while(0)

/**
 * Pack FlowData into int32_t
 * 
 * Usage:
 *   FlowData_t flow1 = {.pulses = 100, .milliseconds = 2000, .newData = 1};
 *   GENERICSENS_PACK_FLOWDATA(flow1, genericSens_.generic.flowData1);
 */
#define GENERICSENS_PACK_FLOWDATA(src_flowdata, dest_int) \
    do { \
        memcpy(&(dest_int), &(src_flowdata), sizeof(int32_t)); \
    } while(0)

/**
 * Structure size validation
 * 
 * Use this to verify structure size matches expected value.
 * Example: static_assert(sizeof(union GENERICSENS_) == GENERICSENS_SIZE_BYTES, "Structure size mismatch");
 */

// ============================================================================
// Platform Compatibility Notes
// ============================================================================

/*
 * Platform Requirements:
 * 
 * 1. Endianness: Structure assumes little-endian byte order
 *    - ESP32-C6: Little-endian (default)
 *    - Raspberry Pi: Little-endian (default)
 * 
 * 2. Integer Size: int32_t must be exactly 32-bit (4 bytes)
 *    - ESP32-C6: int32_t is 32-bit
 *    - Raspberry Pi: int32_t is 32-bit
 * 
 * 3. Float Format: float must be IEEE 754 single-precision (32-bit)
 *    - ESP32-C6: IEEE 754 single-precision
 *    - Raspberry Pi: IEEE 754 single-precision
 * 
 * 4. Structure Packing: #pragma pack(1) ensures no padding
 *    - Must be consistent across platforms
 * 
 * 5. Alignment: Structure starts at byte boundary
 *    - No alignment requirements beyond standard types
 * 
 * Compatibility Validation:
 * - Verify sizeof(union GENERICSENS_) == 104 bytes
 * - Verify sizeof(int32_t) == 4 bytes
 * - Verify sizeof(float) == 4 bytes
 * - Verify byte order (little-endian)
 */

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */

