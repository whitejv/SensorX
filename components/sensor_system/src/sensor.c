/*
 * sensor.c - Sensor Data Structure Implementation
 * 
 * This file provides the implementation for the shared sensor data structure.
 * Specifically, it defines the JSON variable names array used for MQTT JSON transmission.
 * 
 * This file should be compiled into both ESP32-C6 and Raspberry Pi applications.
 */

#include "sensor.h"

// JSON Variable Names Array
// Index corresponds to word number in the GENERICSENS_ structure
const char* genericsens_ClientData_var_name[] = {
    "X001D:flowData1",        // Word 0
    "X001D:flowData2",        // Word 1
    "X001D:flowData3",        // Word 2
    "X001D:adc_sensor",       // Word 3
    "X001D:gpio_sensor",      // Word 4
    "X001D:temp1",            // Word 5
    "X001D:temp1_f",          // Word 6
    "X001D:tempSensorcount",  // Word 7
    "X001D:cycle_count",       // Word 8
    "X001D:fw_version",        // Word 9
    "X001D:GPIO_x1",          // Word 10
    "X001D:GPIO_x2",          // Word 11
    "X001D:adc_x1",           // Word 12
    "X001D:adc_x2",           // Word 13
    "X001D:adc_x3",           // Word 14
    "X001D:adc_x4",           // Word 15
    "X001D:adc_x5",           // Word 16
    "X001D:adc_x6",           // Word 17
    "X001D:adc_x7",           // Word 18
    "X001D:adc_x8",           // Word 19
    "X001D:tempx",            // Word 20
    "X001D:pressurex",        // Word 21
    "X001D:humidity",         // Word 22
    "X001D:temp2",            // Word 23
    "X001D:temp3",            // Word 24
    "X001D:temp4"             // Word 25
};

