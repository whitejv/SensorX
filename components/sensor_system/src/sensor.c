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
    "S005D:flowData1",        // Word 0
    "S005D:flowData2",        // Word 1
    "S005D:flowData3",        // Word 2
    "S005D:adc_sensor",       // Word 3
    "S005D:gpio_sensor",      // Word 4
    "S005D:temp1",            // Word 5
    "S005D:temp1_f",          // Word 6
    "S005D:tempSensorcount",  // Word 7
    "S005D:cycle_count",       // Word 8
    "S005D:fw_version",        // Word 9
    "S005D:GPIO_x1",          // Word 10
    "S005D:GPIO_x2",          // Word 11
    "S005D:adc_x1",           // Word 12
    "S005D:adc_x2",           // Word 13
    "S005D:adc_x3",           // Word 14
    "S005D:adc_x4",           // Word 15
    "S005D:adc_x5",           // Word 16
    "S005D:adc_x6",           // Word 17
    "S005D:adc_x7",           // Word 18
    "S005D:adc_x8",           // Word 19
    "S005D:tempx",            // Word 20
    "S005D:pressurex",        // Word 21
    "S005D:humidity",         // Word 22
    "S005D:temp2",            // Word 23
    "S005D:temp3",            // Word 24
    "S005D:temp4"             // Word 25
};

