/*
 * sensor_acquisition.h - Sensor Acquisition Task Header
 *
 * This file contains task declarations for the sensor acquisition task.
 */

#ifndef SENSOR_ACQUISITION_H
#define SENSOR_ACQUISITION_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t xSensorAcquisitionTaskHandle;

// Function declaration
void vSensorAcquisitionTask(void *pvParameters);

#endif /* SENSOR_ACQUISITION_H */

