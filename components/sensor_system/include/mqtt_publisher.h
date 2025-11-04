/*
 * mqtt_publisher.h - MQTT Publisher Task Header
 *
 * This file contains task declarations for the MQTT publisher task.
 */

#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t xMqttPublisherTaskHandle;

// Function declaration
void vMqttPublisherTask(void *pvParameters);

#endif /* MQTT_PUBLISHER_H */

