#ifndef SENSOR_READING_H
#define SENSOR_READING_H

#include <freertos/FreeRTOS.h>

typedef enum {
	READING_TEMPERATURE = 0,
	READING_PRESSURE,
} reading_type_t;

typedef struct {
	reading_type_t type;
	float value;
	TickType_t timestamp_ticks;
} sensor_reading_t;

#endif
