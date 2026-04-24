#ifndef SAMPLE_BUFFER_H
#define SAMPLE_BUFFER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stddef.h>

#include <sensor_reading.h>

#define READING_RING_BUFFER_SIZE 32

esp_err_t sample_buffer_init(void);
void sample_buffer_push(const sensor_reading_t *reading);
size_t sample_buffer_count(void);
SemaphoreHandle_t sample_buffer_data_semaphore(void);

#endif
