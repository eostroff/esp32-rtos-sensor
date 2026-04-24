#include <sample_buffer.h>

#include <esp_log.h>
#include <stdint.h>

static const char *TAG = "sample_buffer";

static sensor_reading_t reading_ring_buffer[READING_RING_BUFFER_SIZE];
static uint32_t ring_write_index = 0;
static uint32_t ring_count = 0;

static SemaphoreHandle_t ring_buffer_mutex = NULL;
static SemaphoreHandle_t readings_available_sem = NULL;

esp_err_t sample_buffer_init(void) {
	ring_buffer_mutex = xSemaphoreCreateMutex();
	if (ring_buffer_mutex == NULL) {
		return ESP_ERR_NO_MEM;
	}

	readings_available_sem = xSemaphoreCreateCounting(READING_RING_BUFFER_SIZE, 0);
	if (readings_available_sem == NULL) {
		return ESP_ERR_NO_MEM;
	}

	ring_write_index = 0;
	ring_count = 0;
	return ESP_OK;
}

void sample_buffer_push(const sensor_reading_t *reading) {
	if (reading == NULL || ring_buffer_mutex == NULL || readings_available_sem == NULL) {
		return;
	}

	bool new_slot = false;
	if (xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY) == pdTRUE) {
		reading_ring_buffer[ring_write_index] = *reading;
		ring_write_index = (ring_write_index + 1U) % READING_RING_BUFFER_SIZE;
		if (ring_count < READING_RING_BUFFER_SIZE) {
			ring_count++;
			new_slot = true;
		}
		xSemaphoreGive(ring_buffer_mutex);
	}

	if (new_slot) {
		if (xSemaphoreGive(readings_available_sem) != pdTRUE) {
			ESP_LOGW(TAG, "Failed to increment readings semaphore");
		}
	}
}

size_t sample_buffer_count(void) {
	size_t count = 0;

	if (ring_buffer_mutex == NULL) {
		return 0;
	}

	if (xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY) == pdTRUE) {
		count = ring_count;
		xSemaphoreGive(ring_buffer_mutex);
	}

	return count;
}

SemaphoreHandle_t sample_buffer_data_semaphore(void) {
	return readings_available_sem;
}
