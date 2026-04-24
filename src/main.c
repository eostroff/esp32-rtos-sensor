#include <bmp390.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

static const char *TAG = "app";
static QueueHandle_t temperature_queue = NULL;
static QueueHandle_t pressure_queue = NULL;
static QueueSetHandle_t sensor_queue_set = NULL;
static SemaphoreHandle_t ring_buffer_mutex = NULL;
static SemaphoreHandle_t readings_available_sem = NULL;
static bmp390_handle_t sensor_handle = NULL;

#define READING_RING_BUFFER_SIZE 32

typedef enum {
	READING_TEMPERATURE = 0,
	READING_PRESSURE,
} reading_type_t;

typedef struct {
	reading_type_t type;
	float value;
	TickType_t timestamp_ticks;
} sensor_reading_t;

static sensor_reading_t reading_ring_buffer[READING_RING_BUFFER_SIZE];
static uint32_t ring_write_index = 0;
static uint32_t ring_count = 0;

static void ring_buffer_push(const sensor_reading_t *reading) {
	if (xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY) == pdTRUE) {
		reading_ring_buffer[ring_write_index] = *reading;
		ring_write_index = (ring_write_index + 1U) % READING_RING_BUFFER_SIZE;
		if (ring_count < READING_RING_BUFFER_SIZE) {
			ring_count++;
		}
		xSemaphoreGive(ring_buffer_mutex);
	}
}

static void sensor_read_task(void *pvParameters) {
	(void)pvParameters;

	while (true) {
		float temperature_c = 0.0f;
		float pressure_pa = 0.0f;

		esp_err_t err = bmp390_get_measurements(sensor_handle, &temperature_c, &pressure_pa);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "BMP390 read failed: %s", esp_err_to_name(err));
		} else {
			if (xQueueOverwrite(temperature_queue, &temperature_c) != pdPASS) {
				ESP_LOGW(TAG, "Failed to enqueue temperature reading");
			}

			if (xQueueOverwrite(pressure_queue, &pressure_pa) != pdPASS) {
				ESP_LOGW(TAG, "Failed to enqueue pressure reading");
			}

			ESP_LOGI(TAG, "Enqueued temperature: %.2f C, pressure: %.2f hPa", temperature_c, pressure_pa / 100.0f);
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

static void queue_to_ring_buffer_task(void *pvParameters) {
	(void)pvParameters;

	while (true) {
		QueueSetMemberHandle_t ready_member = xQueueSelectFromSet(sensor_queue_set, portMAX_DELAY);
		sensor_reading_t reading = {
			.type = READING_TEMPERATURE,
			.value = 0.0f,
			.timestamp_ticks = xTaskGetTickCount(),
		};

		BaseType_t received = pdFALSE;
		if (ready_member == temperature_queue) {
			received = xQueueReceive(temperature_queue, &reading.value, 0);
			reading.type = READING_TEMPERATURE;
		} else if (ready_member == pressure_queue) {
			received = xQueueReceive(pressure_queue, &reading.value, 0);
			reading.type = READING_PRESSURE;
		}

		if (received == pdTRUE) {
			reading.timestamp_ticks = xTaskGetTickCount();
			ring_buffer_push(&reading);
			xSemaphoreGive(readings_available_sem);
		}
	}
}

void app_main(void) {
	i2c_master_bus_handle_t bus_handle = NULL;

	i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = GPIO_NUM_21,
		.scl_io_num = GPIO_NUM_22,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
		return;
	}

	bmp390_config_t sensor_config = I2C_BMP390_CONFIG_DEFAULT;
	err = bmp390_init(bus_handle, &sensor_config, &sensor_handle);
	if (err != ESP_OK || sensor_handle == NULL) {
		ESP_LOGE(TAG, "BMP390 init failed: %s", esp_err_to_name(err));
		return;
	}

	temperature_queue = xQueueCreate(1, sizeof(float));
	if (temperature_queue == NULL) {
		ESP_LOGE(TAG, "Failed to create temperature queue");
		return;
	}

	pressure_queue = xQueueCreate(1, sizeof(float));
	if (pressure_queue == NULL) {
		ESP_LOGE(TAG, "Failed to create pressure queue");
		return;
	}

	sensor_queue_set = xQueueCreateSet(2);
	if (sensor_queue_set == NULL) {
		ESP_LOGE(TAG, "Failed to create sensor queue set");
		return;
	}

	if (xQueueAddToSet(temperature_queue, sensor_queue_set) != pdPASS ||
		xQueueAddToSet(pressure_queue, sensor_queue_set) != pdPASS) {
		ESP_LOGE(TAG, "Failed to add queues to queue set");
		return;
	}

	ring_buffer_mutex = xSemaphoreCreateMutex();
	if (ring_buffer_mutex == NULL) {
		ESP_LOGE(TAG, "Failed to create ring buffer mutex");
		return;
	}

	readings_available_sem = xSemaphoreCreateCounting(READING_RING_BUFFER_SIZE, 0);
	if (readings_available_sem == NULL) {
		ESP_LOGE(TAG, "Failed to create readings semaphore");
		return;
	}

	BaseType_t task_ok = xTaskCreate(sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create sensor read task");
		return;
	}

	task_ok = xTaskCreate(queue_to_ring_buffer_task, "queue_to_ring_buffer_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create queue to ring buffer task");
		return;
	}

	ESP_LOGI(TAG, "Sensor read and queue-to-ring tasks started");
	vTaskDelete(NULL);
}