#include <sensor_pipeline.h>

#include <bmp390.h>

#include <ble_beacon.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <sample_buffer.h>
#include <sensor_reading.h>

static const char *TAG = "sensor_pipeline";

static QueueHandle_t temperature_queue = NULL;
static QueueHandle_t pressure_queue = NULL;
static QueueSetHandle_t sensor_queue_set = NULL;
static bmp390_handle_t sensor_handle = NULL;

static void sensor_read_task(void *pvParameters) {
	(void)pvParameters;

	while (true) {
		float temperature_c = 0.0f;
		float pressure_pa = 0.0f;

		esp_err_t err = bmp390_get_measurements(sensor_handle, &temperature_c, &pressure_pa);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "BMP390 read failed: %s", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG, "T=%.2f°C P=%.2fPa", temperature_c, pressure_pa);

			if (xQueueOverwrite(temperature_queue, &temperature_c) != pdPASS) {
				ESP_LOGW(TAG, "Failed to enqueue temperature reading");
			}

			if (xQueueOverwrite(pressure_queue, &pressure_pa) != pdPASS) {
				ESP_LOGW(TAG, "Failed to enqueue pressure reading");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

static void beacon_update_task(void *pvParameters) {
	(void)pvParameters;

	SemaphoreHandle_t data_sem = sample_buffer_data_semaphore();

	while (true) {
		if (xSemaphoreTake(data_sem, portMAX_DELAY) == pdTRUE) {
			sensor_reading_t reading;
			if (sample_buffer_get_latest_by_type(READING_TEMPERATURE, &reading) == ESP_OK) {
				ble_beacon_set_temperature(reading.value);
			}
		}
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
			sample_buffer_push(&reading);
		}
	}
}

esp_err_t sensor_pipeline_start(void) {
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
		return err;
	}

	bmp390_config_t sensor_config = I2C_BMP390_CONFIG_DEFAULT;
	err = bmp390_init(bus_handle, &sensor_config, &sensor_handle);
	if (err != ESP_OK || sensor_handle == NULL) {
		ESP_LOGE(TAG, "BMP390 init failed: %s", esp_err_to_name(err));
		return err != ESP_OK ? err : ESP_FAIL;
	}

	temperature_queue = xQueueCreate(1, sizeof(float));
	if (temperature_queue == NULL) {
		ESP_LOGE(TAG, "Failed to create temperature queue");
		return ESP_ERR_NO_MEM;
	}

	pressure_queue = xQueueCreate(1, sizeof(float));
	if (pressure_queue == NULL) {
		ESP_LOGE(TAG, "Failed to create pressure queue");
		return ESP_ERR_NO_MEM;
	}

	sensor_queue_set = xQueueCreateSet(2);
	if (sensor_queue_set == NULL) {
		ESP_LOGE(TAG, "Failed to create sensor queue set");
		return ESP_ERR_NO_MEM;
	}

	if (xQueueAddToSet(temperature_queue, sensor_queue_set) != pdPASS ||
		xQueueAddToSet(pressure_queue, sensor_queue_set) != pdPASS) {
		ESP_LOGE(TAG, "Failed to add queues to queue set");
		return ESP_FAIL;
	}

	err = sample_buffer_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize sample buffer: %s", esp_err_to_name(err));
		return err;
	}

	BaseType_t task_ok = xTaskCreate(sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create sensor read task");
		return ESP_FAIL;
	}

	task_ok = xTaskCreate(queue_to_ring_buffer_task, "queue_to_ring_buffer_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create queue to ring buffer task");
		return ESP_FAIL;
	}

	task_ok = xTaskCreate(beacon_update_task, "beacon_update_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create beacon update task");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Sensor pipeline started");
	return ESP_OK;
}
