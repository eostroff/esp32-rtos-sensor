#include <bmp390.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "app";
static QueueHandle_t temperature_queue = NULL;
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
			if (xQueueOverwrite(temperature_queue, &temperature_c) != pdPASS) {
				ESP_LOGW(TAG, "Failed to enqueue temperature reading");
			} else {
				ESP_LOGI(TAG, "Enqueued temperature: %.2f C", temperature_c);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
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

	BaseType_t task_ok = xTaskCreate(sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create sensor read task");
		return;
	}

	ESP_LOGI(TAG, "Sensor read task started");
	vTaskDelete(NULL);
}