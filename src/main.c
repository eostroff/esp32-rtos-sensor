#include <bmp390.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "app";

void app_main(void) {
	i2c_master_bus_handle_t bus_handle = NULL;
	bmp390_handle_t sensor_handle = NULL;

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

	while (true) {
		float temperature_c = 0.0f;
		float pressure_pa = 0.0f;

		err = bmp390_get_measurements(sensor_handle, &temperature_c, &pressure_pa);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "BMP390 read failed: %s", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f hPa", temperature_c, pressure_pa / 100.0f);
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}