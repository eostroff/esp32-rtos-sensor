#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <ble_beacon.h>
#include <sensor_pipeline.h>

static const char *TAG = "app";

void app_main(void) {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
		return;
	}

	err = ble_beacon_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "BLE beacon init failed: %s", esp_err_to_name(err));
		return;
	}

	err = sensor_pipeline_start();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Pipeline start failed: %s", esp_err_to_name(err));
		return;
	}

	ESP_LOGI(TAG, "Application startup complete");
	vTaskDelete(NULL);
}