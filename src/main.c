#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <sensor_pipeline.h>

static const char *TAG = "app";

void app_main(void) {
	esp_err_t err = sensor_pipeline_start();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Pipeline start failed: %s", esp_err_to_name(err));
		return;
	}

	ESP_LOGI(TAG, "Application startup complete");
	vTaskDelete(NULL);
}