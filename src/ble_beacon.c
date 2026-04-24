#include <ble_beacon.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <stdint.h>

static const char *TAG = "ble_beacon";

// iBeacon proximity UUID (big-endian).
// Replace with your own from https://www.uuidgenerator.net
// Current: E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
#define BEACON_NAME "TempBeacon"

// Advertising interval: 100ms in 0.625ms units
#define ADV_ITVL_UNITS BLE_GAP_ADV_ITVL_MS(100)

// Major encodes temperature * 10 as a signed 16-bit integer.
// Example: 23.5°C → Major 235. Divide by 10 in Beacon Scope to read °C.
#define MAJOR_HI_OFFSET 25
#define MAJOR_LO_OFFSET 26
#define MINOR_HI_OFFSET 27
#define MINOR_LO_OFFSET 28

static volatile float s_temperature_c = 0.0f;

// iBeacon advertising payload (30 bytes):
//   [0-2]   Flags AD
//   [3-29]  Manufacturer Specific Data (Apple iBeacon, 27 bytes)
//             Company ID:    0x004C (Apple Inc.)
//             Subtype:       0x02 0x15 (iBeacon, 21-byte payload)
//             UUID[0-15]:    proximity UUID, big-endian
//             Major[0-1]:    temperature * 10, big-endian signed
//             Minor[0-1]:    0x0000 (reserved / sensor ID)
//             TX Power:      0xC5 (-59 dBm, calibrate for your hardware)
static uint8_t s_adv_data[] = {
    0x02, 0x01, 0x06,              // Flags: LE General Discoverable, no BR/EDR
    0x1A, 0xFF,                    // Manufacturer Specific Data, length=26
    0x4C, 0x00,                    // Apple Inc. company ID (little-endian)
    0x02, 0x15,                    // iBeacon subtype + payload length=21
    // Proximity UUID (16 bytes, big-endian):
    0xE2, 0xC5, 0x6D, 0xB5,        // E2C56DB5-
    0xDF, 0xFB,                    //           DFFB-
    0x48, 0xD2,                    //                48D2-
    0xB0, 0x60,                    //                      B060-
    0xD0, 0xF5, 0xA7, 0x10,        //                            D0F5A710-
    0x96, 0xE0,                    //                                      96E0
    0x00, 0x00,                    // Major (temperature * 10, big-endian)
    0x00, 0x00,                    // Minor (0x0000)
    0xC5,                          // TX Power: -59 dBm
};

// Scan response carries the device name so Beacon Scope can display it.
// Requires scannable advertising (disc_mode = GEN → ADV_SCAN_IND).
static const uint8_t s_scan_rsp[] = {
    sizeof(BEACON_NAME), 0x09,     // Length, Type = Complete Local Name
    'T', 'e', 'm', 'p', 'B', 'e', 'a', 'c', 'o', 'n',
};

static void refresh_adv_payload(void) {
    float temp = s_temperature_c;
    int16_t major = (int16_t)(temp * 10.0f);
    s_adv_data[MAJOR_HI_OFFSET] = (uint8_t)(major >> 8);
    s_adv_data[MAJOR_LO_OFFSET] = (uint8_t)(major & 0xFF);
    // Minor is fixed at 0x0000 (no update needed)
}

static void start_advertising(void) {
    refresh_adv_payload();

    int rc = ble_gap_adv_set_data(s_adv_data, (int)sizeof(s_adv_data));
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_data failed: %d", rc);
        return;
    }

    rc = ble_gap_adv_rsp_set_data(s_scan_rsp, (int)sizeof(s_scan_rsp));
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_data failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_NON,
        // GEN = scannable non-connectable (ADV_SCAN_IND), required for scan
        // response packets that carry the device name.
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min  = ADV_ITVL_UNITS,
        .itvl_max  = ADV_ITVL_UNITS,
    };

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed: %d", rc);
    }
}

static void ble_on_sync(void) {
    start_advertising();
    ESP_LOGI(TAG, "iBeacon advertising started (UUID E2C56DB5-DFFB-48D2-B060-D0F5A71096E0)");
}

static void ble_on_reset(int reason) {
    ESP_LOGW(TAG, "BLE host reset (reason %d); will re-sync", reason);
}

static void nimble_host_task(void *pvParameters) {
    (void)pvParameters;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_beacon_init(void) {
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    nimble_port_freertos_init(nimble_host_task);
    return ESP_OK;
}

void ble_beacon_set_temperature(float temperature_c) {
    s_temperature_c = temperature_c;
    ble_gap_adv_stop();
    start_advertising();
    ESP_LOGI(TAG, "Beacon temperature updated: %.2f°C", temperature_c);
}
