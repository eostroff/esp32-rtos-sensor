#ifndef BLE_BEACON_H
#define BLE_BEACON_H

#include <esp_err.h>

esp_err_t ble_beacon_init(void);
void ble_beacon_set_temperature(float temperature_c);

#endif
