#pragma once
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

esp_err_t ble_server_init();
void ble_transmit_LK8EX1(float pressure_pa, int32_t cps, float batVoltage);
