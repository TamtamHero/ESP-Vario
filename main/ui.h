#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "nvd.h"

void ui_indicate_uncalibrated_accel_gyro();
void ui_indicate_sleep();
void ui_indicate_fault_MS5611();
void ui_indicate_fault_MPU6050();
void ui_indicate_battery_voltage();
void ui_calibrate_accel(CALIB_PARAMS_t *calib);
void ui_calibrate_gyro(CALIB_PARAMS_t *calib);
void ui_calibrate_accel_gyro();
void ui_go_to_sleep();
