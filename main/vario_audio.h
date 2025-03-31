#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

void vaudio_config();
void vaudio_tick_handler(int32_t cps);