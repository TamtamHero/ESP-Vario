#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

void deep_sleep(bool force_deep_sleep);
void task_vario(void *pvParameter);