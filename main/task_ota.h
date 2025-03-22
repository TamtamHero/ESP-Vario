#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"

#include "freertos/FreeRTOS.h"

void task_ota(void *pvParameter);
