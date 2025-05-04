#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

typedef enum{
    STATE_TAKEOFF_DETECTION = 0,
    STATE_VARIO,
    STATE_VARIO_WITH_UDP_LOGGING,
    STATE_OTA,
    STATE_WIPE_NVRAM,
    STATE_MAX
} device_state_t;

extern device_state_t device_state;

void init_state_machine();
esp_err_t switch_state(device_state_t new_state);