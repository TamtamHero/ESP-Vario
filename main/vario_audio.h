#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "stdint.h"

typedef struct {
    int16_t cps; // climb rate, cm.s⁻¹
    uint16_t frequency; // beeper frequency, Hz
    uint16_t length; // duration in ms
    uint8_t duty; // 0-100% value
} sound_profile_point_t;

#define NUM_SOUND_PROFILE_POINTS 9

void vaudio_init();
void vaudio_tick_handler(int32_t cps);