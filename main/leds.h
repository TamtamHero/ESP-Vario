#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

typedef enum{
    LED_R,
    LED_G,
    LED_B
} led_id_t;

void leds_init();
void set_led(led_id_t led_id, uint8_t intensity);
void set_led_rgb(uint8_t intensity_r, uint8_t intensity_g, uint8_t intensity_b);