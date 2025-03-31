#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"


void task_buzzer(void *pvParameter);

void tone(unsigned int frequency, unsigned long duration);

void silence(unsigned long duration);

void noTone();