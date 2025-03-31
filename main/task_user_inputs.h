#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

typedef enum {
    NO_EVENT = 0,
    BTN_PRESS,
    BTN_LONGPRESS, // > 2s press
} btn_state_t;

extern QueueHandle_t task_user_input_queue;

void delegate_task_user_input(bool enable);
void task_user_inputs(void *pvParameter);