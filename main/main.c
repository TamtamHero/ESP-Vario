#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "string.h"

#include "driver/gpio.h"

#include "config.h"
#include "nvd.h"
#include "task_ota.h"
#include "task_user_inputs.h"
#include "task_buzzer.h"
#include "task_vario.h"

#define TAG "main"

TaskHandle_t tasks[5];

void app_main(void)
{
    ESP_LOGI(TAG, "app_main start");

    // switch on red LED
    gpio_reset_pin(pinLED_R);
    gpio_reset_pin(pinLED_G);
    gpio_reset_pin(pinLED_B);
    gpio_set_direction(pinLED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinLED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinLED_B, GPIO_MODE_OUTPUT);
    gpio_set_level(pinLED_R, LED_ON);
    gpio_set_level(pinLED_G, LED_OFF);
    gpio_set_level(pinLED_B, LED_OFF);

    vTaskDelay(1000/portTICK_PERIOD_MS);

    // Initialize NVS.
    nvd_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xTaskCreate(&task_ota, "task_ota", 8192, NULL, 5, &tasks[TASK_OTA]);
    xTaskCreate(&task_buzzer, "task_buzzer", 8192, tasks, 6, &tasks[TASK_BUZZER]);
    xTaskCreate(&task_user_inputs, "task_user_inputs", 8192, tasks, 6, &tasks[TASK_USER_INPUT]);
    xTaskCreate(&task_vario, "task_vario", 32768, tasks, 4, &tasks[TASK_VARIO]);
}
