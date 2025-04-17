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
#include "leds.h"

#define TAG "main"

TaskHandle_t tasks[5];

void app_main(void)
{
    ESP_LOGI(TAG, "app_main start");

    // switch on red LED
    leds_init();
    set_led(LED_R, 5);
    vTaskDelay(100/portTICK_PERIOD_MS);

    // Attempt continuation of deep sleep if device just woke up from it
    deep_sleep(false);

    // Initialize NVS.
    nvd_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xTaskCreate(&task_buzzer, "task_buzzer", 8192, tasks, 6, &tasks[TASK_BUZZER]);
    xTaskCreate(&task_user_inputs, "task_user_inputs", 8192, tasks, 6, &tasks[TASK_USER_INPUT]);
    xTaskCreate(&task_vario, "task_vario", 32768, tasks, 6, &tasks[TASK_VARIO]);
}
