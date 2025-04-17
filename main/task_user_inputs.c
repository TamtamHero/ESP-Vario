#include "task_user_inputs.h"
#include "task_buzzer.h"
#include "task_ota.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "net_logging.h"

#include "driver/gpio.h"

#include "nvd.h"
#include "config.h"
#include "adc.h"
#include "leds.h"
#include "wifi.h"

#define TAG "inputs"

#define BTN_VOLTAGE_HIGH 3.0
#define BTN_LONGPRESS_TIME 1000000
#define BTN_DOWN (voltage < BTN_VOLTAGE_HIGH)

typedef enum{
    STATE_VARIO = 0,
    STATE_VARIO_WITH_UDP_LOGGING,
    STATE_OTA,
    STATE_WIPE_NVRAM,
    STATE_MAX
} ui_state_t;

int64_t btn_pushdown_time = 0;
ui_state_t ui_state = STATE_VARIO;
bool delegate;

QueueHandle_t task_user_input_queue;

void delegate_task_user_input(bool enable){
    delegate = enable;
}

void task_user_inputs(void *pvParameter)
{
    TaskHandle_t *tasks = (TaskHandle_t *)pvParameter;
    ESP_LOGI(TAG, "Task begins...");
    delegate = false;
    adc_init(SENSE_VBAT_CHAN);

    task_user_input_queue = xQueueCreate(1, sizeof(btn_state_t));
    if (task_user_input_queue == NULL) {
        ESP_LOGE(TAG, "Could not create input queue");
        vTaskDelete(NULL);
    }

    while(1){
        float voltage = adc_read_battery_voltage(SENSE_VBAT_CHAN);
        if(btn_pushdown_time > 0){
            if(BTN_DOWN){
                // ongoing button press
                if(esp_timer_get_time() - btn_pushdown_time > BTN_LONGPRESS_TIME){
                    // case of long button press, might wanna trigger beeps until device is off ?
                    ESP_LOGI(TAG, "long btn press");
                }
            }
            else{
                // case of short button press
                ESP_LOGI(TAG, "short btn press");
                btn_pushdown_time = 0;
                // if delegation is enabled, this button press is going to be signaled through the task_user_input_queue for other tasks to know
                if(delegate){
                    delegate = false;
                    btn_state_t msg = BTN_PRESS;
                    xQueueSend(task_user_input_queue, &msg, 0);
                }
                else{
                    ui_state = (ui_state + 1) % STATE_MAX;
                    ESP_LOGI(TAG, "new state: %d", ui_state);
                    switch (ui_state)
                    {
                    case STATE_VARIO:
                        set_led_rgb(5, 0, 0);
                        tone(200, 100);
                        vTaskDelete(tasks[TASK_OTA]);
                        set_wifi(false);
                        vTaskResume(tasks[TASK_VARIO]);
                        break;

                    case STATE_VARIO_WITH_UDP_LOGGING:
                        set_led_rgb(0, 5, 0);
                        tone(200, 100);
                        set_wifi(true);
                        ESP_ERROR_CHECK_WITHOUT_ABORT(udp_logging_init("255.255.255.255", 6789, 1));
                        break;

                    case STATE_OTA:
                        set_led_rgb(0, 0, 5);
                        tone(200, 100);
                        vTaskSuspend(tasks[TASK_VARIO]);
                        esp_log_set_vprintf(vprintf);
                        if(xTaskCreate(&task_ota, "task_ota", 8192, NULL, 0, &tasks[TASK_OTA]) != pdPASS){
                            ESP_LOGI(TAG, "Failed starting ota task, resetting...");
                            abort();
                        }
                        break;

                    case STATE_WIPE_NVRAM:
                        set_led_rgb(5, 0, 5);
                        tone(200, 100);
                        nvd_set_defaults();
                        ESP_ERROR_CHECK(nvd_commit());
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        abort();
                        break;

                    default:
                        break;
                    }
                }
            }
        }
        else{
            if(BTN_DOWN){
                // new button press detected
                btn_pushdown_time = esp_timer_get_time();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}