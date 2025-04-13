#include "task_user_inputs.h"
#include "task_buzzer.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "config.h"
#include "adc.h"

#define TAG "inputs"

#define BTN_VOLTAGE_HIGH 3.0
#define BTN_LONGPRESS_TIME 1000000
#define BTN_DOWN (voltage < BTN_VOLTAGE_HIGH)

int64_t btn_pushdown_time = 0;
uint32_t led_state = LED_OFF;
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
                // otherwise, switch between OTA/Vario mode
                else{
                    led_state = !led_state;
                    gpio_set_level(pinLED_G, led_state);
                    if(led_state == LED_ON){
                        tone(800, 100);
                        vTaskResume(tasks[TASK_OTA]);
                        vTaskSuspend(tasks[TASK_VARIO]);
                    }
                    else{
                        tone(200, 100);
                        vTaskSuspend(tasks[TASK_OTA]);
                        vTaskResume(tasks[TASK_VARIO]);
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
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}