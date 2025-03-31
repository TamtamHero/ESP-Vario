#include "task_buzzer.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "config.h"

#define TAG "buzzer"

#define LEDC_DUTY (4096) // Set duty to 50% for square waves (2 ** 13) * 50% = 4096
#define DEFAULT_FREQ (1000)

typedef enum {
    TONE_START,
    TONE_END
  } tone_cmd_t;

typedef struct {
    tone_cmd_t tone_cmd;
    unsigned int frequency;
    unsigned long duration;
} tone_msg_t;

static QueueHandle_t _tone_queue = NULL;

void task_buzzer(void *pvParameter)
{
    ESP_LOGI(TAG, "Task begins...");

    gpio_reset_pin(pinAudio);
    gpio_set_direction(pinAudio, GPIO_MODE_OUTPUT);

    // Create message queue
    _tone_queue = xQueueCreate(128, sizeof(tone_msg_t));
    if (_tone_queue == NULL) {
        ESP_LOGE(TAG, "Could not create tone queue");
        vTaskDelete(NULL);
    }

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = DEFAULT_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pinAudio,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    tone_msg_t tone_msg;
    while (1) {
        xQueueReceive(_tone_queue, &tone_msg, portMAX_DELAY);
        switch (tone_msg.tone_cmd) {
            case TONE_START:
                ESP_LOGD(TAG, "TONE_START: frequency=%u Hz, duration=%lu ms", tone_msg.frequency, tone_msg.duration);
                uint32_t duty = tone_msg.frequency == 0 ? 0 : LEDC_DUTY;
                uint32_t freq = tone_msg.frequency == 0 ? DEFAULT_FREQ : tone_msg.frequency;
                ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
                vTaskDelay(pdMS_TO_TICKS(tone_msg.duration));
                ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 0);
            break;

            case TONE_END:
                ESP_LOGD(TAG, "TONE_END");
                ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 0);
            break;

            default:;  // do nothing
        }  // switch
    }  // infinite loop
}

void noTone() {
    tone_msg_t tone_msg = {
        .tone_cmd = TONE_END,
        .frequency = 0,  // Ignored
        .duration = 0,   // Ignored
    };
    xQueueReset(_tone_queue);  // clear queue
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
}

// parameters:
// frequency - PWM frequency in Hz
// duration - time in ms - how long will the signal be outputted.
//   If not provided, or 0 you must manually call noTone to end output
void tone(unsigned int frequency, unsigned long duration) {
    tone_msg_t tone_msg = {
        .tone_cmd = TONE_START,
        .frequency = frequency,
        .duration = duration,
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
}

void silence(unsigned long duration){
    tone(0, duration);
}