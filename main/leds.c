#include "leds.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "config.h"

#define TAG "leds"

#define MAX_DUTY 8192 // 2**13
#define TO_DUTY(level) (MAX_DUTY - (MAX_DUTY * level / 100))// Convert an intensity level (0-100) to a duty cycle

void leds_init(){
    gpio_reset_pin(pinLED_R);
    gpio_reset_pin(pinLED_G);
    gpio_reset_pin(pinLED_B);
    gpio_set_direction(pinLED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinLED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinLED_B, GPIO_MODE_OUTPUT);
    gpio_set_level(pinLED_R, LED_OFF);
    gpio_set_level(pinLED_G, LED_OFF);
    gpio_set_level(pinLED_B, LED_OFF);

    ledc_timer_config_t ledc_timer_R = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = 3000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config_t ledc_timer_G = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_2,
        .freq_hz          = 3000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config_t ledc_timer_B = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_3,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_R));
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_G));
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_B));

    ledc_channel_config_t ledc_channel_R = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pinLED_R,
        .duty           = TO_DUTY(0),
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_G = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER_2,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pinLED_G,
        .duty           = TO_DUTY(0),
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel_B = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_3,
        .timer_sel      = LEDC_TIMER_3,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pinLED_B,
        .duty           = TO_DUTY(0),
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_R));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_G));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_B));
}

void set_led(led_id_t led_id, uint8_t intensity){
    ledc_timer_t timer = led_id == LED_R ? LEDC_TIMER_1 : led_id == LED_G ? LEDC_TIMER_2 : LEDC_TIMER_3;
    if(intensity == 0){
        ledc_stop(LEDC_LOW_SPEED_MODE, timer, LED_OFF);
    }
    else{
        ledc_set_duty(LEDC_LOW_SPEED_MODE, timer, TO_DUTY(intensity));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, timer);
    }
}

void set_led_rgb(uint8_t intensity_r, uint8_t intensity_g, uint8_t intensity_b){
    set_led(LED_R, intensity_r);
    set_led(LED_G, intensity_g);
    set_led(LED_B, intensity_b);
}