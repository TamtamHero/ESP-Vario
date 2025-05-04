#include "state.h"

#define TAG "state_machine"

device_state_t device_state = STATE_TAKEOFF_DETECTION;

SemaphoreHandle_t state_mutex = NULL;
StaticSemaphore_t mutex_buffer;

void init_state_machine(){
    state_mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
    device_state = STATE_TAKEOFF_DETECTION;
}

esp_err_t switch_state(device_state_t new_state){
    if(xSemaphoreTake(state_mutex, pdMS_TO_TICKS(100)) != pdTRUE){
        return ESP_FAIL;
    }
    device_state = new_state;
    if(xSemaphoreGive(state_mutex) != pdTRUE){
        ESP_LOGE(TAG, "Error releasing mutex, impossible");
        return ESP_FAIL;
    }
    return ESP_OK;
}