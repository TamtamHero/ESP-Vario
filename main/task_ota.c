#include "task_ota.h"
#include "wifi.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/event_groups.h"

#define TAG "ota"

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

esp_err_t _http_ota_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

void task_ota(void *pvParameter){
    // TODO: we mark current partition as valid if we are able to reach STATE_OTA but we could probably do cleaner
    esp_ota_mark_app_valid_cancel_rollback();
    while(1){
        if(!is_wifi_enabled()){
            ESP_ERROR_CHECK(set_wifi(true));
        }

        struct ifreq ifr;
        esp_netif_get_netif_impl_name(wifi_netif, ifr.ifr_name);
        ESP_LOGI(TAG, "Bind interface name is %s", ifr.ifr_name);
        esp_http_client_config_t config = {
            .url = CONFIG_FIRMWARE_OTA_UPGRADE_URL,
            .cert_pem = (char *)server_cert_pem_start,
            .event_handler = _http_ota_event_handler,
            .keep_alive_enable = true,
            .buffer_size = 2048
        };

        esp_https_ota_config_t ota_config = {
            .http_config = &config,
            .bulk_flash_erase = true
        };

        while (is_wifi_enabled()) {
            ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
            esp_err_t ret = esp_https_ota(&ota_config);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
                esp_restart();
            } else {
                ESP_LOGE(TAG, "Firmware upgrade failed");
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}