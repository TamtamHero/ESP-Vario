#include "config.h"
#include "adc.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "ADC"

#define NUM_AVG_SAMPLES 4
// get this value in from bootloader with command "espefuse.py --port /dev/ttyUSB0 adc_info"
#define REF_VOLTAGE 1135

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_chan_handle;

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_bitwidth_t bitwidth, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = bitwidth,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

// static void adc_calibration_deinit(adc_cali_handle_t handle)
// {
//     ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
//     ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
// }

void adc_init(adc_channel_t channel){

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_2_5,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));

    //-------------ADC1 Calibration Init---------------//
    adc1_cali_chan_handle = NULL;
    adc_calibration_init(ADC_UNIT_1, channel, ADC_ATTEN_DB_2_5, ADC_BITWIDTH_12, &adc1_cali_chan_handle);

}


float adc_read_battery_voltage(adc_channel_t channel) {
    uint32_t adcSample = 0;
    for (int inx = 0; inx < NUM_AVG_SAMPLES; inx++) {
        int adc_raw, voltage;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));
        ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, channel, adc_raw);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan_handle, adc_raw, &voltage));
        ESP_LOGD(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, channel, voltage);
        adcSample += voltage;
      }
    adcSample /= NUM_AVG_SAMPLES;
    // sample is the voltage drop across R1, where R1=100k and R2=340k, hence multiplying by 4.4 should give the total voltage 
    // (but we do *4.5, don't remember why maybe the resistors are not very accurate)
    return 4.5*adcSample/1000.0;
}