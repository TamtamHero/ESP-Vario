#include "nvd.h"

#include "string.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"

#define TAG "nvd"

// Non-volatile data structure
NVD_t Nvd;

// CRC-16 configuration (using DNP polynomial)
#define CRC16_DNP      0x3D65
#define POLYNOM        CRC16_DNP

// NVS namespace and key for our data
#define NVS_NAMESPACE  "nvd_storage"
#define NVS_DATA_KEY   "nvd_data"

static uint16_t crc16(uint16_t crcValue, uint8_t newByte);
static uint16_t nvd_checksum(void);

static uint16_t crc16(uint16_t crcValue, uint8_t newByte) {
    for (int bitIndex = 0; bitIndex < 8; bitIndex++) {
        if (((crcValue & 0x8000) >> 8) ^ (newByte & 0x80)) {
            crcValue = (crcValue << 1) ^ POLYNOM;
        } else {
            crcValue = (crcValue << 1);
        }
        newByte <<= 1;
    }
    return crcValue;
}

static uint16_t nvd_checksum(void) {
    uint16_t crc = 0x0000; // DNP initialization
    int nBytes = sizeof(NVD_PARAMS_t) - 2;
    for (int inx = 0; inx < nBytes; inx++) {
        crc = crc16(crc, Nvd.buf[inx]);
    }
    return (~crc); // DNP final operation
}

void nvd_init(void) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS namespace
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        nvd_set_defaults();
        nvd_commit();
        return;
    }

    // Read data from NVS
    size_t required_size = sizeof(NVD_PARAMS_t);
    err = nvs_get_blob(nvs_handle, NVS_DATA_KEY, Nvd.buf, &required_size);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading NVS data: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        nvd_set_defaults();
        nvd_commit();
        return;
    }

    uint16_t checkSum = nvd_checksum();
    ESP_LOGD(TAG, "Sizeof(NVD_PARAMS_t) = %d bytes\n", sizeof(NVD_PARAMS_t));
    ESP_LOGD(TAG, "Calculated checkSum = 0x%04x\n", checkSum);
    ESP_LOGD(TAG, "Saved checkSum = ~0x%04x\n", ~Nvd.par.checkSum & 0xFFFF);

    bool badData = ((Nvd.par.calib.axBias == -1) && (Nvd.par.calib.ayBias == -1) && (Nvd.par.calib.azBias == -1)) ? true : false;

    if ((badData == false) && (checkSum ^ Nvd.par.checkSum) == 0xFFFF) {
        ESP_LOGI(TAG, "NVD checkSum OK\n");
        ESP_LOGD(TAG, "WiFi Credentials\n");
        ESP_LOGD(TAG, "SSID = %s\n", Nvd.par.cfg.cred.ssid);

        ESP_LOGD(TAG, "ACCEL & GYRO Calibration Values\n");
        ESP_LOGD(TAG, "axBias = %d\n", Nvd.par.calib.axBias);
        ESP_LOGD(TAG, "ayBias = %d\n", Nvd.par.calib.ayBias);
        ESP_LOGD(TAG, "azBias = %d\n", Nvd.par.calib.azBias);
        ESP_LOGD(TAG, "gxBias = %d\n", Nvd.par.calib.gxBias);
        ESP_LOGD(TAG, "gyBias = %d\n", Nvd.par.calib.gyBias);
        ESP_LOGD(TAG, "gzBias = %d\n", Nvd.par.calib.gzBias);

        ESP_LOGD(TAG, "VARIO\n");
        ESP_LOGD(TAG, "climbThresholdCps = %d\n", Nvd.par.cfg.vario.climbThresholdCps);
        ESP_LOGD(TAG, "zeroThresholdCps = %d\n", Nvd.par.cfg.vario.zeroThresholdCps);
        ESP_LOGD(TAG, "sinkThresholdCps = %d\n", Nvd.par.cfg.vario.sinkThresholdCps);
        ESP_LOGD(TAG, "crossoverCps = %d\n", Nvd.par.cfg.vario.crossoverCps);

        ESP_LOGD(TAG, "KALMAN FILTER\n");
        ESP_LOGD(TAG, "accelVariance = %d\n", Nvd.par.cfg.kf.accelVariance);
        ESP_LOGD(TAG, "zMeasVariance = %d\n", Nvd.par.cfg.kf.zMeasVariance);

        ESP_LOGD(TAG, "MISCELLANEOUS\n");
        ESP_LOGD(TAG, "sleepTimeoutMinutes = %d\n", Nvd.par.cfg.misc.sleepTimeoutMinutes);

    } else {
        ESP_LOGE(TAG, "ERROR!! NVD BAD CHECKSUM, SETTING DEFAULTS\n");
        nvd_set_defaults();
        nvd_commit();
    }

    nvs_close(nvs_handle);
}

void nvd_set_defaults() {
    memset(Nvd.par.cfg.cred.ssid, 0, 30);
    memset(Nvd.par.cfg.cred.password, 0, 30);

    Nvd.par.calib.axBias = 0;
    Nvd.par.calib.ayBias = 0;
    Nvd.par.calib.azBias = 0;
    Nvd.par.calib.gxBias = 0;
    Nvd.par.calib.gyBias = 0;
    Nvd.par.calib.gzBias = 0;

    Nvd.par.cfg.vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
    Nvd.par.cfg.vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
    Nvd.par.cfg.vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT;
    Nvd.par.cfg.vario.crossoverCps = VARIO_CROSSOVER_CPS_DEFAULT;

    Nvd.par.cfg.kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
    Nvd.par.cfg.kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

    Nvd.par.cfg.misc.sleepTimeoutMinutes = SLEEP_TIMEOUT_MINUTES_DEFAULT;
}

esp_err_t nvd_commit(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    uint16_t checkSum = nvd_checksum();
    Nvd.par.checkSum = ~checkSum;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace for commit: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, NVS_DATA_KEY, Nvd.buf, sizeof(NVD_PARAMS_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing NVS blob: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

void nvd_save_calib_params(CALIB_PARAMS_t *calib) {
    Nvd.par.calib.axBias = calib->axBias;
    Nvd.par.calib.ayBias = calib->ayBias;
    Nvd.par.calib.azBias = calib->azBias;
    Nvd.par.calib.gxBias = calib->gxBias;
    Nvd.par.calib.gyBias = calib->gyBias;
    Nvd.par.calib.gzBias = calib->gzBias;
    nvd_commit();
}

void nvd_save_config_params(CONFIG_PARAMS_t *cfg) {
    memcpy(&Nvd.par.cfg.cred, &cfg->cred, sizeof(WIFI_CRED_t));
    memcpy(&Nvd.par.cfg.vario, &cfg->vario, sizeof(VARIO_PARAMS_t));
    memcpy(&Nvd.par.cfg.kf, &cfg->kf, sizeof(KALMAN_FILTER_PARAMS_t));
    memcpy(&Nvd.par.cfg.misc, &cfg->misc, sizeof(MISC_PARAMS_t));
    nvd_commit();
}