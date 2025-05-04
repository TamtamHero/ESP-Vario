#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <string.h>
#include <math.h>

#include "task_buzzer.h"
#include "misc_utils.h"

#define TAG "mpu6050"

mpu6050_t* mpu6050 = NULL;

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle) {
    if (mpu6050 == NULL) {
        mpu6050 = (mpu6050_t*)malloc(sizeof(mpu6050_t));
    }
    if (mpu6050 == NULL) {
        return ESP_FAIL;
    }

    memset(mpu6050, 0, sizeof(mpu6050_t));

    // set bus
    mpu6050->bus_handle = bus_handle;
    mpu6050->dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mpu6050->dev_cfg.device_address = 0xDE;
    mpu6050->dev_cfg.scl_speed_hz =MPU6050_I2C_SPEED;
    mpu6050->i2c_dev = NULL;

    // probe for sensor on bus
    esp_err_t err = mpu6050_device_create(MPU6050_I2C_ADDRESS);
    if (err != ESP_OK) return err;
    if ((err = mpu6050_check_id(MPU6050_I2C_ADDRESS)) != ESP_OK)
    {
        err = mpu6050_device_create(MPU6050_I2C_ADDRESS+1);
        if (err != ESP_OK) return err;
        if ((err = mpu6050_check_id(MPU6050_I2C_ADDRESS+1)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Sensor not found.");
            return err;
        }
    }

    // Example accel biases (actual values should be calibrated)
    mpu6050->axBias = -80;
    mpu6050->ayBias = -33;
    mpu6050->azBias = -386;
    mpu6050->aScale = 1.0f / MPU6050_4G_SENSITIVITY;

    // Example gyro biases (actual values should be calibrated)
    mpu6050->gxBias = 23;
    mpu6050->gyBias = -9;
    mpu6050->gzBias = 24;
    mpu6050->gScale = 1.0f / MPU6050_1000DPS_SENSITIVITY;

    return ESP_OK;
}

esp_err_t mpu6050_device_create(const uint16_t dev_addr)
{
    ESP_LOGI("mpu6050", "device_create for BMP280/BME280 sensors on ADDR %X", dev_addr);
    mpu6050->dev_cfg.device_address = dev_addr;
    // Add device to the I2C bus
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_add_device(mpu6050->bus_handle, &mpu6050->dev_cfg, &mpu6050->i2c_dev));
    return ESP_OK;
}

void mpu6050_get_accel_gyro_data(float* pAccelData, float* pGyroData) {
    uint8_t buf[14];
    int16_t raw[3];

    mpu6050_read_bytes(ACCEL_XOUT_H, 14, buf);

    // Process accelerometer data
    raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    pAccelData[0] = (float)(raw[0] - mpu6050->axBias) * mpu6050->aScale;
    pAccelData[1] = (float)(raw[1] - mpu6050->ayBias) * mpu6050->aScale;
    pAccelData[2] = (float)(raw[2] - mpu6050->azBias) * mpu6050->aScale;

    // Process gyroscope data
    raw[0] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
    raw[1] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
    raw[2] = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);
    pGyroData[0] = (float)(raw[0] - mpu6050->gxBias) * mpu6050->gScale;
    pGyroData[1] = (float)(raw[1] - mpu6050->gyBias) * mpu6050->gScale;
    pGyroData[2] = (float)(raw[2] - mpu6050->gzBias) * mpu6050->gScale;
}

esp_err_t mpu6050_check_id(uint8_t address) {
    uint8_t whoami = mpu6050_read_byte(WHO_AM_I_MPU6050);
    return ((whoami == address) ? ESP_OK : ESP_FAIL);
}

void mpu6050_get_calib_params(CALIB_PARAMS_t* calib) {
    mpu6050->axBias = calib->axBias;
    mpu6050->ayBias = calib->ayBias;
    mpu6050->azBias = calib->azBias;
    mpu6050->gxBias = calib->gxBias;
    mpu6050->gyBias = calib->gyBias;
    mpu6050->gzBias = calib->gzBias;

    ESP_LOGD(TAG, "Calibration parameters from NVD");
    ESP_LOGD(TAG, "Accel : axBias %d, ayBias %d, azBias %d\r\n",
               mpu6050->axBias, mpu6050->ayBias, mpu6050->azBias);
    ESP_LOGD(TAG, "Gyro : gxBias %d, gyBias %d, gzBias %d\r\n",
               mpu6050->gxBias, mpu6050->gyBias, mpu6050->gzBias);
}

void mpu6050_sleep() {
    mpu6050_write_byte(PWR_MGMT_1, 0x40);
}

void mpu6050_config_accel_gyro() {
    // Reset mpu6050
    mpu6050_write_byte(PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Select best available clock source
    mpu6050_write_byte(PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Fsync disabled, gyro bandwidth = 184Hz
    mpu6050_write_byte(CONFIG, 0x01);

    // Output data rate = 500Hz
    mpu6050_write_byte(SMPLRT_DIV, 0x01);

    // Set gyro FS = 1000dps, fchoice_b = 00
    mpu6050_write_byte(GYRO_CONFIG, 0x10);

    // Set accelerometer FS = +/-4G
    mpu6050_write_byte(ACCEL_CONFIG, 0x08);

    // Set accelerometer BW = 184Hz
    mpu6050_write_byte(ACCEL_CONFIG2, 0x01);

    // Interrupt configuration
    mpu6050_write_byte(INT_PIN_CFG, 0x10);
    mpu6050_write_byte(INT_ENABLE, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void mpu6050_calibrate_accel(CALIB_PARAMS_t* calib) {
    uint8_t buf[6];
    int16_t x, y, z;
    int32_t axAccum, ayAccum, azAccum;

    axAccum = ayAccum = azAccum = 0;

    for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++) {
        mpu6050_read_bytes(ACCEL_XOUT_H, 6, buf);
        x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
        y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
        z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);

        axAccum += (int32_t)x;
        ayAccum += (int32_t)y;
        azAccum += (int32_t)z;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    x = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
    y = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
    z = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);

    x = -10 * MPU6050_4G_SENSITIVITY;
    y = 60 * MPU6050_4G_SENSITIVITY;
    z = 8192 + (MPU6050_4G_SENSITIVITY * 15);

    ESP_LOGD(TAG, "ax = %d  ay = %d  az = %d\r\n", x, y, z);

    calib->axBias = mpu6050->axBias = y;
    calib->ayBias = mpu6050->ayBias = x;
    calib->azBias = mpu6050->azBias = z > 0 ? z - (int16_t)(1000.0f * MPU6050_4G_SENSITIVITY) :
                                           z + (int16_t)(1000.0f * MPU6050_4G_SENSITIVITY);

    ESP_LOGD(TAG, "axBias = %d\r\n", (int)mpu6050->axBias);
    ESP_LOGD(TAG, "ayBias = %d\r\n", (int)mpu6050->ayBias);
    ESP_LOGD(TAG, "azBias = %d\r\n", (int)mpu6050->azBias);
}

int mpu6050_calibrate_gyro(CALIB_PARAMS_t* calib) {
    uint8_t buf[6];
    int16_t gx, gy, gz;
    int32_t gxAccum, gyAccum, gzAccum;
    int foundBadData;
    int numTries = 1;

    do {
        vTaskDelay(pdMS_TO_TICKS(500));
        foundBadData = 0;
        gxAccum = gyAccum = gzAccum = 0;

        for (int inx = 0; inx < GYRO_NUM_CALIB_SAMPLES; inx++) {
            mpu6050_read_bytes(GYRO_XOUT_H, 6, buf);
            gx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
            gy = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
            gz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);

            if ((ABS(gx) > GYRO_OFFSET_LIMIT_1000DPS) ||
                (ABS(gy) > GYRO_OFFSET_LIMIT_1000DPS) ||
                (ABS(gz) > GYRO_OFFSET_LIMIT_1000DPS)) {
                foundBadData = 1;
                break;
            }

            gxAccum += (int32_t)gx;
            gyAccum += (int32_t)gy;
            gzAccum += (int32_t)gz;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    } while (foundBadData && (++numTries < 10));

    if (!foundBadData) {
        calib->gxBias = mpu6050->gxBias = (int16_t)(gxAccum / GYRO_NUM_CALIB_SAMPLES);
        calib->gyBias = mpu6050->gyBias = (int16_t)(gyAccum / GYRO_NUM_CALIB_SAMPLES);
        calib->gzBias = mpu6050->gzBias = (int16_t)(gzAccum / GYRO_NUM_CALIB_SAMPLES);
    }

    ESP_LOGD(TAG, "Num Tries = %d\r\n", numTries);
    ESP_LOGD(TAG, "gxBias = %d\r\n", mpu6050->gxBias);
    ESP_LOGD(TAG, "gyBias = %d\r\n", mpu6050->gyBias);
    ESP_LOGD(TAG, "gzBias = %d\r\n", mpu6050->gzBias);

    return (foundBadData ? 0 : 1);
}

esp_err_t mpu6050_write_byte(uint8_t registerAddress, uint8_t d) {
    uint8_t dat[2] = {registerAddress, d};
    return i2c_master_transmit(mpu6050->i2c_dev, dat, 2, MPU6050_TIMEOUT);
}

uint8_t mpu6050_read_byte(uint8_t registerAddress) {
    uint8_t d = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu6050_read_bytes(registerAddress, 1, &d));
    return d;
}

esp_err_t mpu6050_read_bytes(uint8_t registerAddress, uint8_t count, uint8_t* dest) {
    return i2c_master_transmit_receive(mpu6050->i2c_dev, &registerAddress, sizeof(registerAddress), dest, count, MPU6050_TIMEOUT);
}

void mpu6050_print_param(const char* name, uint8_t registerAddress, uint8_t len) {
    ESP_LOGD(TAG, "%s ", name);
    for (size_t i = 0; i < len; i++) {
        uint8_t ret = mpu6050_read_byte(registerAddress + i);
        ESP_LOGD(TAG, "%d ", ret);
    }
    ESP_LOGD(TAG, "\n");
}

void mpu6050_get_accel_self_test_factory_trim() {
    uint8_t x, y, z, low;

    x = mpu6050_read_byte(13);
    low = mpu6050_read_byte(16);
    x = (x >> 3) | ((low >> 4) & 0x03);

    y = mpu6050_read_byte(14);
    low = mpu6050_read_byte(16);
    y = (y >> 3) | ((low >> 2) & 0x03);

    z = mpu6050_read_byte(15);
    low = mpu6050_read_byte(16);
    z = (z >> 3) | (low & 0x03);

    ESP_LOGD(TAG, "axT:%d ayT:%d azT:%d\n", x, y, z);
}
