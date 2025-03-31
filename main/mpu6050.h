#pragma once

#include "stdint.h"
#include "driver/i2c_master.h"

#include "config.h"
#include "nvd.h"

#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E

#define FIFO_EN            0x23
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48

#define MOT_DETECT_STATUS  0x61
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define WHO_AM_I_MPU6050   0x75 // Should return 0x71

// i2c 7bit device address is 110 100[AD0] = 0x68
#define MPU6050_I2C_ADDRESS 	0x68

#define MPU6050_2G_SENSITIVITY 		16.384f 	// lsb per milli-g
#define MPU6050_4G_SENSITIVITY     8.192f   // lsb per milli-g

#define MPU6050_500DPS_SENSITIVITY	65.5f 		// lsb per deg/sec
#define MPU6050_1000DPS_SENSITIVITY  32.8f     // lsb per deg/sec

// Constants
#define ACCEL_NUM_AVG_SAMPLES       100
#define GYRO_NUM_CALIB_SAMPLES      50
#define MPU6050_TIMEOUT 50
#define MPU6050_I2C_SPEED 400000


// MPU6050 handle/context structure
typedef struct {
    // I2C master handle via port with configuration
    i2c_master_dev_handle_t i2c_dev;
    // I2C master configuration
    i2c_device_config_t dev_cfg;
    // I2C master handle via port
    i2c_master_bus_handle_t bus_handle;

    // Calibration parameters
    int16_t axBias;
    int16_t ayBias;
    int16_t azBias;
    int16_t gxBias;
    int16_t gyBias;
    int16_t gzBias;

    // Scaling factors
    float aScale;
    float gScale;
} mpu6050_t;

// Function prototypes
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle);
esp_err_t mpu6050_device_create(const uint16_t dev_addr);
void mpu6050_get_accel_gyro_data(float* pAccelData, float* pGyroData);
int mpu6050_check_id(uint8_t address);
void mpu6050_get_calib_params(CALIB_PARAMS_t* calib);
void mpu6050_sleep();
void mpu6050_config_accel_gyro();
void mpu6050_calibrate_accel(CALIB_PARAMS_t* calib);
int mpu6050_calibrate_gyro(CALIB_PARAMS_t* calib);
esp_err_t mpu6050_write_byte(uint8_t registerAddress, uint8_t d);
uint8_t mpu6050_read_byte(uint8_t registerAddress);
esp_err_t mpu6050_read_bytes(uint8_t registerAddress, uint8_t count, uint8_t* dest);
void mpu6050_print_param(const char* name, uint8_t registerAddress, uint8_t len);
void mpu6050_get_accel_self_test_factory_trim();