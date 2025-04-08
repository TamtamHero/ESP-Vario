#include "task_vario.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_types.h"

#include "config.h"
#include "nvd.h"
#include "misc_utils.h"
#include "bmx280.h"
#include "mpu6050.h"
#include "kalmanfilter4.h"
#include "ringbuf.h"
#include "imu.h"
#include "adc.h"
#include "vario_audio.h"
#include "ui.h"
#include "task_ble_server.h"

#define TAG "vario"

#define PIN_DRDY_MASK (1ULL<<pinDRDYInt)

#define ESP_INTR_FLAG_DEFAULT 0

#define BARO_COUNTING 20

i2c_master_bus_handle_t bus_handle;
bmx280_t* bmx280;

volatile int mpu6050_DRDY_counter;
volatile bool mpu6050_DRDY;
volatile int SleepCounter;
volatile int baro_cnt;
volatile int SleepTimeoutSecs;
volatile int Ticks;

uint32_t TimePreviousUs;
uint32_t TimeNowUs;
float 	 ImuTimeDeltaUSecs; // time between mpu6050 samples, in microseconds
float 	 KfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

float AccelmG[3]; // in milli-Gs
float GyroDps[3];  // in degrees/second
float KfAltitudeCm = 0.0f; // kalman filtered altitude in cm
float KfClimbrateCps  = 0.0f; // kalman filtered climbrate in cm/s

// interrupt service routine, called when MPU6050 INT GPIO is rising
void IRAM_ATTR mpu6050_isr_handler(void* arg) {
    mpu6050_DRDY = true;
	mpu6050_DRDY_counter++;
}

void time_init() {
	TimeNowUs = TimePreviousUs = esp_timer_get_time();
}

static inline void time_update(){
	TimeNowUs = esp_timer_get_time();
	ImuTimeDeltaUSecs = TimeNowUs > TimePreviousUs ? (float)(TimeNowUs - TimePreviousUs) : 2000.0f; // if rollover use expected time difference
	TimePreviousUs = TimeNowUs;
}

i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI("test","I2C master bus created");
    return bus_handle;
}

esp_err_t init_bmp280(){
    bmx280 = bmx280_create_master(bus_handle);
    if (!bmx280) {
        ESP_LOGE("test", "Could not create bmx280 driver.");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    // BMP280 has high latency, we make it run continuously while in operation
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_CYCLE));
    return ESP_OK;
}

esp_err_t init_mpu6050(){
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu6050_init(bus_handle));

    // configure MPU6050 to start generating gyro and accel data
    mpu6050_config_accel_gyro();

    // calibrate sensor
    ui_calibrate_accel_gyro();

    // Interrupt setup
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = PIN_DRDY_MASK ;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf));
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(pinDRDYInt, mpu6050_isr_handler, NULL);

    return ESP_OK;
}

esp_err_t init_vario(){

    // indicate battery voltage via buzzer
    ui_indicate_battery_voltage();

    // init sensors
    bus_handle = i2c_bus_init(pinSDA, pinSCL);
    ESP_ERROR_CHECK_WITHOUT_ABORT(init_bmp280());
    ESP_ERROR_CHECK_WITHOUT_ABORT(init_mpu6050());

    // fetch current pressure
    float temp = 0, pres = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmx280_readoutFloat(bmx280, &temp, &pres, NULL));

    // init Kalman filter
    kalmanFilter4_configure((float)Nvd.par.cfg.kf.zMeasVariance, 1000.0f*(float)Nvd.par.cfg.kf.accelVariance, true, pressure_to_altitude(pres, true), 0.0f, 0.0f);

    // retrieve beep parameters from nvd
    vaudio_config();

    time_init();
	KfTimeDeltaUSecs = 0.0f;
	baro_cnt = 0;
	SleepTimeoutSecs = 0;
    Ticks = 0;
	ringbuf_init();
	SleepCounter = 0;

    // init BLE server
    ESP_ERROR_CHECK_WITHOUT_ABORT(ble_server_init());

    return ESP_OK;
}

void task_vario(void *pvParameter)
{
    ESP_LOGI(TAG, "Task begins...");
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(init_vario());

    float battery_voltage = adc_read_battery_voltage(SENSE_VBAT_CHAN);
    int32_t audioCps = 0; // filtered climbrate, rounded to nearest cm/s

    ESP_LOGI(TAG, "%fV", battery_voltage);
    uint32_t cnt = 0;
    while(1){
        if(mpu6050_DRDY){
            cnt++;
            mpu6050_DRDY = false;
            time_update();

            mpu6050_get_accel_gyro_data(AccelmG, GyroDps);
            // ESP_LOGD(TAG, "mpu: %f %f %f - %f %f %f", AccelmG[0], AccelmG[1], AccelmG[2], GyroDps[0], GyroDps[1], GyroDps[2]);

            // We arbitrarily decide that in the assembled vario, the CJMCU-117 board silkscreen Y points "forward" or "north",
            // silkscreen X points "right" or "east", and silkscreen Z points down. This is the North-East-Down (NED)
            // right-handed coordinate frame used in our AHRS algorithm implementation.
            // The required mapping from sensor samples to NED frame for our specific board orientation is :
            // gxned = gx, gyned = gy, gzned = -gz (clockwise rotations about the axis must result in +ve readings on the sensor axis channel)
            // axned = ay, ayned = ax, azned = az (when the axis points down, sensor axis channel reading must be +ve)
            // The AHRS algorithm expects rotation rates in radians/second
            // Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
            float accelMagnitudeSquared = AccelmG[0]*AccelmG[0] + AccelmG[1]*AccelmG[1] + AccelmG[2]*AccelmG[2];
            int bUseAccel = ((accelMagnitudeSquared > 562500.0f) && (accelMagnitudeSquared < 1562500.0f)) ? 1 : 0;
            float dtIMU = ImuTimeDeltaUSecs/1000000.0f;
            float gxned = DEG_TO_RAD*GyroDps[0];
            float gyned = DEG_TO_RAD*GyroDps[1];
            float gzned = -DEG_TO_RAD*GyroDps[2];
            float axned = AccelmG[1];
            float ayned = AccelmG[0];
            float azned = AccelmG[2];
            imu_mahonyAHRS_update6DOF(bUseAccel, dtIMU, gxned, gyned, gzned, axned, ayned, azned);
            float gCompensatedAccel = imu_gravity_compensated_accel(axned, ayned, azned, Q0, Q1, Q2, Q3);
            ringbuf_add_sample(gCompensatedAccel);

            baro_cnt++;
            KfTimeDeltaUSecs += ImuTimeDeltaUSecs;

            if (baro_cnt >= BARO_COUNTING) {
                baro_cnt = 0;
                // average earth-z acceleration over the 38mS interval between z samples
                // is used in the kf algorithm update phase
                float zAccelAverage = ringbuf_average_newest_samples(BARO_COUNTING);
                float dtKF = KfTimeDeltaUSecs/1000000.0f;
                kalmanFilter4_predict(dtKF);

                float temp = 0, pres = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(bmx280_readoutFloat(bmx280, &temp, &pres, NULL));
                float alti_cm = pressure_to_altitude(pres, true);

                kalmanFilter4_update(alti_cm, zAccelAverage, (float*)&KfAltitudeCm, (float*)&KfClimbrateCps);

                KfTimeDeltaUSecs = 0.0f;
                audioCps =  KfClimbrateCps >= 0.0f ? (int32_t)(KfClimbrateCps+0.5f) : (int32_t)(KfClimbrateCps-0.5f);
                vaudio_tick_handler(audioCps);
                if (ABS(audioCps) > SLEEP_THRESHOLD_CPS) {
                    // reset sleep timeout watchdog if there is significant vertical motion
                    SleepTimeoutSecs = 0;
                }
                else if (SleepTimeoutSecs >= (Nvd.par.cfg.misc.sleepTimeoutMinutes*10*60)) {
                    ESP_LOGI(TAG, "Timed out with no significant climb/sink, put mpu6050 and ESP to sleep to minimize current draw");
                    ui_indicate_sleep();
                    ui_go_to_sleep();
                    SleepTimeoutSecs = 0;
                }
            }

            if (mpu6050_DRDY_counter >= 50) {
                mpu6050_DRDY_counter = 0; // 0.1 second elapsed
                // int altM =  KfAltitudeCm > 0.0f ? (int)((KfAltitudeCm+50.0f)/100.0f) :(int)((KfAltitudeCm-50.0f)/100.0f);
                float KfPressure_Pa = altitude_to_pressure(KfAltitudeCm/100.0, false);
                ble_transmit_LK8EX1(KfPressure_Pa, audioCps, battery_voltage);
                // ESP_LOGD(TAG, "%fm %ld cps %fV", KfAltitudeCm/100.0, audioCps, battery_voltage);
                SleepTimeoutSecs++;
                Ticks++;

                // update battery voltage once per minute
                if(Ticks % 600 == 0){
                    battery_voltage = adc_read_battery_voltage(SENSE_VBAT_CHAN);
                    ESP_LOGD(TAG, "battery voltage: %fV", battery_voltage);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}