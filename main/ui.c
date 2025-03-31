#include "ui.h"
#include "config.h"
#include "task_buzzer.h"
#include "task_user_inputs.h"
#include "adc.h"
#include "mpu6050.h"

#define TAG "ui"

#define FORCE_CALIB_ACC_DELAY 15

// !! Accelerometer calibration is REQUIRED for normal vario operation. !!
// If flash was completely erased, or MPU6050 calibration data in flash was never initialized, or
// imu calibration data is corrupt, the accel and gyro biases are set to 0. Uncalibrated
// state is indicated with a continuous sequence of alternating high and low beeps for 5 seconds.
void ui_indicate_uncalibrated_accel_gyro() {
    for (int cnt = 0; cnt < 5; cnt++) {
        tone(UNCALIBRATED_TONE_HZ, 500);
        tone(UNCALIBRATED_TONE_HZ/2, 500);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// "no-activity" timeout sleep is indicated with a series of descending
// tones. If you do hear this, switch off the vario as there is still
// residual current draw from the circuit components in sleep mode
void ui_indicate_sleep() {
	tone(2000,1000);
	tone(1000,1000);
	tone(500, 1000);
	tone(250, 1000);
    vTaskDelay(pdMS_TO_TICKS(4000));
}

void ui_indicate_battery_voltage() {
   int numBeeps;
   float battery_voltage = adc_read_battery_voltage(SENSE_VBAT_CHAN);
   ESP_LOGD(TAG, "\r\nBattery voltage = %.2fV\r\n", battery_voltage);

   if (battery_voltage >= 4.0f) numBeeps = 5;
   else if (battery_voltage >= 3.9f) numBeeps = 4;
   else if (battery_voltage >= 3.7f) numBeeps = 3;
   else if (battery_voltage >= 3.6f) numBeeps = 2;
   else  numBeeps = 1;

   while (numBeeps--) {
        tone(BATTERY_TONE_HZ, 300);
        silence(300);
        vTaskDelay(pdMS_TO_TICKS(600));
    }
}

void ui_calibrate_accel(CALIB_PARAMS_t *calib) {
    ESP_LOGD(TAG, "-- Accelerometer calibration --");
    ESP_LOGD(TAG, "Place vario on a level surface with accelerometer z axis vertical and leave it undisturbed");
    ESP_LOGD(TAG, "You have 10 seconds, counted down with rapid beeps from 50 to 0");
    // clear previous potential sounds in the queue
    noTone();
    for (int inx = 0; inx < 5; inx++) {
        ESP_LOGD(TAG, "%d", 5-inx);
        tone(CALIBRATING_TONE_HZ, 50);
        silence(200);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    ESP_LOGD(TAG, "\r\nCalibrating accelerometer");
    mpu6050_calibrate_accel(calib);
    ESP_LOGD(TAG, "Accelerometer calibration done");
    nvd_save_calib_params(calib);
    }


void ui_calibrate_gyro(CALIB_PARAMS_t *calib) {
   ESP_LOGD(TAG, "\r\nCalibrating gyro");
  // normal power-on operation flow, always attempt to calibrate gyro. If calibration isn't possible because
  // the unit is continuously disturbed (e.g. you turned on the unit while already flying), indicate this and
  // use the last saved gyro biases. Otherwise, save the new gyro biases to flash memory
  if (mpu6050_calibrate_gyro(calib)) {
    ESP_LOGD(TAG, "Gyro calibration OK");
    tone(CALIBRATING_TONE_HZ, 1000);
    nvd_save_calib_params(calib);
    }
  else {
    ESP_LOGD(TAG, "Gyro calibration failed");
    tone(CALIBRATING_TONE_HZ, 1000);
    silence(500);
    tone(CALIBRATING_TONE_HZ/2, 1000);
    }
}


// Vario will attempt to calibrate gyro each time on power up. If the vario is disturbed, it will
// use the last saved gyro calibration values.
// The software delays a few seconds so that the unit can be left undisturbed for gyro calibration.
// This delay is indicated with a series of 10 short beeps. While it is beeping, if you press the
// pgmconfcal button, the unit will calibrate the accelerometer first and then the gyro.
// As soon as you hear the long confirmation tone, release the button and
// put the unit in accelerometer calibration position resting undisturbed on a horizontal surface
// with the accelerometer +z axis pointing vertically downwards. You will have some time
// to do this, indicated by a series of beeps. After calibration, the unit will generate another
// tone, save the calibration parameters to flash, and continue with normal vario operation
void ui_calibrate_accel_gyro() {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
	bool bCalibrateAccel = false;
    // load the accel & gyro calibration parameters from the non-volatile data structure
    mpu6050_get_calib_params(&Nvd.par.calib);
  	if ((Nvd.par.calib.axBias == 0) && (Nvd.par.calib.ayBias == 0) && (Nvd.par.calib.azBias == 0)) {
    	ESP_LOGD(TAG, "! Uncalibrated accelerometer !");
    	ui_indicate_uncalibrated_accel_gyro();
		bCalibrateAccel = true;
    	}
	if (bCalibrateAccel == true) {
    	ESP_LOGD(TAG, "Starting accelerometer calibration");
		ui_calibrate_accel(&Nvd.par.calib);
		bCalibrateAccel = false;
		}
	ESP_LOGD(TAG, "Counting down to gyro calibration");
	ESP_LOGD(TAG, "Press the _BTN button to enforce accelerometer calibration first");
    delegate_task_user_input(true);
	for (int inx = 0; inx < FORCE_CALIB_ACC_DELAY; inx++) {
		tone(CALIBRATING_TONE_HZ*2, 50);
        silence(50);
	}
    btn_state_t button_state = NO_EVENT;
    xQueueReceive(task_user_input_queue, &button_state, pdMS_TO_TICKS(FORCE_CALIB_ACC_DELAY*(50+50)));
    // manual disabling of input delegation, useful in case no event happened
    delegate_task_user_input(false);
    if(button_state == BTN_PRESS) {
        ESP_LOGD(TAG, "Forced accel calibration");
        ui_calibrate_accel(&Nvd.par.calib);
    }
    // in all case, calibrate the gyroscope
    ui_calibrate_gyro(&Nvd.par.calib);
}


// residual current draw in sleep mode is the sum of ESP8266 deep sleep mode current,
// MPU6050 sleep mode current, MS5611 standby current, quiescent current of voltage
// regulators, and miscellaneous current through resistive paths e.g. the
// ADC voltage divider.
void ui_go_to_sleep() {
	noTone(); // switch off pwm audio
	mpu6050_sleep(); // put MPU6050 in sleep mode
	// esp_deep_sleep_start(); // ESP8266 in sleep can only recover with a reset/power cycle
}


