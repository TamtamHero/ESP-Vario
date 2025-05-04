#pragma once

#include "stdint.h"
#include "esp_system.h"

#include "config.h"
#include "vario_audio.h"

#define NVD_SIZE_BYTES 90

typedef struct {
	char ssid[30];
	char password[30];
	} WIFI_CRED_t;

typedef struct {
	int16_t  axBias;
	int16_t  ayBias;
	int16_t  azBias;
	int16_t  gxBias;
	int16_t  gyBias;
	int16_t  gzBias;
	} CALIB_PARAMS_t;

typedef union {
    struct {
        sound_profile_point_t  offscale_low;
        sound_profile_point_t  sink_threshold;
        sound_profile_point_t  glide_threshold;
        sound_profile_point_t  glide_ceiling;
        sound_profile_point_t  zeroing_threshold;
        sound_profile_point_t  zeroing_ceiling;
        sound_profile_point_t  climb_threshold;
        sound_profile_point_t  fast_climb_threshold;
        sound_profile_point_t  offscale_high;
    };
    sound_profile_point_t points[NUM_SOUND_PROFILE_POINTS];
} VARIO_PARAMS_t;

typedef struct  {
	int16_t  accelVariance; // environmental acceleration disturbance variance, divided by 1000
	int16_t  kAdapt; // z measurement noise variance
	} KALMAN_FILTER_PARAMS_t;

typedef struct  {
	int16_t  sleepTimeoutMinutes;
} MISC_PARAMS_t;

typedef struct {
	WIFI_CRED_t cred;
	VARIO_PARAMS_t vario;
	KALMAN_FILTER_PARAMS_t kf;
	MISC_PARAMS_t misc;
	} CONFIG_PARAMS_t;

typedef struct {
	CALIB_PARAMS_t  calib;
	CONFIG_PARAMS_t cfg;
	uint16_t checkSum;
	} NVD_PARAMS_t;

typedef union  {
	NVD_PARAMS_t par;
	uint8_t	   buf[NVD_SIZE_BYTES];
	} NVD_t;

extern NVD_t Nvd;

void nvd_init(void);
void nvd_set_defaults();
void nvd_save_calib_params(CALIB_PARAMS_t *calib);
void nvd_save_config_params(CONFIG_PARAMS_t *cfg);
esp_err_t nvd_commit(void);


