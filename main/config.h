#pragma once

////////////////////////////////////////////////////////////////////////

#define pinAudio 	 7
#define pinSENSE_VBAT 	 0
#define pinDRDYInt   6

#define SENSE_VBAT_CHAN ADC_CHANNEL_0

#define pinBUTTON pinSENSE_VBAT
#define pinLED_R 	 5
#define pinLED_G 	 1
#define pinLED_B 	 10
#define pinSDA       2
#define pinSCL       3

#define LED_OFF 1
#define LED_ON 0

////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet
// Note: modern EN-A & EN-B gliders have a sink rate around 110cm/s in still air

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   	20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   	100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_ZERO_THRESHOLD_CPS_MIN    	-20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    	20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	-250
#define VARIO_SINK_THRESHOLD_CPS_MIN    	-400
#define VARIO_SINK_THRESHOLD_CPS_MAX    	-100

// When generating climbtones, the vario allocates most of the speaker
// frequency bandwidth to climbrates below this crossover threshold
// so you have more frequency discrimination. So set the crossover threshold
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT     400
#define VARIO_CROSSOVER_CPS_MIN         300
#define VARIO_CROSSOVER_CPS_MAX         800

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     100
#define KF_ACCEL_VARIANCE_MIN         50
#define KF_ACCEL_VARIANCE_MAX         150

#define KF_ZMEAS_VARIANCE_DEFAULT    100
#define KF_ZMEAS_VARIANCE_MIN        100
#define KF_ZMEAS_VARIANCE_MAX        400

// Sleep timeout. The vario will go into sleep mode
// if it does not detect climb or sink rates more than
// SLEEP_THRESHOLD_CPS, for the specified minutes.
// You can only exit  sleep mode by power cycling the unit.
#define SLEEP_TIMEOUT_MINUTES_DEFAULT   15
#define SLEEP_TIMEOUT_MINUTES_MIN       5
#define SLEEP_TIMEOUT_MINUTES_MAX       30

// audio feedback tones
#define BATTERY_TONE_HZ       400
#define CALIBRATING_TONE_HZ   800
#define UNCALIBRATED_TONE_HZ  2000

///////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

// This is set low as the residual acceleration bias after calibration
// is expected to have little variation/drift
#define KF_ACCELBIAS_VARIANCE   0.005f

// KF4 Acceleration Update variance default
#define KF_ACCEL_UPDATE_VARIANCE   50.0f

// any climb/sinkrate excursions beyond this level will keep the
// vario active. If it stays below this level for the configured
// time interval, vario goes to sleep to conserve power
#define SLEEP_THRESHOLD_CPS    50

// if you find that gyro calibration fails even when you leave
// the unit undisturbed, increase this offset limit
// until you find that gyro calibration works consistently.
#define GYRO_OFFSET_LIMIT_1000DPS   	200

#define TASK_OTA 0
#define TASK_BUZZER 1
#define TASK_USER_INPUT 2
#define TASK_VARIO 3
