#include "vario_audio.h"
#include "config.h"
#include "misc_utils.h"
#include "nvd.h"
#include "task_buzzer.h"

#define TAG "vario_audio"

// clamp climbrate/sinkrate for audio feedback to +/- 10 m/s
#define VARIO_MAX_CPS         1000

#define TICK_MS 40

static int32_t CurrentCps; // internal state : current climb/sink rate
static int32_t CurrentFreqHz; // internal state : current frequency being generated
// sinktone indicates sinking air, this is a warning tone
static int32_t SinkToneCps; // threshold in cm per second
// climbtone indicates lift is strong enough to try turning to stay in it
static int32_t ClimbToneCps; // threshold in cm per second
// zeroestone indicates weak lift, possibility of stronger lift/thermal nearby
static int32_t ZeroesToneCps; // threshold in cm per second
// allocate roughly 1 decade (10:1) of speaker frequency bandwidth to climbrates below
// crossoverCps, and 1 octave (2:1) of frequency bandwidth to climbrates above
// crossoverCps. So if you are flying in strong conditions, increase crossoverCps.
// If you are flying in weak conditions, decrease crossoverCps.
static int32_t CrossoverCps;

static int Beepperiod; // internal state : current beep interval in ticks
static int BeeponTime; // internal state : current beep  on-time in ticks
static int Tick; // internal state : current tick ( 1 tick ~= 20mS)
// for offscale climbrates above +10m/s generate continuous warbling tone
static const int OffScaleHiTone[8]  = {400,800,1200,1600,2000,1600,1200,800};
// for offscale sinkrates below -10m/s generate continuous descending tone
static const int OffScaleLoTone[8]  = {4000,3500,3000,2500,2000,1500,1000,500};
// how long beep periods are when climbing, in Ticks (40ms), from 0-1m/s to 9-10m/s by step of 1m/s
static const uint8_t ClimbBeepPeriod[10] = {16, 14, 12, 10, 8, 6, 6, 6, 4, 4};

void audio_set_frequency(int freqHz) {
    if (freqHz > 0) {
        tone(freqHz, BeeponTime*TICK_MS);
    }
    else {
        noTone();
    }
}


void vaudio_config() {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    SinkToneCps    =  (int32_t)Nvd.par.cfg.vario.sinkThresholdCps;
    ClimbToneCps   =  (int32_t)Nvd.par.cfg.vario.climbThresholdCps;
    ZeroesToneCps  =  (int32_t)Nvd.par.cfg.vario.zeroThresholdCps;
    CrossoverCps   =  (int32_t)Nvd.par.cfg.vario.crossoverCps;
    ESP_LOGD(TAG, "climbToneCps = %ld\r\n", ClimbToneCps);
    ESP_LOGD(TAG, "zeroesToneCps = %ld\r\n", ZeroesToneCps);
    ESP_LOGD(TAG, "sinkToneCps = %ld\r\n", SinkToneCps);
    ESP_LOGD(TAG, "crossoverCps = %ld\r\n", CrossoverCps);
    Beepperiod	= 0;
    BeeponTime 	= 0;
    CurrentCps 		= 0;
    CurrentFreqHz  	= 0;
	Tick 			= 0;
}


static void vaudio_reset(int32_t nCps) {
	CurrentCps = nCps;
	Tick = 0;
	// if sinking significantly faster than glider sink rate in still air, generate warning sink tone
	if (CurrentCps <= SinkToneCps) {
		if (CurrentCps <= -VARIO_MAX_CPS) {
			Beepperiod = 8;
			BeeponTime = 8;
			CurrentFreqHz = OffScaleLoTone[0];
			audio_set_frequency(CurrentFreqHz);
		}
		else {
			Beepperiod = 40; // sink indicated with descending frequency beeps with long on-times
			BeeponTime  = 30;
			// descending tone starts at higher frequency for higher sink rate
			CurrentFreqHz = VARIO_SPKR_MAX_FREQHZ/2 + ((CurrentCps + VARIO_MAX_CPS)*(VARIO_SPKR_MIN_FREQHZ + 600 - VARIO_SPKR_MAX_FREQHZ/2))/(SinkToneCps + VARIO_MAX_CPS);
			CLAMP(CurrentFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
			audio_set_frequency(CurrentFreqHz);
		}
	}
	//if climbing, generate beeps
	else {
		if (CurrentCps >= ClimbToneCps) {
			if (CurrentCps >= VARIO_MAX_CPS) {
				Beepperiod = 8;
				BeeponTime = 8;
				CurrentFreqHz = OffScaleHiTone[0];
				audio_set_frequency(CurrentFreqHz);
			}
			else {
				int index = CurrentCps/100;
				if (index > 9) index = 9;
				Beepperiod = ClimbBeepPeriod[index];
				BeeponTime = ClimbBeepPeriod[index]/2;
				if (CurrentCps > CrossoverCps) {
					CurrentFreqHz = VARIO_CROSSOVER_FREQHZ + ((CurrentCps - CrossoverCps)*(VARIO_SPKR_MAX_FREQHZ - VARIO_CROSSOVER_FREQHZ))/(VARIO_MAX_CPS - CrossoverCps);
					}
				else {
					CurrentFreqHz = VARIO_SPKR_MIN_FREQHZ + ((CurrentCps - ZeroesToneCps)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(CrossoverCps - ZeroesToneCps);
					}
				CLAMP(CurrentFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
				audio_set_frequency(CurrentFreqHz);
			}
		}
		else if (CurrentCps >= ZeroesToneCps) {
			// in "zeroes" band, indicate with a short pulse and long interval
			Beepperiod = 30;
			BeeponTime = 4;
			CurrentFreqHz = VARIO_SPKR_MIN_FREQHZ + ((CurrentCps - ZeroesToneCps)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(CrossoverCps - ZeroesToneCps);
			CLAMP(CurrentFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
			audio_set_frequency(CurrentFreqHz);
		}
		// between zeroes threshold and sink threshold, chillout
		else {
			Beepperiod = 0;
			BeeponTime  = 0;
			CurrentFreqHz = 0;
			audio_set_frequency(CurrentFreqHz);
		}
	}
}


void vaudio_tick_handler(int32_t nCps) {
    // generate new beep/tone only if current beep/tone has ended, OR climb threshold exceeded
    if ((Beepperiod <= 0)  || ((nCps >= ClimbToneCps) && (CurrentCps < ClimbToneCps)) ) {
		vaudio_reset(nCps);
        }
    else { // still processing current beep/tone
		int32_t newFreqHz;
        Tick++;
        Beepperiod--;

        if (CurrentCps >= VARIO_MAX_CPS) { // offscale climbrate (>= +10m/s) indicated with continuous warbling tone
            BeeponTime = 1;
            newFreqHz = OffScaleHiTone[Tick];
        }
        else if (CurrentCps <= -VARIO_MAX_CPS) {  // offscale sink (<= -10m/s) indicated with continuous descending tone
            BeeponTime = 1;
            newFreqHz = OffScaleLoTone[Tick];
        }
        else if (CurrentCps <= SinkToneCps) {  // sink is indicated with a descending frequency beep
            ESP_LOGD(TAG, "sink %ld", nCps);
            BeeponTime = 1;
            newFreqHz = CurrentFreqHz - 20;
        }
        else {
            newFreqHz = CurrentFreqHz; // no change
        }

        if (newFreqHz != CurrentFreqHz) {
            CurrentFreqHz = newFreqHz;
            audio_set_frequency(CurrentFreqHz);
            }
        }
    }
