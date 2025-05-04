#include "vario_audio.h"
#include "config.h"
#include "misc_utils.h"
#include "nvd.h"
#include "task_buzzer.h"

#define TAG "vario_audio"

#define CONFIG Nvd.par.cfg.vario

#define TICK_MS 40

static int16_t Current_climb_rate; // internal state : current climb/sink rate
static uint16_t Current_freq; // internal state : current frequency being generated

static uint16_t Beep_window; // internal state : current beep interval in ticks
static int Tick; // internal state : current tick ( 1 tick ~= 20mS)

// for offscale climbrates above +10m/s generate continuous warbling tone
static const uint16_t OffScaleHiTone[8]  = {400,800,1200,1600,2000,1600,1200,800};
// for offscale sinkrates below -10m/s generate continuous descending tone
static const uint16_t OffScaleLoTone[8]  = {4000,3500,3000,2500,2000,1500,1000,500};

static float Freq_slopes[NUM_SOUND_PROFILE_POINTS-1];
static float Freq_offsets[NUM_SOUND_PROFILE_POINTS-1];
static float Length_slopes[NUM_SOUND_PROFILE_POINTS-1];
static float Length_offsets[NUM_SOUND_PROFILE_POINTS-1];

void vaudio_init() {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    // compute slopes and offsets from sound profile to faciliate future interpolation
    sound_profile_point_t *points = CONFIG.points;
    for (size_t i = 0; i < NUM_SOUND_PROFILE_POINTS-1; i++)
    {
        if(points[i].cps == points[i+1].cps){
            Freq_slopes[i] = 0;
            Length_slopes[i] = 0;
        }
        else{
            Freq_slopes[i] = (float)(points[i+1].frequency - points[i].frequency)/(points[i+1].cps - points[i].cps);
            Length_slopes[i] = (float)(points[i+1].length - points[i].length)/(points[i+1].cps - points[i].cps);
        }
        Freq_offsets[i] = Freq_slopes[i] == 0 ? (float)points[i].frequency : (float)points[i].frequency - (points[i].cps * Freq_slopes[i]);
        Length_offsets[i] = Length_slopes[i] == 0 ? (float)points[i].length : (float)points[i].length - (points[i].cps * Length_slopes[i]);

        ESP_LOGD(TAG, "%f %f %f %f", Freq_slopes[i], Freq_offsets[i], Length_slopes[i], Length_offsets[i]);
    }

    Beep_window	       = 0;
    Current_climb_rate = 0;
    Current_freq  	   = 0;
	Tick 			   = 0;
}

static void play_interpolated_beep(int32_t cps){
    sound_profile_point_t *points = CONFIG.points;
    for (size_t i = 0; i < NUM_SOUND_PROFILE_POINTS-1; i++)
    {
        if(cps >= points[i].cps && cps <= points[i+1].cps){
            if(points[i].duty == 0){
                Current_freq = 0;
                Beep_window = points[i].length/TICK_MS;
                silence(points[i].length);
                return;
            }
            Current_freq = (uint16_t)(Freq_slopes[i] * cps + Freq_offsets[i]);
            Beep_window = (uint16_t)((Length_slopes[i] * cps + Length_offsets[i])/TICK_MS);
			uint32_t beep_duration  = (uint32_t)(Beep_window*points[i].duty/100.0);
            tone(Current_freq, beep_duration*TICK_MS);
            return;
        }
    }
    ESP_LOGE(TAG, "climb rate (%ld) is out of boundaries", cps);
}

static void vaudio_reset(int32_t climb_rate) {
	Current_climb_rate = climb_rate;
	Tick = 0;
	// if sinking significantly faster than glider sink rate in still air, generate warning sink tone
	if (Current_climb_rate <= CONFIG.offscale_low.cps) {
        Beep_window = 12;
        Current_freq = OffScaleLoTone[0];
        tone(Current_freq, 12*TICK_MS);
	}
    // if climbing really fast, generate warning sink tone
	else if(Current_climb_rate >= CONFIG.offscale_high.cps){
        Beep_window = 12;
        Current_freq = OffScaleHiTone[0];
        tone(Current_freq, 12*TICK_MS);
    }
    // regular climb rate, interpolate the sound profile to play the right beep
    else{
        play_interpolated_beep(climb_rate);
    }
}

// climb_rate in cm.s⁻¹
void vaudio_tick_handler(int32_t climb_rate) {
    // generate new beep/tone only if current beep/tone has ended, OR climb threshold exceeded
    if ((Beep_window <= 0)  || ((climb_rate >= CONFIG.climb_threshold.cps) && (Current_climb_rate <  CONFIG.climb_threshold.cps)) ) {
		vaudio_reset(climb_rate);
    }
    else { // still processing current beep/tone
		uint16_t newFreqHz = Current_freq;
        Tick++;
        Beep_window--;

        if (Current_climb_rate >= CONFIG.offscale_high.cps) { // offscale climbrate (>= +10m/s) indicated with continuous warbling tone
            newFreqHz = OffScaleHiTone[Tick];
        }
        else if (Current_climb_rate <= CONFIG.offscale_low.cps) {  // offscale sink (<= -10m/s) indicated with continuous descending tone
            newFreqHz = OffScaleLoTone[Tick];
        }

        if (newFreqHz != Current_freq) {
            Current_freq = newFreqHz;
            tone(Current_freq, TICK_MS);
        }
    }
}
