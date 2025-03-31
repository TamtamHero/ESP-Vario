#pragma once

#include "hal/adc_types.h"

void adc_init(adc_channel_t channel);
float adc_read_battery_voltage(adc_channel_t channel);