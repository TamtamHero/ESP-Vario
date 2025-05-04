#pragma once

#include "stdint.h"

typedef struct {
    uint8_t len;
    uint8_t head;
    float *buffer;
} ringbuf_t;

void ringbuf_init(ringbuf_t *ringbuf, uint8_t length);
void ringbuf_add_sample(ringbuf_t *ringbuf, float sample);
float ringbuf_average_oldest_samples(ringbuf_t *ringbuf, int numSamples);
float ringbuf_average_newest_samples(ringbuf_t *ringbuf, int numSamples);
