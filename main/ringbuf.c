#include "ringbuf.h"

#include "esp_log.h"
#include "string.h"

#define TAG "ringbuf"

void ringbuf_init(ringbuf_t *ringbuf, uint8_t length) {
    ringbuf->buffer = (float*)malloc(sizeof(float)*length);
    if(ringbuf->buffer == NULL){
        ESP_LOGE(TAG, "failed to allocate ringbuf of length %u", length);
        abort();
    }
    ringbuf->len = length;
    ringbuf->head = ringbuf->len-1;
    memset(ringbuf->buffer,0,ringbuf->len*sizeof(ringbuf->buffer[0]));
}

void ringbuf_add_sample(ringbuf_t *ringbuf, float sample) {
   ringbuf->head++;
   if (ringbuf->head >= ringbuf->len) ringbuf->head = 0;
   ringbuf->buffer[ringbuf->head] = sample;
}

float ringbuf_average_oldest_samples(ringbuf_t *ringbuf, int numSamples) {
   int index = ringbuf->head+1; // oldest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index >= ringbuf->len) index = 0;
      accum += ringbuf->buffer[index];
      index++;
      }
   return accum/numSamples;
}

float ringbuf_average_newest_samples(ringbuf_t *ringbuf, int numSamples) {
   int index = ringbuf->head; // newest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index < 0) index = ringbuf->len - 1;
      accum += ringbuf->buffer[index];
      index--;
      }
   return accum/numSamples;
}
