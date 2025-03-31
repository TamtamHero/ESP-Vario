#pragma once
#include "stdbool.h"

void kalmanFilter4_configure(float zSensorVariance, float aVariance, bool bAdaptUpdateVariance, float zInitial, float vInitial, float aInitial);
void kalmanFilter4_predict(float dt);
void kalmanFilter4_update(float zm, float am, float* pz, float* pv);


