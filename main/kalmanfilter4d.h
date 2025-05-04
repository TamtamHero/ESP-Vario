#pragma once

void kalmanFilter4d_configure(float aVariance, float kAdapt, float zInitial, float vInitial, float aInitial);
void kalmanFilter4d_predict(float dt);
void kalmanFilter4d_update(float zm, float am, float* pz, float* pv);


