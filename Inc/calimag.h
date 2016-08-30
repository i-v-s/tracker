#ifndef CALIMAG_H
#define CALIMAG_H

void startCalibrate();
void calibrate(const int16_t *mag);
void endCalibrate(float *m0, float *m1);

#endif // CALIMAG_H
