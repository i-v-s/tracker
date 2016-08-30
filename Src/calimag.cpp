#include <stdint.h>
#include <ellipsoidcalibrator.h>

typedef Eigen::Quaternion<float> Quat;

//Quat q0;
bool active = false;

void uprintf( const char* format, ... );

EllipsoidCalibrator<double> ec;


void startCalibrate()
{
    ec.reset();
}

int16_t magMin[3] = {}, magMax[3] = {};

void calibrate(const int16_t *mag)
{
    if(!active)
    {
        active = true;
        uprintf("Mag calibration started\n");
        //q0 = quat;
    }
    ec.addValue(mag);
    for(int x = 0; x < 3; x++)
    {
        if(magMin[x] > mag[x]) magMin[x] = mag[x];
        if(magMax[x] < mag[x]) magMax[x] = mag[x];
    }
}

void endCalibrate(float *m0, float *m1)
{
    ec.solve(m0, m1);
}

