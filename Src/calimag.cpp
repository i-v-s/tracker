#include <stdint.h>
#include <ellipsoidcalibrator.h>

typedef Eigen::Quaternion<float> Quat;

//Quat q0;
bool active = false;

void uprintf( const char* format, ... );

EllipsoidCalibrator<float> ec;


void startCalibrate()
{
    ec.reset();
}

void calibrate(const int16_t *mag)
{
    if(!active)
    {
        active = true;
        uprintf("Mag calibration started\n");
        //q0 = quat;
    }
    ec.addValue(mag);
}

void endCalibrate(float *m0, float *m1)
{
    ec.solve(m0, m1);
}

