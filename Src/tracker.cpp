#include "copterimu.h"
#include "timefusion.h"

typedef float Time;
typedef float Scalar;

typedef ImuPoint<Time, Scalar>::Measure Measure;

typedef ImuSequence<Time, Scalar, 1024> Seq;

Measure m;
Seq seq;

#define af 0.1f
#define wf 0.1f
#define GTOL 200//5.0f
#define ATOL 200//f0.3f

const float gRes = 250.0 / 32768.0 * M_PI / 180;

int16_t oldAcc[3], oldGyr[3], oldMag[3];

void processData(int32_t time32, const int16_t *acc, const int16_t *gyr, const uint16_t *mag)
{
    if(abs(acc[0] - oldAcc[0]) < ATOL && abs(acc[1] - oldAcc[1]) < ATOL && abs(acc[2] - oldAcc[2]) < ATOL
            && abs(gyr[0] - oldGyr[0]) < GTOL && abs(gyr[1] - oldGyr[1]) < GTOL && abs(gyr[2] - oldGyr[2]) < GTOL)
    {
        oldAcc[0] += (acc[0] - oldAcc[0]) / 8;
        oldAcc[1] += (acc[1] - oldAcc[1]) / 8;
        oldAcc[2] += (acc[2] - oldAcc[2]) / 8;
        oldGyr[0] += (gyr[0] - oldGyr[0]) / 8;
        oldGyr[1] += (gyr[1] - oldGyr[1]) / 8;
        oldGyr[2] += (gyr[2] - oldGyr[2]) / 8;
    }
    else
    {
        oldAcc[0] = acc[0];
        oldAcc[1] = acc[1];
        oldAcc[2] = acc[2];
        oldGyr[0] = gyr[0];
        oldGyr[1] = gyr[1];
        oldGyr[2] = gyr[2];
    }

    Time time = time32 * 0.001;
    m.a[0] += ((acc[0] - 180) / 16400.0 * 9.8 - m.a[0]) * af;
    m.a[1] += ((acc[1] - 95) / 16400.0 * 9.8 - m.a[1]) * af;
    m.a[2] += ((acc[2] + 605) / 16535.0 * 9.8 - m.a[2]) * af;
    m.w[0] += ((gyr[0] + 38)  * gRes - m.w[0]) * wf;
    m.w[1] += ((gyr[1] - 91)  * gRes - m.w[1]) * wf;
    m.w[2] += ((gyr[2] + 146) * gRes - m.w[2]) * wf;

    if(time > t1b && time < t1e) // Калибровка 1
    {
        a1 += m.a;
        w1 += m.w;
        c1++;
    }
    else if(c1)
    {
        a1 /= c1;
        w1 /= c1;
        c1 = 0;
        seq.setWBias(w1);
        l1 = a1.norm();
        seq.setGravity(l1);

    }
    if(time > t1e && time < t2b) // Интегрируем
    {
        if(to) wi += m.w * (time - to);
        to = time;
        Seq::iterator t = seq.add(time, m);
        if(it)
            t->from(std::make_pair(it, &seq), &seq);
        else
        {
            t->from(&seq);
            Eigen::Vector3d v0;
            v0 << 0, 0, 1;
            t->state.q.setFromTwoVectors(a1, v0);
        }
        it = t;
    }
    if(time > t2b && time < t2e) // Калибровка 2
    {
        a2 += m.a;
        w2 += m.w;
        c2++;
    }
    else if(c2)
    {
        a2 /= c2;
        w2 /= c2;
        l2 = a2.norm();
        c2 = 0;
    }
}
v2 = it->state.v;
p2 = it->state.p;
Time dt = 11.85 - 10.33;
p2 -= v2 * dt * 0.5;
// Пересчёт
Eigen::Vector3d v0;
v0 << 0, 0, l2;
Eigen::Vector3d vt = it->state.q.inverse()._transformVector(v0);


}
