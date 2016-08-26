#include "copterimu.h"
#include "timefusion.h"
#include "stm32f4xx_hal.h"
#include "tracker.h"
#include <string>

typedef float Time;
typedef float Scalar;
typedef Eigen::Vector3f Vector3;
typedef ImuPoint<Time, Scalar>::Measure Measure;

typedef ImuSequence<Time, Scalar, 1024> Seq;

Measure m;
Seq seq;

#define af 0.1f
#define wf 0.1f
#define GTOL 200//5.0f
#define ATOL 200//f0.3f
#define MAXHIST 100

const float gRes = 250.0 / 32768.0 * M_PI / 180;

char *ftoa(char *fstr, float num);
void uprint(char c, const Vector3 &pos);
void uprintxy(char c, const Vector3 &pos);

uint32_t outTime = 0;
Vector3 pos;
#define POSFLT 0.1
#define OUTT 50
int outOn = 0;
//#define

int16_t oldAcc[3], oldGyr[3];//, oldMag[3];
int hist = 0;
bool stopState = false;
Seq::iterator it = nullptr;


void resetPos()
{
    pos.setZero();
    it = nullptr;
}

void processData(uint32_t time32, const int16_t *acc, const int16_t *gyr, const int16_t *mag)
{
    if(abs(acc[0] - oldAcc[0]) < ATOL && abs(acc[1] - oldAcc[1]) < ATOL && abs(acc[2] - oldAcc[2]) < ATOL
            && abs(gyr[0] - oldGyr[0]) < GTOL && abs(gyr[1] - oldGyr[1]) < GTOL && abs(gyr[2] - oldGyr[2]) < GTOL)
    {
        if(hist < MAXHIST)
        {
            hist += 5;
            if(hist >= MAXHIST)
            {
                stopState = true;
            }
        }
    }
    else if(hist > 0)
    {
        hist -= 3;
        if(hist <= 0)
        {
            stopState = false;
            // stop end;
        }
    }

    if(stopState)
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
    m.a[1] += ((acc[1] -  95) / 16400.0 * 9.8 - m.a[1]) * af;
    m.a[2] += ((acc[2] + 605) / 16535.0 * 9.8 - m.a[2]) * af;
    m.w[0] += ((gyr[0] + 38)  * gRes - m.w[0]) * wf;
    m.w[1] += ((gyr[1] - 91)  * gRes - m.w[1]) * wf;
    m.w[2] += ((gyr[2] + 146) * gRes - m.w[2]) * wf;

    static int stopCounter = 0;
    static Vector3 aSum, wSum;

    static Time beginTime;
    if(stopState) // Режим калибровки
    {
        if(stopCounter)
        {
            aSum += m.a;
            wSum += m.w;
        }
        else
        {
            // Постобработка
            if(it)
            {
                it->state.p -= it->state.v * (time - beginTime) * 0.5;
                //uprint('C', it->state.p);
            }

            // Начинаем новую калибровку
            aSum = m.a;
            wSum = m.w;
            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
        }
        stopCounter++;
    }
    else // Режим интегрирования
    {
        if(stopCounter || !it)
        {
            if(stopCounter)
            {
                // Завершаем калибровку
                aSum /= stopCounter;
                wSum /= stopCounter;
                stopCounter = 0;
                seq.setWBias(wSum);
                seq.setGravity(aSum.norm());
            }

            // Инициализируем интегрирование
            beginTime = time;
            Seq::iterator t = seq.add(time, m);
            t->from(&seq);
            Vector3 v0;
            v0 << 0, 0, 1;
            if(it && !std::isnan(it->state.p[0]))
                t->state.p = it->state.p;
            else
                t->state.p.setZero();
            t->state.q.setFromTwoVectors(aSum, v0);
            it = t;

            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
        }
        else
        {
            Seq::iterator t = seq.add(time, m);
            t->from(std::make_pair(it, &seq), &seq);
            it = t;
        }
        outOn = (int) (500 / OUTT);
    }
    if(it && /*outOn &&*/ time32 >= outTime)
    {
        if(std::isnan(it->state.p[0])) return;
        pos += (it->state.p - pos) * POSFLT;
        uprint('P', pos * 100);
        uprintxy('T', pos * 100);
        outTime = time32 + OUTT;
        outOn--;
    }
}

char *ftoa(char *fstr, float num)
{
    if(std::isnan(num))
    {
        *(fstr++) = '?';
        *(fstr) = '\0';
        return fstr;
    }
    if(num < 0) { *(fstr++) = '-'; num = -num; }
    char ibuf[16], *ip = ibuf + sizeof(ibuf);
    unsigned int i = (unsigned int) num;
    num -= i;
    if(!i)
        *(fstr++) = '0';
    else
    {
        do
        {
            *(--ip) = (i % 10) + '0';
            i /= 10;
        } while(i);
        while(ip < ibuf + sizeof(ibuf))
            *(fstr++) = *(ip++);
    }
    *(fstr++) = '.';
    if(num < 1E-6)
        *(fstr++) = '0';
    else for(int ctr = 6; num && ctr; ctr--)
    {
        num *= 10;
        unsigned int d = (unsigned int) num;
        num -= d;
        *(fstr++) = d + '0';
    }
    *(fstr) = '\0';
    return fstr;
}

void uprint(char c, const Vector3 &pos)
{
    char buf[80], *b = buf;
    *(b++) = '*';
    *(b++) = c;
    b = ftoa(b, pos[0]);
    *(b++) = ',';
    b = ftoa(b, pos[1]);
    *(b++) = ',';
    b = ftoa(b, pos[2]);
    *(b++) = '*';
    *(b++) = '\n';
    *b = 0;
    uprintf(buf);
}

void uprintxy(char c, const Vector3 &pos)
{
    char buf[80], *b = buf;
    *(b++) = '*';
    *(b++) = c;
    *(b++) = 'X';
    b = ftoa(b, pos[0]);
    *(b++) = 'Y';
    b = ftoa(b, pos[1]);
    *(b++) = '*';
    *(b++) = '\n';
    *b = 0;
    uprintf(buf);
}
