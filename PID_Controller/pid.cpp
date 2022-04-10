/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: 
 */
#include "pid.h"

PID::PID(double kp, double ki, double kd, unsigned long *nowTickTime, unsigned long *prevTickTime, double* setting, double* actual, double* u)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    nowTickTime_ = nowTickTime;
    prevTickTime_ = prevTickTime;
    setting_ = setting;
    actual_ = actual;
    u_ = u;
    e_ = 0;
    eIntegral_ = 0;
}

void PID::compute()
{
    deltaT_ = (((double)(*nowTickTime_ - *prevTickTime_)) / 1.0e6);
    e_ = *setting_ - *actual_;
    eIntegral_ = eIntegral_ + e_ * deltaT_;
    *u_ = kp_ * e_ + ki_ * eIntegral_;
}

// double PID::getT()
// {
//     return deltaT_;
// }
// double PID::getE()
// {
//     double E = *e_;
//     return E;
// }

// double PID::getU()
// {
//     double U = *u_;
//     return U;
// }