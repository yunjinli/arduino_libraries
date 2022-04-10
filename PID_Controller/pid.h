/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: 
 */

#ifndef PID_H
#define PID_H
#include "Arduino.h"

class PID
{
    public:
        PID(double kp, double ki, double kd, unsigned long *nowTickTime, unsigned long *prevTickTime, double* setting, double* actual, double* u);
        void compute();
        // for debug
        // double getT();
        // double getE();
        // double getU();
    private:
        double kp_;
        double ki_;
        double kd_;
        unsigned long *nowTickTime_;
        unsigned long *prevTickTime_; 
        double* setting_;
        double* actual_;
        double* u_;
        double deltaT_;
        double e_;
        double eIntegral_;
               
};

#endif 