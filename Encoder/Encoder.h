/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: The library for the encoder of the motor from here https://www.dfrobot.com/product-1457.html
 */
#ifndef ENCODER_H
#define ENCODER_H
#include "Arduino.h"

#define ENC_STEPS 8
#define MOTER_REDUCTION 120

const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
const float pi = 3.14159265359;

class Encoder
{
    public:
        Encoder(const int ENC_A, const int ENC_B, unsigned long *nowTickTime, unsigned long *prevTickTime);
        int getEncVal();
        double getVelRad();
        double getVelRpm();
        double getPosRad();
        void sample();
        double getSampleInterval();
        int getNowCounter();
        int getPrevCounter();
        void tick(); 
    private:
        int counter_;
        int inc_; 
        int pinA_;
        int pinB_;
        int oldState_;
        unsigned long *nowTickTime_;
        unsigned long *prevTickTime_; 
        int prevCounter_;
        int nowCounter_;
        float deltaT_;

        double getRadPerCnt();
};

class LowPassFilter
{
    public:
        LowPassFilter(double coeff1, double coeff2, double coeff3, double* nowFilteredVal, double* prevRawVal, double* rawVal);
        double getFilterVal();
    private:
        double coeff1_;
        double coeff2_;
        double coeff3_;
        double* nowFilteredVal_;
        double* prevRawVal_;
        double* rawVal_;
};

#endif 
