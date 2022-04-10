/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: The library for the encoder of the motor from here https://www.dfrobot.com/product-1457.html
 */
#include <Encoder.h>


Encoder::Encoder(const int ENC_A, const int ENC_B, unsigned long *nowTickTime, unsigned long *prevTickTime)
{
    inc_ = 0;
    pinA_ = ENC_A;
    pinB_ = ENC_B;
    pinMode(ENC_B, INPUT);
    counter_ = 0;
    nowCounter_ = counter_;
    nowTickTime_ = nowTickTime;
    prevTickTime_ = prevTickTime;
}

int Encoder::getEncVal()
{
    return counter_;
}

double Encoder::getVelRad()
{
    deltaT_ = (((float)(*nowTickTime_ - *prevTickTime_)) / 1.0e6);
    return (nowCounter_ - prevCounter_) / deltaT_ * getRadPerCnt();
}

double Encoder::getVelRpm()
{
    // return (inc_ / deltaT_) / (8 * 120) * 60; 
    deltaT_ = (((float)(*nowTickTime_ - *prevTickTime_)) / 1.0e6);
    return (nowCounter_ - prevCounter_) / deltaT_ / (ENC_STEPS * MOTER_REDUCTION) * 60;
}

double Encoder::getPosRad()
{
    return (double)(counter_) * getRadPerCnt();
}

void Encoder::sample()
{
    prevCounter_ = nowCounter_;
    nowCounter_ = counter_;
}

double Encoder::getSampleInterval()
{
    deltaT_ = (((float)(*nowTickTime_ - *prevTickTime_)) / 1.0e6);
    return deltaT_;
}

int Encoder::getNowCounter()
{
    return nowCounter_;
}

int Encoder::getPrevCounter()
{
    return prevCounter_;
}

void Encoder::tick()
{
    int nowState = digitalRead(pinA_);

    if((oldState_ == LOW) && nowState == HIGH) // If triggered by rising edge 
    {
        inc_ = 0;
        int val = digitalRead(pinB_);
        if(val == LOW)
        {
            inc_ = -1;
        }
        else if(val == HIGH)
        {
            inc_ = 1;
        }

        counter_ += inc_;
        // Method 2: Compute speed by calculatin how much time elaspsed to trigger one pulse
        // Issue: When motor stop, cannot enter the interrupt, the motor cannot reach zero speed
        // nowTickTime_ = micros();
        // deltaT_ = ((float)(nowTickTime_ - prevTickTime_)) / 1.0e6; // unit: s
        // prevTickTime_ = nowTickTime_;
    }
    oldState_ = nowState;
}

double Encoder::getRadPerCnt()
{
    return 2 * pi / (ENC_STEPS * MOTER_REDUCTION);
}

LowPassFilter::LowPassFilter(double coeff1, double coeff2, double coeff3, double* nowFilteredVal, double* prevRawVal, double* rawVal)
{
    coeff1_ = coeff1;
    coeff2_ = coeff2;
    coeff3_ = coeff3;
    nowFilteredVal_ = nowFilteredVal;
    prevRawVal_ = prevRawVal; 
    rawVal_ = rawVal;
}

double LowPassFilter::getFilterVal()
{
    return  coeff1_ * (*nowFilteredVal_) + coeff2_ * (*rawVal_) + coeff3_ * (*prevRawVal_);
}



