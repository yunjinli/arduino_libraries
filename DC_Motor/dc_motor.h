/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: 
 */
#ifndef DC_MOTOR_H
#define DC_MOTOR_H
#include "Arduino.h"

enum CMD
{
    CW,
    CCW
};

class DC_Motor
{
    public:
        DC_Motor(const int IN1, const int IN2, const int EN);
        void setCmd(CMD cmd);
        void setMotorPwm(CMD cmd, int pwm);
    private:
        int in1_;
        int in2_;
        int en_;

        
};

#endif 
