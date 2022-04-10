/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: 
 */
#include "dc_motor.h"

DC_Motor::DC_Motor(const int IN1, const int IN2, const int EN)
{
    in1_ = IN1;
    in2_ = IN2;
    en_ = EN;
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    pinMode(en_, OUTPUT);
}

void DC_Motor::setCmd(CMD cmd)
{
    switch(cmd)
    {
        case CMD::CW:
            digitalWrite(in1_, HIGH);
            digitalWrite(in2_, LOW);
            break;
        case CMD::CCW:
            digitalWrite(in1_, LOW);
            digitalWrite(in2_, HIGH);
            break;
        default:
            // do nothing
            break;
    }
}
void DC_Motor::setMotorPwm(CMD cmd, int pwm)
{
    if(pwm < 0)
    {
        pwm = 0;
    }
    if(pwm > 255)
    {
        pwm = 255;
    }
    setCmd(cmd);
    analogWrite(en_, pwm);
}

