/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: 
 */
#include "motorControl.h"

MotorControl::MotorControl( VEL_UNIT vel_unit,
        MOTOR_NAME name,
        const int ENC_A, const int ENC_B, 
        const int IN1, const int IN2, const int EN,
        double kp, double ki, double kd, double* setting)
{
    name_ = name;
    wAct_ = 0;
    wFiltered_ = 0;
    wActPrev_ = 0;
    u_ = 0;
    wSetPtr_ = setting;
    encoderPtr_ = new Encoder(ENC_A, ENC_B, &nowTickTime_, &prevTickTime_);
    motorPtr_ = new DC_Motor(IN1, IN2, EN);
    lpPtr_ = new LowPassFilter(0.854, 0.0728, 0.0728, &wFiltered_, &wActPrev_, &wAct_);
    // pidPtr_ = new PID(kp, ki, kd, &nowTickTime_, &prevTickTime_, wSetPtr_, &wFiltered_, &u_);
    vel_unit_ = vel_unit;
    if(vel_unit_ == VEL_UNIT::RAD)
    {
        pidPtr_ = new PID(kp * 60 / (2 * pi), ki * 60 / (2 * pi), kd * 60 / (2 * pi), &nowTickTime_, &prevTickTime_, wSetPtr_, &wFiltered_, &u_);
        // pidPtr_ = new PID(kp, ki, kd, &nowTickTime_, &prevTickTime_, wSetPtr_, &wFiltered_, &u_);
    }
    else
    {
        pidPtr_ = new PID(kp, ki, kd, &nowTickTime_, &prevTickTime_, wSetPtr_, &wFiltered_, &u_);
    }
    
    // encoder_(ENC_A, ENC_B, &nowTickTime_, &prevTickTime_);
    // motor_(IN1, IN2, EN);
    // lp_(0.854, 0.0728, 0.0728, &wFiltered_, &wActPrev_, &wAct_);
    // pid_(kp, ki, kd, &nowTickTime_, &prevTickTime_, wSetPtr_, &wFiltered_, &u_);
}

void MotorControl::tick()
{
    prevTickTime_ = nowTickTime_;
    nowTickTime_ = micros();
    encoderPtr_->sample();
    // encoder_.sample();
    if(vel_unit_ == VEL_UNIT::RPM)
    {
        wAct_ = encoderPtr_->getVelRpm();
    }
    else
    {
        wAct_ = encoderPtr_->getVelRad();
    }
    
    // wAct_ = encoder_.getVelRpm();
    if(name_ == MOTOR_NAME::LEFT)
    {
        wAct_ *= -1;
    }
    wFiltered_ = lpPtr_->getFilterVal();
    
    // wFiltered_ = lp_.getFilterVal();
    wActPrev_ = wAct_;
    pidPtr_->compute();
    // pid_.compute();
    if(u_ > 0)
    {
        motorPtr_->setMotorPwm(CMD::CCW, (int)fabs(u_));
        // motor_.setMotorPwm(CMD::CCW, (int)fabs(u_));
    }
    else
    {
        u_ = u_ * (-1);
        motorPtr_->setMotorPwm(CMD::CW, (int)fabs(u_));
        // motor_.setMotorPwm(CMD::CW, (int)fabs(u_));
    }
    // switch(name_)
    // {
    //     case MOTOR_NAME::RIGHT:
    //         if(u_ > 0)
    //         {
    //             motorPtr_->setMotorPwm(CMD::CCW, (int)fabs(u_));
    //             // motor_.setMotorPwm(CMD::CCW, (int)fabs(u_));
    //         }
    //         else
    //         {
    //             u_ = u_ * (-1);
    //             motorPtr_->setMotorPwm(CMD::CW, (int)fabs(u_));
    //             // motor_.setMotorPwm(CMD::CW, (int)fabs(u_));
    //         }
    //     break;
    //     case MOTOR_NAME::LEFT:
    //         if(u_ > 0)
    //         {
    //             motorPtr_->setMotorPwm(CMD::CCW, (int)fabs(u_));
    //             // motor_.setMotorPwm(CMD::CW, (int)fabs(u_));
    //         }
    //         else
    //         {
    //             u_ = u_ * (-1);
    //             motorPtr_->setMotorPwm(CMD::CW, (int)fabs(u_));
    //             // motor_.setMotorPwm(CMD::CCW, (int)fabs(u_));
    //         }
    //     break;
    //     default:
    //         // Do nothing
    //     break;
    // }
}

void MotorControl::callback()
{
    encoderPtr_->tick();
    // encoder_.tick();
}

// double MotorControl::getVelRpm()
// {
//     return wFiltered_;
// }

double MotorControl::getVel()
{
    return wFiltered_;
}

double MotorControl::getPos()
{
    return encoderPtr_->getPosRad();
}