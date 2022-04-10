/*
 * Author: Yun-Jin Li
 * Website: https://github.com/yunjinli
 * Website: https://www.notion.so/Yun-Jin-Li-Jim-afcdbb109e2b4a76b781bc691ff6037a
 * Description: 
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "Arduino.h"
#include "../PID_Controller/pid.h"
#include "../Encoder/Encoder.h"
#include "../DC_Motor/dc_motor.h"


enum VEL_UNIT
{
    RAD,
    RPM
};


enum MOTOR_NAME
{
    LEFT,
    RIGHT
};

class MotorControl
{
    public:
        MotorControl( VEL_UNIT vel_unit,
        MOTOR_NAME name,
        const int ENC_A, const int ENC_B, 
        const int IN1, const int IN2, const int EN,
        double kp, double ki, double kd, double* setting);
        void tick();
        void callback();
        // double getVelRpm();
        double getVel();
        double getPos();
    private:
        Encoder* encoderPtr_;
        DC_Motor* motorPtr_;
        PID* pidPtr_;
        LowPassFilter* lpPtr_;
        // Encoder encoder_;
        // DC_Motor motor_;
        // PID pid_;
        // LowPassFilter lp_;

        MOTOR_NAME name_;

        unsigned long nowTickTime_;
        unsigned long prevTickTime_;

        double* wSetPtr_;
        double wAct_;
        double wFiltered_;
        double wActPrev_;
        double u_;
        VEL_UNIT vel_unit_;

};

#endif 
