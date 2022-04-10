#include <dc_motor.h>
#include <pid.h>
#include <Encoder.h>
#include<motorControl.h>

#define nullptr NULL
#define PIN_A_L 2
#define PIN_B_L 11
#define PIN_A_R 3
#define PIN_B_R 4
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define ENA 10
#define ENB 5

// RPM
//double wSetR = 10;
//double wSetL = 10;
// Rad/s
double wSetR = 3.0 / 60 * 2 * pi;
double wSetL = 3.0 / 60 * 2 * pi;

MotorControl* RM_ptr = nullptr;
MotorControl* LM_ptr = nullptr;
double kp;
double ki;
double kd;

void callbackL()
{
  LM_ptr->callback();
}

void callbackR()
{
  RM_ptr->callback();
}

void setup() 
{
  Serial.begin(57600);//Initialize the serial port
  kp = 5;
  ki = 10;
  kd = 0;
  RM_ptr = new MotorControl( VEL_UNIT::RAD,
        MOTOR_NAME::RIGHT,
        PIN_A_R, PIN_B_R, 
        IN1, IN2, ENA,
        kp, ki, kd, &wSetR);
  LM_ptr = new MotorControl( VEL_UNIT::RAD,
        MOTOR_NAME::LEFT,
        PIN_A_L, PIN_B_L, 
        IN3, IN4, ENB,
        kp, ki, kd, &wSetL);
  attachInterrupt(digitalPinToInterrupt(PIN_A_L), callbackL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_A_R), callbackR, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  RM_ptr->tick();
  LM_ptr->tick();
  Serial.print(100);
  Serial.print(" ");
  Serial.print(-100);
  Serial.print(" ");
  Serial.print(wSetR);
  Serial.print(" ");
  Serial.print(RM_ptr->getVel());
  Serial.print(" ");
  Serial.print(wSetL);
  Serial.print(" ");
  Serial.println(LM_ptr->getVel());
//  Serial.println(LM_ptr->getPos());
//  Serial.println(RM_ptr->getPos());
  delay(1);
  
}
