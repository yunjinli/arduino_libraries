#include <dc_motor.h>
#include <pid.h>
#include <Encoder.h>
#include<motorControl.h>
#include <ros.h>
//#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/JointState.h>
//#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <robot_model/wheelInfo.h>

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

double wSetR = 0;
double wSetL = 0;
double omega = 0;
MotorControl* RM_ptr = nullptr;
MotorControl* LM_ptr = nullptr;
double kp;
double ki;
double kd;

ros::NodeHandle  nh;

//void callback( const geometry_msgs::Twist& vel_cmd)
//{
//  omega = vel_cmd.linear.x;
//  wSetR = omega;
//  wSetL = omega;
//}
void lw_callback( const std_msgs::Float64& cmd)
{
  wSetL = cmd.data;
}
void rw_callback( const std_msgs::Float64& cmd)
{
  wSetR = cmd.data;
}

//ros::Subscriber<geometry_msgs::Twist> sub("vel_cmd", &callback);
ros::Subscriber<std_msgs::Float64> lw_cmd_sub("left_wheel_cmd", &lw_callback);
ros::Subscriber<std_msgs::Float64> rw_cmd_sub("right_wheel_cmd", &rw_callback);

/*Define the publushed message*/
robot_model::wheelInfo wheelState;
ros::Publisher wheelStatePublisher("wheel_state", &wheelState);
/*Define the published message*/

void callbackL()
{
  LM_ptr->callback();
}

void callbackR()
{
  RM_ptr->callback();
}

void initPubMsg()
{
  wheelState.lwPos = 0;
  wheelState.rwPos = 0;
  wheelState.lwVel = 0;
  wheelState.rwVel = 0;
}
void setup() 
{
  Serial.begin(57600);//Initialize the serial port
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(lw_cmd_sub);
  nh.subscribe(rw_cmd_sub);
  initPubMsg();
  nh.advertise(wheelStatePublisher);  
  
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

  wheelState.lwPos = -LM_ptr->getPos();
  wheelState.lwVel = LM_ptr->getVel();
  wheelState.rwPos = RM_ptr->getPos();
  wheelState.rwVel = RM_ptr->getVel();

  wheelStatePublisher.publish(&wheelState);
  nh.spinOnce();
  delay(1);
}
