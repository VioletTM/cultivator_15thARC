/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster.h
*******************************************************************************/

#ifndef CULTIVATOR_CORE_CONFIG_H_
#define CULTIVATOR_CORE_CONFIG_H_

#define USE_USBCON

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "cultivator_core_motor_driver.h"
#include "cultivator_core_cultivate_driver.h"

#define CONTROL_PERIOD                  8000

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define MOTOR_MAX_VALUE 1023

float motor_value_rate = 0.1;

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

// Publish function prototypes


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/


/*******************************************************************************
* Declaration for motor
*******************************************************************************/
CultivatorCoreMotorDriver motor_driver;         // 走行機構クラス宣言
CultivatorCoreCultivateDriver cultivate_driver; // 耕耘機構クラス宣言

#endif