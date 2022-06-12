/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster.h
*******************************************************************************/

#ifndef CULTIVATOR_CORE_CONFIG_H_
#define CULTIVATOR_CORE_CONFIG_H_

#define USE_USBCON

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "cultivator_core_motor_driver.h"
#include "cultivator_core_cultivate_driver.h"

#define DEG2RAD(x)  (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)  (x * 57.2957795131)  // *180/PI

const int control_period = 8000;
const int motor_max_value = 1023;
const float max_linear_velocity = 0.22;     // m/s
const float max_angular_velocity = 2.84;    // rad/s
const float min_linear_velocity = -max_linear_velocity;
const float min_angular_velocity = -max_angular_velocity;
const float velocity_linear_x = 0.01;
const float velocity_angular_z = 0.1;
const int scale_velocity_linear_x = 1;
const int scale_velocity_angular_z = 1;

float motor_value_rate = 0.1;
float linear_goal_velocity = 0.0;
float angular_goal_velocity = 0.0;

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);  // 車輪駆動
void cultivateCallback(const std_msgs::UInt16& cmd_msg);                // 耕運作業命令

void testDCCallback(const std_msgs::Empty& toggle_msg);         // DCモーターテスト
void testLedCallback(const std_msgs::Empty& toggle_msg);        // LEDテスト
void testServoCallback(const std_msgs::UInt16& toggle_msg);     // サーボモーターテスト

// Publish function prototypes
void publishMotorState(void);   // 走行モーターの状態を通知
void publishCultivateFeedback(void);    // 耕運機構フィードバック

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", commandVelocityCallback);  // 車輪駆動
ros::Subscriber<std_msgs::UInt16> cultivate_sub("/cultivate", cultivateCallback);    // 耕運作業命令

ros::Subscriber<std_msgs::Empty> test_dc_sub("/test_dc", testDCCallback);            // DCモーターテスト
ros::Subscriber<std_msgs::Empty> test_led_sub("/test_led", testLedCallback);         // LEDテスト
ros::Subscriber<std_msgs::UInt16> test_servo_sub("/test_servo", testServoCallback);  // サーボモーターテスト

/*******************************************************************************
* Publisher
*******************************************************************************/
std_msgs::UInt16 motor_state_msg;
ros::Publisher motor_state_pub("/motor_state", &motor_state_msg);

std_msgs::UInt16 cultivate_fb_msg;
ros::Publisher cultivate_pub("/cultivate_fb", &cultivate_fb_msg);

//nav_msgs::Odometry odom;
//ros::Publisher odom_pub("/odom", &odom);

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
CultivatorCoreMotorDriver motor_driver;         // 走行機構クラス宣言
CultivatorCoreCultivateDriver cultivate_driver; // 耕耘機構クラス宣言

#endif
