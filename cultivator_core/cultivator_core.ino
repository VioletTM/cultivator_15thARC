/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster.ino
https://yoraish.com/2021/09/08/a-full-autonomous-stack-a-tutorial-ros-raspberry-pi-arduino-slam/
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#include "cultivator_core_config.h"

void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  
  motor_driver.init();
  cultivate_driver.init();
}

void loop()
{
  nh.spinOnce();
  motor_driver.catchException();
  cultivate_driver.catchException();
  delay(1);
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  motor_driver.updateLastCommunicatedAt();

  const float linear_x = cmd_vel_msg.linear.x;
  const float angle_z = cmd_vel_msg.angular.z;
  float speed = MOTOR_MAX_VALUE * motor_value_rate;

  motor_driver.motorSteering(linear_x, angle_z, speed);
}
