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
  nh.subscribe(cultivate_sub);

  nh.subscribe(test_dc_sub);
  nh.subscribe(test_led_sub);
  nh.subscribe(test_servo_sub);

  nh.advertise(motor_state_pub);
  nh.advertise(cultivate_pub);
  //nh.advertise(odom_pub);
  
  motor_driver.init();
  cultivate_driver.init();
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

/* 車輪駆動 */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  motor_driver.updateLastCommunicatedAt();

  linear_goal_velocity = cmd_vel_msg.linear.x;
  angular_goal_velocity = cmd_vel_msg.angular.z;

  linear_goal_velocity = constrain(linear_goal_velocity, min_linear_velocity, max_linear_velocity);
  angular_goal_velocity = constrain(angular_goal_velocity, min_angular_velocity, max_angular_velocity);

  motor_driver.motorSteering(linear_goal_velocity, angular_goal_velocity);
}

/* 耕運作業命令 */
void cultivateCallback(const std_msgs::UInt16& cmd_msg)
{
  uint16_t cmd_num = cmd_msg.data;
  switch (cmd_num)
  {
  case static_cast<uint16_t>(cultivate_cmd::work_begin):
    cultivate_driver.cultivateBegin();
    break;
  case static_cast<uint16_t>(cultivate_cmd::work_end):
    cultivate_driver.cultivateEnd();
    break;
  default:
    break;
  }
}

/* 走行モーターの状態を通知 */
void publishMotorState(void)
{
  if (motor_driver.catchException() == true) {
    motor_state_msg.data = static_cast<uint16_t>(motor_state::exception);     // 例外検知
  } else {
    if (linear_goal_velocity == 0) {
      motor_state_msg.data = static_cast<uint16_t>(motor_state::stop);        // 停止
    } else {
      if (abs(angular_goal_velocity) < 0.1) {
        motor_state_msg.data = static_cast<uint16_t>(motor_state::straight);  // 直進
      } else {
        motor_state_msg.data = static_cast<uint16_t>(motor_state::curve);     // カーブ
      }
    }
  }
  motor_state_pub.publish(&motor_state_msg);
}

/* 耕運機構フィードバック */
void publishCultivateFeedback(void)
{
  cultivate_fb_msg.data = static_cast<uint16_t>(cultivate_driver.cultivateState());
  cultivate_pub.publish(&cultivate_fb_msg);
}

/*******************************************************************************
* Test Callbacks
*******************************************************************************/
/* DCモーターテスト */
void testDCCallback(const std_msgs::Empty& toggle_msg)
{
  cultivate_driver.testDCMotor();
}

/* LEDテスト */
void testLedCallback(const std_msgs::Empty& toggle_msg)
{
  motor_driver.testLED();
}

/* サーボモーターテスト */
void testServoCallback(const std_msgs::UInt16& toggle_msg)
{
  int servo_angle = static_cast<int>(toggle_msg.data);
  motor_driver.testServo(servo_angle);
}
