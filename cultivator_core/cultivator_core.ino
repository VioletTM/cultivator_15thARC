/*
引用URL：https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster.ino
*/

#include "cultivator_core_config.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for RC100 remote conroller
*******************************************************************************/
//RC100 remote_controller;
double const_cmd_vel    = 0.2;

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
CultivatorCoreMotorDriver motor_driver;

double linear_x              = 0.0;
double angular_z             = 0.0;
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

void setup()
{
  motor_driver.init();  //モータードライバー初期化

  // Setting for RC100 remote control and cmd_vel
  //remote_controller.begin(1);  //57600bps for RC100

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  // Start Dynamixel Control Interrupt
  startDynamixelControlInterrupt();
}

void loop()
{
  receiveRemoteControlData();
  controlMonster();
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlMonster);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Receive RC100 remote controller data
*******************************************************************************/
void receiveRemoteControlData(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if (received_data & RC100_BTN_U)        //加速
    {
      linear_x += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
      angular_z = 0.0;
    }
    else if (received_data & RC100_BTN_D)   //減速
    {
      linear_x -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
      angular_z = 0.0;
    }

    if (received_data & RC100_BTN_L)        //左旋回
    {
      linear_x = 0.0;
      angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if (received_data & RC100_BTN_R)   //右旋回
    {
      linear_x = 0.0;
      angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }

    if (received_data & RC100_BTN_1)
    {

    }
    else if (received_data & RC100_BTN_2)
    {

    }
    else if (received_data & RC100_BTN_3)
    {

    }
    else if (received_data & RC100_BTN_4)
    {

    }

    if (received_data & RC100_BTN_6)
    {
      linear_x = const_cmd_vel;
      angular_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)   //停止
    {
      linear_x = 0.0;
      angular_z = 0.0;
    }

    if (linear_x > MAX_LINEAR_VELOCITY)
    {
      linear_x = MAX_LINEAR_VELOCITY;
    }

    if (angular_z > MAX_ANGULAR_VELOCITY)
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }

    goal_linear_velocity = linear_x;
    goal_angular_velocity = angular_z;
  }
}

/*******************************************************************************
* Control bike speed
*******************************************************************************/
void controlMonster(void)
{
  bool control_result = false;

  double wheel1_spd_cmd, wheel2_spd_cmd, wheel3_spd_cmd, wheel4_spd_cmd;
  //double lin_vel1, lin_vel2, lin_vel3, lin_vel4;  //各タイヤの速度値
  int lin_vel1, lin_vel2, lin_vel3, lin_vel4;       //各タイヤの速度値(~240)

  double rotation_center;

  //左後輪速度コマンド
  wheel1_spd_cmd = goal_linear_velocity - (sqrt(WHEEL_POS_FROM_CENTER_X_1 * WHEEL_POS_FROM_CENTER_X_1 + WHEEL_POS_FROM_CENTER_Y_1 * WHEEL_POS_FROM_CENTER_Y_1) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_1 / WHEEL_POS_FROM_CENTER_X_1));
  //右後輪速度コマンド
  wheel2_spd_cmd = goal_linear_velocity + (sqrt(WHEEL_POS_FROM_CENTER_X_2 * WHEEL_POS_FROM_CENTER_X_2 + WHEEL_POS_FROM_CENTER_Y_2 * WHEEL_POS_FROM_CENTER_Y_2) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_2 / WHEEL_POS_FROM_CENTER_X_2));
  //左前輪速度コマンド
  wheel3_spd_cmd = goal_linear_velocity - (sqrt(WHEEL_POS_FROM_CENTER_X_3 * WHEEL_POS_FROM_CENTER_X_3 + WHEEL_POS_FROM_CENTER_Y_3 * WHEEL_POS_FROM_CENTER_Y_3) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_3 / WHEEL_POS_FROM_CENTER_X_3));
  //右前輪速度コマンド
  wheel4_spd_cmd = goal_linear_velocity + (sqrt(WHEEL_POS_FROM_CENTER_X_4 * WHEEL_POS_FROM_CENTER_X_4 + WHEEL_POS_FROM_CENTER_Y_4 * WHEEL_POS_FROM_CENTER_Y_4) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_4 / WHEEL_POS_FROM_CENTER_X_4));

  //左後輪速度値
  //lin_vel1 = wheel1_spd_cmd * VELOCITY_CONSTANT_VAULE;
  lin_vel1 = (int)(wheel1_spd_cmd * VELOCITY_CONSTANT_VAULE);
  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 = -LIMIT_X_MAX_VELOCITY;
  }
  //右後輪速度値
  //lin_vel2 = -1 * wheel2_spd_cmd * VELOCITY_CONSTANT_VAULE;
  lin_vel2 = (int)(wheel2_spd_cmd * VELOCITY_CONSTANT_VAULE);
  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 = -LIMIT_X_MAX_VELOCITY;
  }
  //左前輪速度値
  //lin_vel3 = wheel3_spd_cmd * VELOCITY_CONSTANT_VAULE;
  lin_vel3 = (int)(wheel3_spd_cmd * VELOCITY_CONSTANT_VAULE);
  if (lin_vel3 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel3 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel3 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel3 = -LIMIT_X_MAX_VELOCITY;
  }
  //右前輪速度値
  //lin_vel4 = -1 * wheel4_spd_cmd * VELOCITY_CONSTANT_VAULE;
  lin_vel4 = (int)(wheel4_spd_cmd * VELOCITY_CONSTANT_VAULE);
  if (lin_vel4 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel4 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel4 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel4 = -LIMIT_X_MAX_VELOCITY;
  }

  //lin_vel1 = left_rear_wheel_value
  //lin_vel2 = right_rear_wheel_value
  //lin_vel3 = left_front_wheel_value
  //lin_vel4 = right_front_wheel_value
  control_result = motor_driver.controlMotor(lin_vel1, lin_vel2, lin_vel3, lin_vel4);
  if (control_result == false)
  {
    return;
  }
}