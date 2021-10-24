/*
引用URL：https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster_motor_driver.cpp
*/

#include "cultivator_core_motor_driver.h"

CultivatorCoreMotorDriver::CultivatorCoreMotorDriver()
{}

CultivatorCoreMotorDriver::~CultivatorCoreMotorDriver()
{}

bool CultivatorCoreMotorDriver::init(void)
{
    pinMode(RIGHT_FRONT_MOTOR_PIN, OUTPUT);     //右前輪モーター
    pinMode(LEFT_FRONT_MOTOR_PIN, OUTPUT);      //左前輪モーター
    pinMode(RIGHT_REAR_MOTOR_PIN, OUTPUT);      //右後輪モーター
    pinMode(LEFT_REAR_MOTOR_PIN, OUTPUT);       //左後輪モーター

    /* 停止 */
    analogWrite(RIGHT_FRONT_MOTOR_PIN, 0);
    analogWrite(LEFT_FRONT_MOTOR_PIN, 0);
    analogWrite(RIGHT_REAR_MOTOR_PIN, 0);
    analogWrite(LEFT_REAR_MOTOR_PIN, 0);
    digitalWrite(RIGHT_FRONT_MOTOR_PIN, LOW);
    digitalWrite(LEFT_FRONT_MOTOR_PIN, LOW);
    digitalWrite(RIGHT_REAR_MOTOR_PIN, LOW);
    digitalWrite(LEFT_REAR_MOTOR_PIN, LOW);

    // Enable Motor Torque
    setTorque(right_front_wheel_id_, true);
    setTorque(left_front_wheel_id_, true);
    setTorque(right_rear_wheel_id_, true);
    setTorque(left_rear_wheel_id_, true);

    return true;
}

void CultivatorCoreMotorDriver::closeMotor(void)
{
    /* 停止 */
    analogWrite(RIGHT_FRONT_MOTOR_PIN, 0);
    analogWrite(LEFT_FRONT_MOTOR_PIN, 0);
    analogWrite(RIGHT_REAR_MOTOR_PIN, 0);
    analogWrite(LEFT_REAR_MOTOR_PIN, 0);
    digitalWrite(RIGHT_FRONT_MOTOR_PIN, LOW);
    digitalWrite(LEFT_FRONT_MOTOR_PIN, LOW);
    digitalWrite(RIGHT_REAR_MOTOR_PIN, LOW);
    digitalWrite(LEFT_REAR_MOTOR_PIN, LOW);

    // Disable Motor Torque
    setTorque(right_front_wheel_id_, false);
    setTorque(left_front_wheel_id_, false);
    setTorque(right_rear_wheel_id_, false);
    setTorque(left_rear_wheel_id_, false);
}

bool CultivatorCoreMotorDriver::setTorque(uint8_t id, bool onoff)
{}

bool CultivatorCoreMotorDriver::controlMotor(int left_rear_wheel_value, int right_rear_wheel_value, int left_front_wheel_value, int right_front_wheel_value)
{
    analogWrite(RIGHT_FRONT_MOTOR_PIN, right_front_wheel_value);
    analogWrite(LEFT_FRONT_MOTOR_PIN, left_front_wheel_value);
    analogWrite(RIGHT_REAR_MOTOR_PIN, right_rear_wheel_value);
    analogWrite(LEFT_REAR_MOTOR_PIN, left_rear_wheel_value);

    return true;
}
