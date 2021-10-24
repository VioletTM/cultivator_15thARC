/*
引用URL：https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster_motor_driver.h
*/

#ifndef CULTIVATOR_CORE_MOTOR_DRIVER_H_
#define CULTIVATOR_CORE_MOTOR_DRIVER_H_

#include "Arduino.h"

#define RIGHT_FRONT_MOTOR_PIN   2   //右前輪スピードコントロール
#define LEFT_FRONT_MOTOR_PIN    3   //左前輪スピードコントロール
#define RIGHT_REAR_MOTOR_PIN    4   //右後輪スピードコントロール
#define LEFT_REAR_MOTOR_PIN     5   //左後輪スピードコントロール

#define CULTIVATE_MOTOR_1_PIN   6
#define CULTIVATE_MOTOR_2_PIN   7
#define CULTIVATE_LIFT_UP_PIN   8
#define CULTIVATE_LIFT_DOWN_PIN 9

#define LIMIT_X_MAX_VELOCITY    240

class CultivatorCoreMotorDriver
{
    public:
        CultivatorCoreMotorDriver();
        ~CultivatorCoreMotorDriver();
        bool init(void);
        void closeMotor(void);
        bool setTorque(uint8_t id, bool onoff);
        bool controlMotor(int left_rear_wheel_value, int right_rear_wheel_value, int left_front_wheel_value, int right_front_wheel_value);
    
    private:
        uint8_t left_rear_wheel_id_, right_rear_wheel_id_;
        uint8_t left_front_wheel_id_, right_front_wheel_id_;
}

#endif