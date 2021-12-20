/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster_motor_driver.h
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#ifndef CULTIVATOR_CORE_MOTOR_DRIVER_H_
#define CULTIVATOR_CORE_MOTOR_DRIVER_H_

#include "cultivator_core_arduino_setting.h"

#define DETECT_LOST_CONNECTION_MS 3000

#define MOTOR_MOVING            13  //モーター動作LED

#define RIGHT_FRONT_MOTOR_PWM   2   //右前輪スピードコントロール(PWM信号制御)
#define LEFT_FRONT_MOTOR_PWM    3   //左前輪スピードコントロール(PWM信号制御)
#define RIGHT_REAR_MOTOR_PWM    4   //右後輪スピードコントロール(PWM信号制御)
#define LEFT_REAR_MOTOR_PWM     5   //左後輪スピードコントロール(PWM信号制御)

/*
#define RIGHT_FRONT_MOTOR_PIN1  26  //右前輪Input1
#define RIGHT_FRONT_MOTOR_PIN2  27  //右前輪Input2
#define LEFT_FRONT_MOTOR_PIN1   28  //左前輪Input1
#define LEFT_FRONT_MOTOR_PIN2   29  //左前輪Input2
#define RIGHT_REAR_MOTOR_PIN1   30  //右後輪Input1
#define RIGHT_REAR_MOTOR_PIN2   31  //右後輪Input2
#define LEFT_REAR_MOTOR_PIN1    32  //左後輪Input1
#define LEFT_REAR_MOTOR_PIN2    33  //左後輪Input2
*/

#define WHEEL_RADIUS                    0.115     // meter
#define WHEEL_SEPARATION                0.16      // meter (BURGER => 0.16, WAFFLE => 0.287)
// #define ROBOT_LENGTH                    0.165     // meter

// #define WHEEL_POS_FROM_CENTER_X_1       -0.100    // meter
// #define WHEEL_POS_FROM_CENTER_Y_1       -0.128    // meter
// #define WHEEL_POS_FROM_CENTER_X_2       0.100     // meter
// #define WHEEL_POS_FROM_CENTER_Y_2       -0.128    // meter
// #define WHEEL_POS_FROM_CENTER_X_3       -0.100    // meter
// #define WHEEL_POS_FROM_CENTER_Y_3       0.128     // meter
// #define WHEEL_POS_FROM_CENTER_X_4       0.100     // meter
// #define WHEEL_POS_FROM_CENTER_Y_4       0.128     // meter

#define WHEEL_POS_FROM_CENTER_X_1       -0.369    // meter
#define WHEEL_POS_FROM_CENTER_Y_1       -0.218    // meter
#define WHEEL_POS_FROM_CENTER_X_2       0.369     // meter
#define WHEEL_POS_FROM_CENTER_Y_2       -0.218    // meter
#define WHEEL_POS_FROM_CENTER_X_3       -0.369    // meter
#define WHEEL_POS_FROM_CENTER_Y_3       0.218     // meter
#define WHEEL_POS_FROM_CENTER_X_4       0.369     // meter
#define WHEEL_POS_FROM_CENTER_Y_4       0.218     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define VELOCITY_CONSTANT_VAULE         1263.632956882  // V = r * w = r * RPM * 0.10472
                                                        //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                        // Goal RPM = V * 1263.632956882

#define LIMIT_X_MAX_VELOCITY    240

/* 走行機構クラス */
class CultivatorCoreMotorDriver
{
private:
    LEDSetting enableLED(MOTOR_MOVING);     //モーター動作LED
    /*
    DrivingMotor rightFrontMotor(RIGHT_FRONT_MOTOR_PIN1, RIGHT_FRONT_MOTOR_PIN2, RIGHT_FRONT_MOTOR_PWM);    // 右前輪モーター
    DrivingMotor leftFrontMotor(LEFT_FRONT_MOTOR_PIN1, LEFT_FRONT_MOTOR_PIN2, LEFT_FRONT_MOTOR_PWM);        // 左前輪モーター
    DrivingMotor rightRearMotor(RIGHT_REAR_MOTOR_PIN1, RIGHT_REAR_MOTOR_PIN2, RIGHT_REAR_MOTOR_PWM);        // 右後輪モーター
    DrivingMotor leftRearMotor(LEFT_REAR_MOTOR_PIN1, LEFT_REAR_MOTOR_PIN2, LEFT_REAR_MOTOR_PWM);            // 左後輪モーター
    */
    DrivingMotor rightFrontMotor(RIGHT_FRONT_MOTOR_PWM);    // 右前輪モーター
    DrivingMotor leftFrontMotor(LEFT_FRONT_MOTOR_PWM);      // 左前輪モーター
    DrivingMotor rightRearMotor(RIGHT_REAR_MOTOR_PWM);      // 右後輪モーター
    DrivingMotor leftRearMotor(LEFT_REAR_MOTOR_PWM);        // 左後輪モーター
    
    unsigned long lastCommunicatedAt = 0;
    unsigned long detectLostAt = 0;
    int currentMotorL = 0;
    int currentMotorR = 0;

public:
    CultivatorCoreMotorDriver(){}       // コンストラクタ
    ~CultivatorCoreMotorDriver(){}      // デストラクタ
    bool init(void);                    // 初期化
    bool catchException(void);          // 例外検出
    void updateLastCommunicatedAt(void);        // 
    void motorSteering(const float linear_x, const float angle_z, float speed); // ステアリング

private:
    void setMotorsSpeed(int left, int right);   // 各モーター速度の設定
    void sleepMotors(void);                     // 各モーター停止
};

#endif