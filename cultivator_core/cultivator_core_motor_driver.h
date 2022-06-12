/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster_motor_driver.h
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#ifndef CULTIVATOR_CORE_MOTOR_DRIVER_H_
#define CULTIVATOR_CORE_MOTOR_DRIVER_H_

#include "cultivator_core_arduino_setting.h"

#include <cmath>

#define ENCODER_MIN  -2147483648     // raw
#define ENCODER_MAX  2147483648      // raw

const double robot_length = 0.436;      // meter
const double wheel_separation = 0.738;  // meter
const double wheel_radius = 0.115;      // meter

enum class motor_state {
    exception,
    stop,
    straight,
    curve
};

/* 走行機構クラス */
class CultivatorCoreMotorDriver
{
private:
    const int right_front_motor_pin = 2;
    const int left_front_motor_pin = 3;
    const int right_rear_motor_pin = 4;
    const int left_rear_motor_pin = 5;
    const int motor_moving_pin = 13;

    const double wheel_pos_from_center_x1 = -(wheel_separation / 2);
    const double wheel_pos_from_center_y1 = -(robot_length / 2);
    const double wheel_pos_from_center_x2 = (wheel_separation / 2);
    const double wheel_pos_from_center_y2 = -(robot_length / 2);
    const double wheel_pos_from_center_x3 = -(wheel_separation / 2);
    const double wheel_pos_from_center_y3 = (robot_length / 2);
    const double wheel_pos_from_center_x4 = (wheel_separation / 2);
    const double wheel_pos_from_center_y4 = (robot_length / 2);

    LEDSetting enableLED;   //モーター動作LED
    TimeSetting runTime;    // 経過時間
    DrivingMotor rightFrontMotor;   // 右前輪モーター
    DrivingMotor leftFrontMotor;    // 左前輪モーター
    DrivingMotor rightRearMotor;    // 右後輪モーター
    DrivingMotor leftRearMotor;     // 左後輪モーター

    const unsigned long detect_lost_connection_ms = 3000;
    const double velocity_constant_value = 1263.632956882;  // V = r * w = r * RPM * 0.10472
                                                            //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                            // Goal RPM = V * 1263.632956882
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
    void motorSteering(float linear_x, float angle_z);  // ステアリング
    /* テスト用メソッド */
    void testLED(void);
    void testServo(int angle);
    /* テスト用メソッド */

private:
    void setMotorsSpeed(int right_front, int left_front, int right_rear, int left_rear);    // 各モーター速度の設定
    void sleepMotors(void);                     // 各モーター停止
};

#endif
