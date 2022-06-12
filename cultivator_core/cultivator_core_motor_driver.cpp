/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster_motor_driver.cpp
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#include "cultivator_core_motor_driver.h"

/* 初期化 */
bool CultivatorCoreMotorDriver::init(void)
{
    this->enableLED.setLEDOutput(this->motor_moving_pin);               //モーター動作LED
    this->rightFrontMotor.setMotorOutput(this->right_front_motor_pin);  //右前輪モーター
    this->leftFrontMotor.setMotorOutput(this->left_front_motor_pin);    //左前輪モーター
    this->rightRearMotor.setMotorOutput(this->right_rear_motor_pin);    //右後輪モーター
    this->leftRearMotor.setMotorOutput(this->left_rear_motor_pin);      //左後輪モーター
    return true;
}

/* 例外検出 */
bool CultivatorCoreMotorDriver::catchException(void)
{
    if (this->detectLostAt < runTime.milliseconds() && this->currentMotorL != 0 && this->currentMotorR != 0) {
        this->currentMotorL = 0;
        this->currentMotorR = 0;
        this->sleepMotors();
        return true;
    }
    return false;
}

/**/
void CultivatorCoreMotorDriver::updateLastCommunicatedAt(void)
{
    this->lastCommunicatedAt = millis();
    this->detectLostAt = this->lastCommunicatedAt + this->detect_lost_connection_ms;
}

/* ステアリング */
void CultivatorCoreMotorDriver::motorSteering(float linear_x, float angle_z)
{
    float RFWheelSpd, LFWheelSpd, RRWheelSpd, LRWheelSpd;
    RFWheelSpd = linear_x - (sqrt(pow(this->wheel_pos_from_center_x1, 2.0) + pow(this->wheel_pos_from_center_y1, 2.0) * angle_z) * cos(atan(this->wheel_pos_from_center_y1 / this->wheel_pos_from_center_x1)));
    LFWheelSpd = linear_x + (sqrt(pow(this->wheel_pos_from_center_x2, 2.0) + pow(this->wheel_pos_from_center_y2, 2.0) * angle_z) * cos(atan(this->wheel_pos_from_center_y2 / this->wheel_pos_from_center_x2)));
    RRWheelSpd = linear_x - (sqrt(pow(this->wheel_pos_from_center_x3, 2.0) + pow(this->wheel_pos_from_center_y3, 2.0) * angle_z) * cos(atan(this->wheel_pos_from_center_y3 / this->wheel_pos_from_center_x3)));
    LRWheelSpd = linear_x + (sqrt(pow(this->wheel_pos_from_center_x4, 2.0) + pow(this->wheel_pos_from_center_y4, 2.0) * angle_z) * cos(atan(this->wheel_pos_from_center_y4 / this->wheel_pos_from_center_x4)));

    RFWheelSpd = RFWheelSpd * this->velocity_constant_value;
    LFWheelSpd = LFWheelSpd * this->velocity_constant_value;
    RRWheelSpd = RRWheelSpd * this->velocity_constant_value;
    LRWheelSpd = LRWheelSpd * this->velocity_constant_value;

    this->setMotorsSpeed((int)RFWheelSpd, (int)LFWheelSpd, (int)RRWheelSpd, (int)LRWheelSpd);
}

/* 各モーター速度の設定 */
void CultivatorCoreMotorDriver::setMotorsSpeed(int right_front, int left_front, int right_rear, int left_rear)
{
    this->enableLED.LEDOn();    //モーター動作LED
    this->rightFrontMotor.setMotorSpeed(right_front);   //右前輪モーター
    this->leftFrontMotor.setMotorSpeed(left_front);     //左前輪モーター
    this->rightRearMotor.setMotorSpeed(right_rear);     //右後輪モーター
    this->leftRearMotor.setMotorSpeed(left_rear);       //左後輪モーター
}

/* 各モーター停止 */
void CultivatorCoreMotorDriver::sleepMotors(void)
{
    this->setMotorsSpeed(0, 0, 0, 0);   //モーター停止（速度0）
    this->enableLED.LEDOff();
}

/*******************************************************************************
* 以下、テスト用メソッド
*******************************************************************************/
/* LEDテスト */
void CultivatorCoreMotorDriver::testLED(void)
{
    this->enableLED.LEDflash(1000); // １秒感覚でLEDを点滅させる
}

/* サーボモーターテスト */
void CultivatorCoreMotorDriver::testServo(int angle)
{
    this->rightFrontMotor.setMotorAngle(angle);
    this->leftFrontMotor.setMotorAngle(angle);
    this->rightRearMotor.setMotorAngle(angle);
    this->leftRearMotor.setMotorAngle(angle);
}
