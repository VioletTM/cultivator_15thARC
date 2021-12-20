/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_monster/turtlebot3_monster_motor_driver.cpp
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#include "cultivator_core_motor_driver.h"

/* 初期化 */
bool CultivatorCoreMotorDriver::init(void)
{
    enableLED.setLEDOutput();           //モーター動作LED
    rightFrontMotor.setMotorOutput();   //右前輪モーター
    leftFrontMotor.setMotorOutput();    //左前輪モーター
    rightRearMotor.setMotorOutput();    //右後輪モーター
    leftRearMotor.setMotorOutput();     //左後輪モーター
    return true;
}

/* 例外検出 */
bool CultivatorCoreMotorDriver::catchException(void)
{
    if (this->detectLostAt < mills() && this->currentMotorL != 0 && this->currentMotorR != 0) {
        this->currentMotorL = 0;
        this->currentMotorR = 0;
        this->sleepMotors();
        return true;
    }
    return false;
}


void CultivatorCoreMotorDriver::updateLastCommunicatedAt(void)
{
    this->lastCommunicatedAt = millis();
    this->detectLostAt = this->lastCommunicatedAt + (unsigned long)DETECT_LOST_CONNECTION_MS;
}

/* ステアリング */
void CultivatorCoreMotorDriver::motorSteering(const float linear_x, const float angle_z, float speed)
{
    int newMotorL = 0;
    int newMotorR = 0;

    // [plus/minus]SpeedIntの式は、デバッグで決める
    int plusSpeedInt = 1;
    int minusSpeedInt = -1;

    if (linear_x > 0.0 && angle_z == 0.0) {
        // 前進
        newMotorL = plusSpeedInt;
        newMotorR = plusSpeedInt;
    } else if (linear_x < 0.0 && angle_z == 0.0) {
        // 後退
        newMotorL = minusSpeedInt;
        newMotorR = minusSpeedInt;
    } else if (angle_z > 0.0) {
        if (linear_x == 0.0) {
            // 左旋回
            newMotorL = minusSpeedInt;
            newMotorR = plusSpeedInt;
        } else if (linear_x > 0.0) {
            // 左前進
            newMotorR = plusSpeedInt;
        } else {
            // 右後退
            newMotorL = minusSpeedInt;
        }
    } else if (angle_z < 0.0) {
        if (linear_x == 0.0) {
            // 右旋回
            newMotorL = plusSpeedInt;
            newMotorR = minusSpeedInt;
        } else if (linear_x > 0.0) {
            // 右前進
            newMotorL = plusSpeedInt;
        } else {
            // 左後退
            newMotorR = minusSpeedInt;
        }
    }
    if (this->currentMotorL == newMotorL && this->currentMotorR == newMotorR) {
        return;
    }
    if (newMotorL == 0 && newMotorR == 0) {
        this->sleepMotors();
    } else {
        this->setMotorsSpeed(newMotorL, newMotorR);
    }
    this->currentMotorL = newMotorL;
    this->currentMotorR = newMotorR;
}

/* 各モーター速度の設定 */
void CultivatorCoreMotorDriver::setMotorsSpeed(int left, int right)
{
    enableLED.LEDOn();                      //モーター動作LED
    rightFrontMotor.setMotorSpeed(right);   //右前輪モーター
    leftFrontMotor.setMotorSpeed(left);     //左前輪モーター
    rightRearMotor.setMotorSpeed(right);    //右後輪モーター
    leftRearMotor.setMotorSpeed(left);      //左後輪モーター
}

/* 各モーター停止 */
void CultivatorCoreMotorDriver::sleepMotors(void)
{
    this->setMotorsSpeed(0, 0);     //モーター停止（速度0）
    enableLED.LEDOff();
}
