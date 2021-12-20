#include "cultivator_core_cultivate_driver.h"

/* 初期化 */
bool CultivatorCoreCultivateDriver::init(void)
{
    cultivateMotor.setMotorOutput();    // 掘削モーター
    updownMotor.setMotorOutput();       // 上下モーター
    return true;
}

/* 例外検出 */
bool CultivatorCoreCultivateDriver::catchException(void)
{
    return false;
}

/* 機構を下げて耕耘を始める */
bool CultivatorCoreCultivateDriver::cultivateBegin(void)
{
    bool cultivateOnFlag = false;
    cultivateOnFlag = updownMotor.drop();   // 機構を下げる
    if ((cultivateOnFlag == true) && (updownMotor.dropping() == true)) {
        cultivateMotor.cultivateOn();       // 掘削モーターをON
    }
    // 機構を下限まで下げたら、上下モーター停止
    while (updownMotor.dropping() == true) {
        if (updownMotor.underLimit() == true) {
            break;
        }
    }
    updownMotor.stop(false);    // 非常停止ではないので、ブレーキを掛ける
    return true;
}

/* 機構を上げて耕耘を終える */
bool CultivatorCoreCultivateDriver::cultivateEnd(void)
{
    bool cultivateOffFlag = false;
    cultivateOffFlag = updownMotor.raise(); // 機構を上げる
    if ((cultivateOffFlag == true) && (updownMotor.raising() == true)) {
        cultivateMotor.cultivateOff();      // 掘削モーターをOFF
    }
    // 機構を上限まで上げたら、上下モーター停止
    while (updownMotor.raising() == true) {
        if (updownMotor.overLimit() == true) {
            break;
        }
    }
    updownMotor.stop(false);    // 非常停止ではないので、ブレーキを掛ける
    return true;
}
