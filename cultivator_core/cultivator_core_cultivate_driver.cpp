#include "cultivator_core_cultivate_driver.h"

/* 初期化 */
void CultivatorCoreCultivateDriver::init(void)
{
    this->cultivateMotor.setMotorOutput(this->cultivate_pin_1, this->cultivate_pin_2, this->cultivate_pwm_pin); // 掘削モーター
    this->updownMotor.setMotorOutput(this->lift_up_pin, this->lift_pwm_pin, this->lift_pwm_pin);    // 上下モーター
    this->updownMotor.setSensorInput(this->over_limit_pin, this->under_limit_pin, this->up_pin, this->down_pin);
}

/* 例外検出 */
bool CultivatorCoreCultivateDriver::catchException(void)
{
    // 上昇及び下降がともにONの場合
    if ((this->updownMotor.raising() == true) && (this->updownMotor.dropping() == true)) {
        return true;
    }
    // 上限及び下限がともにONの場合
    if ((this->updownMotor.overLimit() == true) && (this->updownMotor.underLimit() == true)) {
        return true;
    }
    // 上昇及び下限がともにONの場合
    if ((this->updownMotor.raising() == true) && (this->updownMotor.underLimit() == true)) {
        return true;
    }
    // 下降及び上限がともにONの場合
    if ((this->updownMotor.dropping() == true) && (this->updownMotor.overLimit() == true)) {
        return true;
    }
    return false;
}

/* 機構を下げて耕耘を始める */
void CultivatorCoreCultivateDriver::cultivateBegin(void)
{
    bool cultivateOnFlag = false;
    cultivateOnFlag = this->updownMotor.drop();   // 機構を下げる
    if ((cultivateOnFlag == true) && (this->updownMotor.dropping() == true)) {
        this->cultivateMotor.cultivateOn();       // 掘削モーターをON
    }
    // 機構を下限まで下げたら、上下モーター停止
    while (this->updownMotor.dropping() == true) {
        if (this->updownMotor.underLimit() == true) {
            break;
        }
    }
    this->updownMotor.stop(false);    // 非常停止ではないので、ブレーキを掛ける
}

/* 機構を上げて耕耘を終える */
void CultivatorCoreCultivateDriver::cultivateEnd(void)
{
    bool cultivateOffFlag = false;
    cultivateOffFlag = this->updownMotor.raise(); // 機構を上げる
    if ((cultivateOffFlag == true) && (this->updownMotor.raising() == true)) {
        this->cultivateMotor.cultivateOff();      // 掘削モーターをOFF
    }
    // 機構を上限まで上げたら、上下モーター停止
    while (this->updownMotor.raising() == true) {
        if (this->updownMotor.overLimit() == true) {
            break;
        }
    }
    this->updownMotor.stop(false);    // 非常停止ではないので、ブレーキを掛ける
}

/* 耕運作業の状態をフィードバック */
cultivate_feedback CultivatorCoreCultivateDriver::cultivateState(void)
{
    if (this->catchException() == true){
        return cultivate_feedback::exception;
    } else {
        if (this->updownMotor.overLimit() == true) {
            return cultivate_feedback::standby;
        }
        if (this->updownMotor.raising() == true) {
            return cultivate_feedback::raising;
        }
        if (this->updownMotor.dropping() == true) {
            return cultivate_feedback::dropping;
        }
        if (this->updownMotor.underLimit() == true) {
            return cultivate_feedback::working;
        }
    }
}

/*******************************************************************************
* 以下、テスト用メソッド
*******************************************************************************/
/* DCモーターテスト */
void CultivatorCoreCultivateDriver::testDCMotor(void)
{
    for (int index_i = 0; index_i < 3; index_i++){
        this->cultivateMotor.setMotorSpeed(0);
        this->runWait.wait(1000);
        this->cultivateMotor.setMotorSpeed(100);
        this->runWait.wait(3000);
        this->cultivateMotor.setMotorSpeed(0);
        this->runWait.wait(1000);
        this->cultivateMotor.setMotorSpeed(-100);
        this->runWait.wait(3000);
        this->cultivateMotor.setMotorBreak();
        this->runWait.wait(2000);
    }
}
