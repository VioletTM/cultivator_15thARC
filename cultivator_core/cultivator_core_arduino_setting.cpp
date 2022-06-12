/*******************************************************************************
参考URL：
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#include "cultivator_core_arduino_setting.h"
#include <cstdlib>

/*******************************************************************************
* LEDSetting メソッド
*******************************************************************************/
/* LED出力設定 */
void LEDSetting::setLEDOutput(int led_pin)
{
    // 出力ピン番号設定
    this->pin_no = led_pin;

    pinMode(this->pin_no, OUTPUT);
    this->LEDOff();     // 出力設定の段階では、LEDは消灯させる
}

/* LED点灯 */
void LEDSetting::LEDOn(void)
{
    digitalWrite(this->pin_no, HIGH);
}

/* LED消灯 */
void LEDSetting::LEDOff(void)
{
    digitalWrite(this->pin_no, LOW);
}

/* LED点滅 */
void LEDSetting::LEDflash(unsigned long mmsecond)
{
    while(1) {
        this->LEDOn();
        delay(mmsecond);
        this->LEDOff();
        delay(mmsecond);
    }
}

/*******************************************************************************
* ServoMotorSetting メソッド
*******************************************************************************/
/* サーボ／ブラシレスモーター出力設定 */
void ServoMotorSetting::setMotorOutput(int pwm_pin)
{
    // 出力ピン番号設定
    this->pwm_pin_no = pwm_pin;

    this->attach(this->pwm_pin_no);
    this->setMotorAngle(this->servo_stop_angle);  // 連続回転タイプの場合、停止となる
}

/* サーボ／ブラシレスモーター角度設定（angle：0~180） */
void ServoMotorSetting::setMotorAngle(int angle)
{
    // 連続回転タイプの場合
    // angle < 90 : 正回転
    // angle = 90 : 停止
    // angle > 90 : 逆回転
    this->write(angle);
}

/* サーボ／ブラシレスモーター角度設定（microsecond：1000~2000） */
void ServoMotorSetting::setMotorAngleMS(int microsecond)
{
    // 連続回転タイプの場合
    // microsecond < 1500 : 正回転
    // microsecond = 1500 : 停止
    // microsecond > 1500 : 逆回転
    this->writeMicroseconds(microsecond);
}

/*******************************************************************************
* DrivingMotor メソッド（1500 = DEFAULT_PULSE_WIDTH）
*******************************************************************************/
/* モーター回転速度設定 */
void DrivingMotor::setMotorSpeed(int intValue)
{
    // intValue > 0 -> microsecond < 1500 （正回転）
    // intValue = 0 -> microsecond = 1500 （停止）
    // intValue < 0 -> microsecond > 1500 （逆回転）
    if (std::abs(intValue) > this->servo_pulse_width) {
        if (intValue > 0) {
            intValue = this->servo_pulse_width;
        } else {
            intValue = this->servo_pulse_width * (-1);
        }
    }
    this->setMotorAngleMS(this->default_pulse_width - intValue);
}

/* モーター回転速度取得 */
int DrivingMotor::getMotorSpeed(void)
{
    // サーボ角度を読み込み、microsecondに変換
    // read() < 90 -> microsecond < 1500 （正回転）
    // read() = 90 -> microsecond = 1500 （停止）
    // read() > 90 -> microsecond > 1500 （逆回転）
    int microsecond = this->default_pulse_width + ((this->read() - this->servo_stop_angle) * this->servo_pulse_width / this->servo_stop_angle);
    return (this->default_pulse_width - microsecond);
}

/*******************************************************************************
* DCMotorSetting メソッド
*******************************************************************************/
/* DCモーター出力設定 */
void DCMotorSetting::setMotorOutput(int b_pin, int f_pin, int pwm_pin)
{
    // 出力ピン番号設定
    this->f_pin_no = f_pin;
    this->b_pin_no = b_pin;
    this->pwm_pin_no = pwm_pin;
    
    pinMode(this->f_pin_no, OUTPUT);
    pinMode(this->b_pin_no, OUTPUT);
    if (this->pwm_pin_no >= 2) {
        // PWM制御ピンに接続したときのみ有効
        this->isPwmControl = true;
        pinMode(this->pwm_pin_no, OUTPUT);
    } else {
        this->isPwmControl = false;
    }
    this->setMotorSpeed(0);     // 出力設定の段階では、モーターは停止させる
}

/* DCモーター速度設定（intValue：-255~255） */
void DCMotorSetting::setMotorSpeed(int intValue)
{
    if (intValue == 0) {
        // 停止（但し、惰性で回転する）
        digitalWrite(this->f_pin_no, LOW);
        digitalWrite(this->b_pin_no, LOW);
    } else if (intValue > 0) {
        // 正回転
        digitalWrite(this->f_pin_no, HIGH);
        digitalWrite(this->b_pin_no, LOW);
    } else {
        // 逆回転
        digitalWrite(this->f_pin_no, LOW);
        digitalWrite(this->b_pin_no, HIGH);
    }

    if (this->isPwmControl == true) {
        analogWrite(this->pwm_pin_no, std::abs(intValue));
    }
}

/* DCモーターブレーキ */
void DCMotorSetting::setMotorBreak(void)
{
    digitalWrite(this->f_pin_no, HIGH);
    digitalWrite(this->b_pin_no, HIGH);

    if (this->isPwmControl == true) {
        analogWrite(this->pwm_pin_no, 0);
    }
}

/*******************************************************************************
* CultivateMotor メソッド
*******************************************************************************/
/* 掘削開始（モーター回転、PWM制御なし）*/
void CultivateMotor::cultivateOn(void)
{
    this->setMotorSpeed(1);
}

/* 掘削終了（モーター停止、PWM制御なし）*/
void CultivateMotor::cultivateOff(void)
{
    this->setMotorSpeed(0);
}

/*******************************************************************************
* UpDownMotor メソッド
*******************************************************************************/
/* センサー入力設定 */
void UpDownMotor::setSensorInput(int over_limit_pin, int under_limit_pin, int up_pin, int down_pin)
{
    // 入力ピン番号設定
    this->over_limit_pin_no = over_limit_pin;
    this->under_limit_pin_no = under_limit_pin;
    this->up_pin_no = up_pin;
    this->down_pin_no = down_pin;
    
    pinMode(this->over_limit_pin_no, INPUT);
    pinMode(this->under_limit_pin_no, INPUT);
    pinMode(this->up_pin_no, INPUT);
    pinMode(this->down_pin_no, INPUT);
}

/* 耕耘機構上昇（PWM制御なし） */
bool UpDownMotor::raise(void)
{
    this->setMotorSpeed(1);
    delay(this->updown_after_wait_ms);
    return true;
}

/* 耕耘機構下降（PWM制御なし） */
bool UpDownMotor::drop(void)
{
    this->setMotorSpeed(-1);
    delay(this->updown_after_wait_ms);
    return true;
}

/* 上昇／下降停止 */
void UpDownMotor::stop(bool emergency)
{
    if (emergency == false) {
        this->setMotorBreak();
    } else {
        this->setMotorSpeed(0); // 非常停止した場合、機構を手動で収納させるためブレーキをかけない
    }
}

/* 上限位置到達確認 */
bool UpDownMotor::overLimit(void)
{
    return this->getSensorRead(this->over_limit_pin_no);
}

/* 下限位置到達確認 */
bool UpDownMotor::underLimit(void)
{
    return this->getSensorRead(this->under_limit_pin_no);
}

/* 耕耘機構上昇確認 */
bool UpDownMotor::raising(void)
{
    return this->getSensorRead(this->up_pin_no);
}

/* 耕耘機構下降確認 */
bool UpDownMotor::dropping(void)
{
    return this->getSensorRead(this->down_pin_no);
}

/* センサー入力値取得 */
bool UpDownMotor::getSensorRead(int pin_no)
{
    if (digitalRead(pin_no) == HIGH) {
        return true;
    } else {
        return false;
    }
}
