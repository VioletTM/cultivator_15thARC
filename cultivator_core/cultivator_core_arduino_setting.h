/*******************************************************************************
参考URL：
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#ifndef CULTIVATOR_CORE_ARDUINO_SETTING_H_
#define CULTIVATOR_CORE_ARDUINO_SETTING_H_

#include <Arduino.h>
#include <Servo.h>

/*******************************************************************************
* LED関連クラス
*******************************************************************************/
/* LED設定クラス */
class LEDSetting
{
protected:
    int pin_no;     // LEDピン番号

public:
    LEDSetting(){}      // コンストラクタ
    ~LEDSetting(){}     // デストラクタ
    void setLEDOutput(int led_pin); // LED出力設定
    void LEDOn(void);               // LED点灯
    void LEDOff(void);              // LED消灯
    void LEDflash(unsigned long mmsecond);  // LED点滅
};

/*******************************************************************************
* 経過時間表示関連クラス
*******************************************************************************/
/* 経過時間表示設定クラス */
class TimeSetting
{
public:
    TimeSetting(){}    // コンストラクタ
    ~TimeSetting(){}   // デストラクタ
    unsigned long milliseconds(void){ return millis(); }    // ミリ秒
    unsigned long microseconds(void){ return micros(); }    // マイクロ秒
};

/*******************************************************************************
* 時間停止関連クラス
*******************************************************************************/
/* 時間停止設定クラス */
class WaitSetting
{
public:
    WaitSetting(){}     // コンストラクタ
    ~WaitSetting(){}    // デストラクタ
    void wait(unsigned long mmsecond){ delay(mmsecond); }   // ミリ秒
    void waitMS(unsigned long microsecond){ delayMicroseconds(microsecond); }   // マイクロ秒
};

/*******************************************************************************
* サーボ／ブラシレスモーター関連クラス
*******************************************************************************/
/* サーボ／ブラシレスモーター設定スーパークラス */
class ServoMotorSetting : public Servo
{
protected:
    int pwm_pin_no;     // PWMピン番号
    const int servo_stop_angle = 90;
    const int servo_pulse_width = 500;
    const int default_pulse_width = 1500;
    
public:
    ServoMotorSetting(){}                   // コンストラクタ
    ~ServoMotorSetting(){ this->detach(); } // デストラクタ
    void setMotorOutput(int pwm_pin);       // モーター出力設定
    void setMotorAngle(int angle);          // モーター角度設定
    void setMotorAngleMS(int microsecond);  // モーター角度設定
};

/* 車輪駆動用モーター設定クラス */
class DrivingMotor : public ServoMotorSetting
{
public:
    DrivingMotor(){}    // コンストラクタ
    ~DrivingMotor(){}   // デストラクタ
    void setMotorSpeed(int intValue);   // モーター速度設定
    int getMotorSpeed(void);            // モーター速度取得
};

/*******************************************************************************
* DCモーター関連クラス
*******************************************************************************/
/* DCモーター設定スーパークラス */
class DCMotorSetting
{
private:
    bool isPwmControl;  // PWM制御の有無

protected:
    int b_pin_no;
    int f_pin_no;
    int pwm_pin_no;     // PWMピン番号

public:
    DCMotorSetting(){}      // コンストラクタ
    ~DCMotorSetting(){}     // デストラクタ
    void setMotorOutput(int b_pin, int f_pin, int pwm_pin); // DCモーター出力設定
    void setMotorSpeed(int intValue);   // DCモーター速度設定
    void setMotorBreak(void);           // DCモーターブレーキ
};

/* 耕耘掘削用モーター設定クラス */
class CultivateMotor : public DCMotorSetting
{
public:
    CultivateMotor(){}      // コンストラクタ
    ~CultivateMotor(){}     // デストラクタ
    void cultivateOn(void);     // 掘削開始（モーター回転、PWM制御なし）
    void cultivateOff(void);    // 掘削終了（モーター停止、PWM制御なし）
};

/* 耕耘機構上下用モーター設定クラス */
class UpDownMotor : public DCMotorSetting
{
private:
    const unsigned long updown_after_wait_ms = 3000;    // 上昇／下降開始後の待機時間[ms]

protected:
    int over_limit_pin_no;
    int under_limit_pin_no;
    int up_pin_no;
    int down_pin_no;

public:
    UpDownMotor(){}     // コンストラクタ
    ~UpDownMotor(){}    // デストラクタ
    void setSensorInput(int over_limit_pin, int under_limit_pin, int up_pin, int down_pin);  // センサー入力設定
    bool raise(void);           // 耕耘機構上昇（PWM制御なし）
    bool drop(void);            // 耕耘機構下降（PWM制御なし）
    void stop(bool emergency);  // 上昇／下降停止
    bool overLimit(void);       // 上限位置到達確認（0：未達, それ以外：到達）
    bool underLimit(void);      // 下限位置到達確認（0：未達, それ以外：到達）
    bool raising(void);         // 耕耘機構上昇確認（0：下降, それ以外：上昇中）
    bool dropping(void);        // 耕耘機構下降確認（0：上昇, それ以外：下降中）

private:
    bool getSensorRead(int pin_no);     // センサー入力値取得
};

#endif
