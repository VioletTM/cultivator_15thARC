/*******************************************************************************
参考URL：
https://github.com/asukiaaa/kagotos/blob/master/atmega/src/main.cpp
*******************************************************************************/

#ifndef CULTIVATOR_CORE_ARDUINO_SETTING_H_
#define CULTIVATOR_CORE_ARDUINO_SETTING_H_

#include <Arduino.h>
#include <Servo.h>

#define SERVO_STOP_ANGLE    90
#define SERVO_PULSE_WIDTH   500

/*******************************************************************************
* LED関連クラス
*******************************************************************************/
/* LED設定クラス */
class LEDSetting
{
protected:
    int pin_no;     // LEDピン番号

public:
    LEDSetting(int led_pin){ this->pin_no = led_pin; }  // コンストラクタ
    ~LEDSetting(){}                 // デストラクタ
    void setLEDOutput(void);        // LED出力設定
    void LEDOn(void);               // LED点灯
    void LEDOff(void);              // LED消灯
    void LEDflash(int mmsecond);    // LED点滅
};

/*******************************************************************************
* サーボ／ブラシレスモーター関連クラス
*******************************************************************************/
/* サーボ／ブラシレスモーター設定スーパークラス */
class ServoMotorSetting : public Servo
{
protected:
    int pwm_pin_no;     // PWMピン番号
    
public:
    ServoMotorSetting(int pwm_pin){ this->pwm_pin_no = pwm_pin; }   // コンストラクタ
    ~ServoMotorSetting(){ this->detach(); }                         // デストラクタ
    void setMotorOutput(void);              // モーター出力設定
    void setMotorAngle(int angle);          // モーター角度設定
    void setMotorAngleMS(int microsecond);  // モーター角度設定
}

/* 車輪駆動用モーター設定クラス */
class DrivingMotor : public ServoMotorSetting
{
public:
    DrivingMotor(int pwm_pin) : ServoMotorSetting(pwm_pin){}     // コンストラクタ
    ~DrivingMotor(){}                   // デストラクタ
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
    DCMotorSetting(int b_pin, int f_pin, int pwm_pin)     // コンストラクタ
    {
        this->f_pin_no = f_pin;
        this->b_pin_no = b_pin;
        this->pwm_pin_no = pwm_pin;
    }
    ~DCMotorSetting(){}                 // デストラクタ
    void setMotorOutput(void);          // DCモーター出力設定
    void setMotorSpeed(int intValue);   // DCモーター速度設定
    void setMotorBreak(void);           // DCモーターブレーキ
};

/* 耕耘掘削用モーター設定クラス */
class CultivateMotor : public DCMotorSetting
{
public:
    CultivateMotor(int b_pin, int f_pin, int pwm_pin) : DCMotorSetting(b_pin, f_pin, pwm_pin){}   // コンストラクタ
    ~CultivateMotor(){}         // デストラクタ
    void cultivateOn(void);     // 掘削開始（モーター回転、PWM制御なし）
    void cultivateOff(void);    // 掘削終了（モーター停止、PWM制御なし）
};

/* 耕耘機構上下用モーター設定クラス */
class UpDownMotor : public DCMotorSetting
{
private:
    const int updown_after_wait_ms = 3000;  // 上昇／下降開始後の待機時間[ms]

protected:
    int over_limit_pin_no;
    int under_limit_pin_no;
    int up_pin_no;
    int down_pin_no;

public:
    // コンストラクタ
    UpDownMotor(int b_pin, int f_pin, int pwm_pin, int over_limit_pin, int under_limit_pin, int up_pin, int down_pin)
        : DCMotorSetting(b_pin, f_pin, pwm_pin)
    {
        this->over_limit_pin_no = over_limit_pin;
        this->under_limit_pin_no = under_limit_pin;
        this->up_pin_no = up_pin;
        this->down_pin_no = down_pin;
    }
    ~UpDownMotor(){}            // デストラクタ
    void setSensorInput(void);  // センサー入力設定
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