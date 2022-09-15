#ifndef GPIO_SETTING_H_
#define GPIO_SETTING_H_

#include <wiringPi.h>

/* GPIO設定スーパークラス */
class GPIOSetting
{
public:
    GPIOSetting(){ wiringPiSetup(); };
    ~GPIOSetting();

protected:
    void setPinOutput(unsigned int pin_no){ pinMode(pin_no, OUTPUT); };     // GPIO出力設定
    void outHigh(unsigned int pin_no){ digitalWrite(pin_no, HIGH); };       // HIGH出力
    void outLow(unsigned int pin_no){ digitalWrite(pin_no, LOW); };         // LOW出力
};

/* 非常停止設定クラス */
class EmergencySwitch : public GPIOSetting
{
private:
    unsigned int emg_pin = 25;

public:
    EmergencySwitch() {
        this->setPinOutput(this->emg_pin);
        this->switchOff();  // 初期状態ではOFFとする
    };
    ~EmergencySwitch();
    void switchOn(void){ this->outHigh(this->emg_pin); };   // 非常停止ON
    void switchOff(void){ this->outLow(this->emg_pin); };   // 非常停止OFF
};

/* 運転準備設定クラス */
class RunPrepareSwitch : public GPIOSetting
{
private:
    unsigned int switch_pin = 17;   // 運転準備スイッチ
    unsigned int signal_pin = 18;   // 運転準備完了

public:
    RunPrepareSwitch() {
        this->setPinOutput(this->switch_pin);
        this->setPinOutput(this->signal_pin);
    };
    ~RunPrepareSwitch();
    void switchOn(void){ this->outHigh(this->switch_pin); };    // スイッチON
    void switchOff(void){ this->outLow(this->switch_pin); };    // スイッチOFF
    void signalOn(void){ this->outHigh(this->signal_pin); };    // 準備完了ON
    void signalOff(void){ this->outLow(this->signal_pin); };    // 準備完了OFF
};

/* シグナルランプ設定クラス */
class Signal : public GPIOSetting
{
private:
    unsigned int blue_pin = 27;     // シグナルランプ：青
    unsigned int yellow_pin = 22;   // シグナルランプ：黃
    unsigned int red_pin = 23;      // シグナルランプ：赤

public:
    Signal() {
        this->setPinOutput(this->blue_pin);
        this->setPinOutput(this->yellow_pin);
        this->setPinOutput(this->red_pin);
    };
    ~Signal();
    // 青色ランプ点灯（動作状態）
    void blueOn(void) {
        this->lightOff();
        this->lightOn(this->blue_pin);
    };
    // 黄色ランプ点灯（待機(初期)状態）
    void yellowOn(void) {
        this->lightOff();
        this->lightOn(this->yellow_pin);
    };
    // 赤色ランプ点灯（非常停止状態）
    void redOn(void) {
        this->lightOff();
        this->lightOn(this->red_pin);
    };
    
private:
    // ランプ消灯
    void lightOff(void) {
        this->outLow(this->blue_pin);
        this->outLow(this->yellow_pin);
        this->outLow(this->red_pin);
    };
    // ランプ点灯
    void lightOn(unsigned int pin_no){ this->outHigh(pin_no); };
};

#endif
