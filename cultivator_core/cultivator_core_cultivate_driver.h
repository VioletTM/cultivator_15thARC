#ifndef CULTIVATOR_CORE_CULTIVATE_DRIVER_H_
#define CULTIVATOR_CORE_CULTIVATE_DRIVER_H_

#include "cultivator_core_arduino_setting.h"

enum class cultivate_cmd {
    work_no,
    work_begin,
    work_end
};

enum class cultivate_feedback {
    exception,
    standby,
    dropping,
    working,
    raising
};

/* 耕耘機構クラス */
class CultivatorCoreCultivateDriver
{
private:
    const int cultivate_pin_1 = 6;
    const int cultivate_pin_2 = 7;
    const int cultivate_pwm_pin = -1;     // 使用予定はないのでダミー入力
    const int lift_up_pin = 8;
    const int lift_down_pin = 9;
    const int lift_pwm_pin = -1;    // 使用予定はないのでダミー入力
    const int over_limit_pin = 40;
    const int up_pin = 41;
    const int down_pin = 42;
    const int under_limit_pin = 43;
    
    CultivateMotor cultivateMotor;  // 掘削モーター
    UpDownMotor updownMotor;        // 上下モーター
    WaitSetting runWait;

public:
    CultivatorCoreCultivateDriver(){}       // コンストラクタ
    ~CultivatorCoreCultivateDriver(){}      // デストラクタ
    void init(void);            // 初期化
    bool catchException(void);  // 例外検出
    void cultivateBegin(void);  // 機構を下げて耕耘を始める
    void cultivateEnd(void);    // 機構を上げて耕耘を終える
    cultivate_feedback cultivateState(void);    // 耕運作業の状態をフィードバック
    /* テスト用メソッド */
    void testDCMotor(void);
    /* テスト用メソッド */
};

#endif
