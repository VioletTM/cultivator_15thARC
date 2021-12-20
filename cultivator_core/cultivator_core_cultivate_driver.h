#ifndef CULTIVATOR_CORE_CULTIVATE_DRIVER_H_
#define CULTIVATOR_CORE_CULTIVATE_DRIVER_H_

#include "cultivator_core_arduino_setting.h"

#define CULTIVATE_MOTOR_1_PIN       6
#define CULTIVATE_MOTOR_2_PIN       7
#define LIFT_UP_PIN                 8
#define LIFT_DOWN_PIN               9

#define CULTIVATE_MOTOR_PWM_PIN     -1  //2021/12現在、使用する予定はないのでダミー入力
#define LIFT_PWM_PIN                -1  //2021/12現在、使用する予定はないのでダミー入力

#define OVER_LIMIT_PIN    40
#define UP_PIN            41
#define DOWN_PIN          42
#define UNDER_LIMIT_PIN   43

/* 耕耘機構クラス */
class CultivatorCoreCultivateDriver
{
private:
    // 掘削モーター
    CultivateMotor cultivateMotor(CULTIVATE_MOTOR_1_PIN, CULTIVATE_MOTOR_2_PIN, CULTIVATE_MOTOR_PWM_PIN);
    // 上下モーター
    UpDownMotor updownMotor(LIFT_UP_PIN, LIFT_DOWN_PIN, LIFT_PWM_PIN, OVER_LIMIT_PIN, UNDER_LIMIT_PIN, UP_PIN, DOWN_PIN);

public:
    CultivatorCoreCultivateDriver(){}       // コンストラクタ
    ~CultivatorCoreCultivateDriver(){}      // デストラクタ
    bool init(void);            // 初期化
    bool catchException(void);  // 例外検出
    bool cultivateBegin(void);  // 機構を下げて耕耘を始める
    bool cultivateEnd(void);    // 機構を上げて耕耘を終える
}

#endif