#include "cultivator_service_core.h"

/* 各種パラメータ初期化 */
bool CultivatorServiceCore::init(void)
{
    this->command_type_no = static_cast<std_msgs::UInt16>(command_type::no_command);
    this->motor_state_no = static_cast<std_msgs::UInt16>(motor_state::stop);
    this->cultivate_feedback_no = static_cast<std_msgs::UInt16>(cultivate_feedback::standby);
    this->cultivate_cmd_no = static_cast<std_msgs::UInt16>(cultivate_cmd::work_no);
    this->robot_state_standby();    // 待機状態にする
}

/* 駆動モーターの状態を受信 */
void CultivatorServiceCore::motorStateCallback(const std_msgs::UInt16& motor_state_msg)
{
    this->motor_state_no = motor_state_msg.data;
    if (this->motor_state_no == static_cast<std_msgs::UInt16>(motor_state::exception)){
        this->robot_state_emergency();  // 例外検知した場合、非常停止
    }
}

/* 耕運作業のフィードバックを受信 */
void CultivatorServiceCore::cultivateCallback(const std_msgs::UInt16& cultivate_feedback_msg)
{
    this->cultivate_feedback_no = cultivate_feedback_msg.data;
    if (this->cultivate_feedback_no == static_cast<std_msgs::UInt16>(cultivate_feedback::exception)){
        this->robot_state_emergency();  // 例外検知した場合、非常停止
    }
}

/* PCからのボタン命令を受信 */
void CultivatorServiceCore::cmdTypeCallback(const std_msgs::UInt16& cmd_type_msg)
{
    this->command_type_no = cmd_type_msg.data;
    if (this->robot_state_no == static_cast<std_msgs::UInt16>(robot_state::emergency)){
        // 非常停止状態の場合（赤ランプ点灯）
        if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::emergency_stop)){
            this->robot_state_standby();    // 非常停止を解除して待機状態
        }
    } else if (this->robot_state_no == static_cast<std_msgs::UInt16>(robot_state::standby)){
        // 待機状態の場合（黄ランプ点灯）
        if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::emergency_stop)){
            this->robot_state_emergency();  // 非常停止状態
        }
        else if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::moveable)){
            this->robot_state_other();      // その他（ロボットの運搬など）状態
        }
    } else {
        // その他（青ランプ点灯）
        if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::emergency_stop)){
            this->robot_state_emergency();  // 非常停止状態
        }
        else if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::moveable)){
            this->robot_state_standby();    // 待機状態
        }
        switch (this->robot_state_no)
        {
        case static_cast<std_msgs::UInt16>(robot_state::other):
            if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::mapping)){
                this->robot_state_mapping();    // マッピング作業開始
            }
            else if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::cultivate)){
                this->robot_state_cultivate();  // 耕耘作業開始
            }
            break;
        case static_cast<std_msgs::UInt16>(robot_state::mapping):
            if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::mapping)){
                this->robot_state_other();      // マッピング作業終了
            }
            break;
        case static_cast<std_msgs::UInt16>(robot_state::cultivate):
            if (this->command_type_no == static_cast<std_msgs::UInt16>(command_type::mapping)){
                this->robot_state_other();      // 耕耘作業終了
            }
            break;
        }
    }
}

/* 駆動モーターの状態と耕運作業のフィードバックから作業命令を決定 */
void CultivatorServiceCore::cultivate_command(void)
{
    if (this->motor_state_no == static_cast<std_msgs::UInt16>(motor_state::curve)){
        this->cultivate_cmd_no.data = static_cast<std_msgs::UInt16>(cultivate_cmd::work_end);
    }
    // 耕運作業命令をArduinoへ送信
    if (this->robot_state_no == static_cast<std_msgs::UInt16>(robot_state::cultivate)){
        this->cultivate_pub.publish(&this->cultivate_cmd_no);
    }
}

/* ロボットの状態を送信 */
void CultivatorServiceCore::send_robot_state_msg(void)
{
    this->robot_state_pub.publish(&this->robot_state_no);
}

/* 非常停止 */
void CultivatorServiceCore::robot_state_emergency(void)
{
    this->robot_state_no = static_cast<std_msgs::UInt16>(robot_state::emergency);
    this->emergency.switchOn();     // 非常停止
    this->lump.redOn();             // 赤色ランプ点灯
    this->send_robot_state_msg();   // ロボットの状態を送信
}

/* 待機（初期状態）*/
void CultivatorServiceCore::robot_state_standby(void)
{
    this->robot_state_no = static_cast<std_msgs::UInt16>(robot_state::standby);
    this->emergency.switchOff();    // 非常停止解除
    this->lump.yellowOn();          // 黄色ランプ点灯
    this->runprepare.switchOff();   // 運転準備スイッチOFF
    this->runprepare.signalOff();   // 準備完了OFF
    this->send_robot_state_msg();   // ロボットの状態を送信
}

/* マッピング処理 */
void CultivatorServiceCore::robot_state_mapping(void)
{
    this->robot_state_no = static_cast<std_msgs::UInt16>(robot_state::mapping);
    this->lump.blueOn();            // 青色ランプ点灯
    this->send_robot_state_msg();   // ロボットの状態を送信
}

/* 耕運作業 */
void CultivatorServiceCore::robot_state_cultivate(void)
{
    this->robot_state_no = static_cast<std_msgs::UInt16>(robot_state::cultivate);
    this->lump.blueOn();            // 青色ランプ点灯
    this->send_robot_state_msg();   // ロボットの状態を送信
}

/* その他（ロボットの運搬など）*/
void CultivatorServiceCore::robot_state_other(void)
{
    this->robot_state_no = static_cast<std_msgs::UInt16>(robot_state::other);
    this->lump.blueOn();            // 青色ランプ点灯
    this->runprepare.switchOn();    // 運転準備スイッチON
    this->runprepare.signalOn();    // 準備完了ON
    this->send_robot_state_msg();   // ロボットの状態を送信
}


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cultivator_service_core");
    CultivatorServiceCore cultivator_service_core;
    
    ros::Rate loop_rate(125);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
