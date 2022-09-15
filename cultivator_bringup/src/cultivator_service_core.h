#ifndef CULTIVATOR_SERVICE_CORE_H_
#define CULTIVATOR_SERVICE_CORE_H_

#include <ros/ros.h>    //ROS基本ヘッダファイル
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "gpio_setting.h"

/* PCからのボタン命令 */
enum class command_type {
    emergency_stop,
    no_command,
    moveable,
    mapping,
    cultivate,
    digging,
    auto
};

/* ロボットの状態 */
enum class robot_state {
    emergency,
    standby,
    other,
    mapping,
    cultivate
};

/* 駆動モーターの状態 */
enum class motor_state {
    exception,
    stop,
    straight,
    curve
};

/* 耕運作業命令 */
enum class cultivate_cmd {
    work_no,
    work_begin,
    work_end
};

/* 耕運作業のフィードバック */
enum class cultivate_feedback {
    exception,
    standby,
    dropping,
    working,
    raising
};

class CultivatorServiceCore
{
private:
    Signal lump;                    // シグナルランプ
    EmergencySwitch emergency;      // 非常停止スイッチ
    RunPrepareSwitch runprepare;    // 運転準備スイッチ
    std_msgs::UInt16 command_type_no;
    std_msgs::UInt16 robot_state_no;
    std_msgs::UInt16 motor_state_no;
    std_msgs::UInt16 cultivate_feedback_no;
    std_msgs::UInt16 cultivate_cmd_no;

    // ROS NodeHandle
    ros::NodeHandle nh_;
    bool init(void);    // 各種パラメータ初期化

    // ロボットの状態
    void robot_state_emergency(void);   // 非常停止
    void robot_state_standby(void);     // 待機（初期状態）
    void robot_state_mapping(void);     // マッピング処理
    void robot_state_cultivate(void);   // 耕運作業
    void robot_state_other(void);       // その他（ロボットの運搬など）

public:
    CultivatorServiceCore(){ this->init(); };
    ~CultivatorServiceCore();
    bool controlLoop(void);
    // ROS Topic Publishers
    ros::Publisher cultivate_pub = nh_.advertise<std_msgs::UInt16>("/cultivate", 1);        // 耕運作業命令をArduinoへ送信
    ros::Publisher robot_state_pub = nh_.advertise<std_msgs::UInt16>("/robot_state", 1);    // ロボットの作業状況をPCへ送信
    // ROS Topic Subscribers
    ros::Subscriber motor_state_sub = nh_.subscribe("/motor_state", CultivatorServiceCore::motorStateCallback, this);  // 駆動モーターの状態を受信
    ros::Subscriber cultivate_sub = nh_.subscribe("/cultivate_fb", CultivatorServiceCore::cultivateCallback, this);    // 耕運作業のフィードバックを受信
    ros::Subscriber cmd_type_sub = nh_.subscribe("/cmd_type", CultivatorServiceCore::cmdTypeCallback, this);

    // Function prototypes
    void motorStateCallback(const std_msgs::UInt16& motor_state_msg);       // 駆動モーターの状態を受信
    void cultivateCallback(const std_msgs::UInt16& cultivate_feedback_msg); // 耕運作業のフィードバックを受信
    void cmdTypeCallback(const std_msgs::UInt16& cmd_type_msg);             // PCからのボタン命令を受信

    void cultivate_command(void);       // 駆動モーターの状態と耕運作業のフィードバックから作業命令を決定
    void send_robot_state_msg(void);    // ロボットの状態を送信
};

#endif
