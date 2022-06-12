#!/usr/bin/env python
# -*- coding: utf8 -*-
import rospy
from std_msgs.msg import UInt16
from enum_setting import CommandType, RobotState, MotorState, CultivateCmd, CultivateFb
from gpio_setting import Signal, EmergencySwitch

class ServiceCore():
    def __init__(self):
        self._initParam()                   # 各種パラメータ初期化
        self.lump = Signal()                # シグナルランプ
        self.emergency = EmergencySwitch()  # 非常停止スイッチ
        # Publisher
        self.cultivate_pub = rospy.Publisher('/cultivate', UInt16)      # 耕運作業命令をArduinoへ送信
        self.robot_state_pub = rospy.Publisher('/robot_state', UInt16)  # ロボットの作業状況をPCへ送信
        # Subscriber
        self.motor_state_sub = rospy.Subscriber('/motor_state', UInt16, self.motorStateCallback)    # 駆動モーターの状態を受信
        self.cultivate_sub = rospy.Subscriber('/cultivate_fb', UInt16, self.cultivateCallback)      # 耕運作業のフィードバックを受信
        self.cmd_type_sub = rospy.Subscriber('/cmd_type', UInt16)

    # 各種パラメータ初期化
    def _initParam(self):
        self.cmd_type_msg = CommandType.no_command
        self.motor_state = MotorState.stop
        self.cultivate_feedback = CultivateFb.standby
        self.cultivate_command_msg = CultivateCmd.work_no
        self._robot_state_standby()     # 待機状態にする

    # 駆動モーターの状態を受信
    def motorStateCallback(self, motor_state_msg):
        self.motor_state = motor_state_msg.data
        if (self.motor_state == MotorState.exception):
            self._robot_state_emergency()   # 例外検知した場合、非常停止

    # 耕運作業のフィードバックを受信
    def cultivateCallback(self, cultivate_feedback_msg):
        self.cultivate_feedback = cultivate_feedback_msg.data
        if (self.cultivate_feedback == CultivateFb.exception):
            self._robot_state_emergency()   # 例外検知した場合、非常停止
    
    # PCからのボタン命令を受信
    def cmdTypeCallback(self, cmd_type_msg):
        self.cmd_type_msg = cmd_type_msg.data
        if (self.robot_state_msg == RobotState.emergency):  # 非常停止状態の場合
            if (self.cmd_type_msg == CommandType.emergency_stop):
                self._robot_state_standby()     # 非常停止を解除して待機状態
        elif (self.robot_state_msg == RobotState.standby):  # 待機状態の場合
            if (self.cmd_type_msg == CommandType.emergency_stop):
                self._robot_state_emergency()   # 非常停止
            elif (self.cmd_type_msg == CommandType.moveable):
                self._robot_state_other()       # その他（ロボットの運搬など）状態
        else:
            if (self.cmd_type_msg == CommandType.emergency_stop):
                self._robot_state_emergency()   # 非常停止
            elif (self.cmd_type_msg == CommandType.moveable):
                self._robot_state_standby()     # 待機状態
            
            if (self.robot_state_msg == RobotState.other):
                if (self.cmd_type_msg == CommandType.mapping):
                    self._robot_state_mapping()     # マッピング処理開始
                elif (self.cmd_type_msg == CommandType.cultivate):
                    self._robot_state_cultivate()   # 耕耘作業開始
            elif (self.robot_state_msg == RobotState.mapping):
                if (self.cmd_type_msg == CommandType.mapping):
                    self._robot_state_other()       # マッピング処理終了
            elif (self.robot_state_msg == RobotState.cultivate):
                if (self.cmd_type_msg == CommandType.cultivate):
                    self._robot_state_other()       # 耕耘作業終了
        
    # 駆動モーターの状態と耕運作業のフィードバックから作業命令を決定
    def _cultivate_cmd(self):
        if (self.motor_state == MotorState.curve):
            self.cultivate_command_msg = CultivateCmd.work_end
        return self.cultivate_command_msg

    # 耕運作業命令をArduinoへ送信
    def send_cultivate_command_msg(self):
        if (self.robot_state_msg == RobotState.cultivate):
            self.cultivate_pub.publish(self._cultivate_cmd())
    
    # ロボットの状態
    ## 非常停止
    def _robot_state_emergency(self):
        self.robot_state_msg = RobotState.emergency
        self.emergency.switchOn()
        self.lump.redOn()
    ## 待機（初期状態）
    def _robot_state_standby(self):
        self.robot_state_msg = RobotState.standby
        self.emergency.switchOff()  # 非常停止解除
        self.lump.yellowOn()
    ## マッピング処理
    def _robot_state_mapping(self):
        self.robot_state_msg = RobotState.mapping
        self.lump.blueOn()
    ## 耕運作業
    def _robot_state_cultivate(self):
        self.robot_state_msg = RobotState.cultivate
        self.lump.blueOn()
    ## その他（ロボットの運搬など）
    def _robot_state_other(self):
        self.robot_state_msg = RobotState.other
        self.lump.blueOn()
    
    # ロボットの作業状況をPCへ送信
    def send_robot_state_msg(self):
        self.robot_state_pub.publish(self.robot_state_msg)

def main():
    rospy.init_node('service_core') # ノード立ち上げ
    servicecore = ServiceCore()     # クラス生成
    rospy.spin()                    # spin
    rate = rospy.Rate(2)            # ratesleep
    while not rospy.is_shutdown():
        servicecore.send_robot_state_msg()          # PCへ
        servicecore.send_cultivate_command_msg()    # Arduinoへ
        rate.sleep()

if __name__=="__main__":
    main()
