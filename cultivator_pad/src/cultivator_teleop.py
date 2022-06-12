#!/usr/bin/env python
# -*- coding: utf8 -*-

import sys
from curses import window
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from controller import Ui_MainWindow

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from enum_setting import CommandType, RobotState

class CultivatorTeleop(QDialog):
    def __init__(self, parent:None):
        # GUI初期設定
        super(CultivatorTeleop, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # 速度／角速度設定
        self.LIN_VEL = 1    # 速度[m/s]
        self.ANG_VEL = 1    # 角速度[rad/s]
        # Publisher
        self.cmd_vel_Twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_type = CommandType.no_command
        self.cmd_type_pub = rospy.Publisher('/cmd_type', UInt16)
        # Subscriber
        self.robot_state_msg = RobotState.standby
        self.robot_state_sub = rospy.Subscriber('/robot_state', UInt16, self.robotStateCallback)

    # 前進
    def go_front(self):
        self.cmd_vel_Twist.angular.z = 0
        self.cmd_vel_Twist.linear.x = self.LIN_VEL      # LIN_VEL[m/s]で前進
        self.send_cmd_vel()
    # 後退
    def go_back(self):
        self.cmd_vel_Twist.angular.z = 0
        self.cmd_vel_Twist.linear.x = -self.LIN_VEL     # LIN_VEL[m/s]で後退
        self.send_cmd_vel()
    # 左旋回
    def go_rotate_left(self):
        self.cmd_vel_Twist.linear.x = 0
        self.cmd_vel_Twist.angular.z = self.ANG_VEL     # ANG_VEL[rad/s]で左旋回
        self.send_cmd_vel()
    # 右旋回
    def go_rotate_right(self):
        self.cmd_vel_Twist.linear.x = 0
        self.cmd_vel_Twist.angular.z = -self.ANG_VEL    # ANG_VEL[rad/s]で右旋回
        self.send_cmd_vel()
    # 停止
    def stop(self):
        self.cmd_vel_Twist.linear.x = 0
        self.cmd_vel_Twist.angular.z = 0
        self.send_cmd_vel()
    ## Arduinoへ送信
    def send_cmd_vel(self):
        if ((self.robot_state_msg != RobotState.emergency) and (self.robot_state_msg != RobotState.standby)):
            self.cmd_vel_pub.publish(self.cmd_vel_Twist)
    
    # 非常停止
    def emergency_stop(self):
        self.cmd_type = CommandType.emergency_stop
        self.send_cmd_type()
    # マッピング
    def mapping(self):
        self.cmd_type = CommandType.mapping
        self.send_cmd_type()
    # 耕耘作業
    def cultivate(self):
        self.cmd_type = CommandType.cultivate
        self.send_cmd_type()
    # 自律走行
    def auto_flag(self):
        self.cmd_type = CommandType.auto
        self.send_cmd_type()
    # 手動耕耘
    def digging(self):
        self.cmd_type = CommandType.digging
        self.send_cmd_type()
    # 動作可能
    def moveable_flag(self):
        self.cmd_type = CommandType.moveable
        self.send_cmd_type()
    ## RaspberryPiへコマンドの種類を送信
    def send_cmd_type(self):
        self.cmd_type_pub.publish(self.cmd_type)

    # RaspberryPiからロボットの状態を受信
    def robotStateCallback(self, robot_state_msg):
        self.robot_state_msg = robot_state_msg.data

def main():
    rospy.init_node('cultivator_teleop')
    app = QApplication(sys.argv)
    window = CultivatorTeleop()
    window.show()
    rospy.spin()
    sys.exit(app.exec_())

if __name__=="__main__":
    main()
