#!/usr/bin/env python
# -*- coding: utf8 -*-
import rospy
from geometry_msgs.msg import Twist
from tkinter import *
from tkinter import messagebox, ttk

#ボタン幅、高さ
B_WIDTH = 150
B_HEIGHT = 50

class ControllerGUIMain:
    # 初期化
    def __init__(self, root):
        root.title("Cultivator_Controller")
        root.geometry("600x600")

        # マッピング処理開始ボタン
        mapping_button = ttk.Button(text='Mapping', command=self.on_mapping)
        mapping_button.place(x=50, y=350, width=B_WIDTH, height=B_HEIGHT)

        # 耕耘作業開始ボタン
        cultivate_button = ttk.Button(text='Cultivate', command=self.on_cultivate)
        cultivate_button.place(x=50, y=425, width=B_WIDTH, height=B_HEIGHT)

        # 自律／手動切替ボタン
        auto_button = ttk.Button(text='Auto', command=self.on_auto)
        auto_button.place(x=50, y=500, width=B_WIDTH, height=B_HEIGHT)

        # 耕耘機構上下ボタン
        digging_button = ttk.Button(text='Digging', command=self.on_digging)
        digging_button.place(x=225, y=500, width=B_WIDTH, height=B_HEIGHT)

        # ロボット可動ボタン
        moveable_button = ttk.Button(text='Moveable', command=self.on_moveable)
        moveable_button.place(x=400, y=500, width=B_WIDTH, height=B_HEIGHT)

        # 非常停止ボタン
        emergency_button = ttk.Button(command=self.on_emgstop)
        emergency_button.place(x=250, y=350, width=100, height=100)

        # 前進or加速ボタン
        front_button = ttk.Button(text='↑', command=self.on_front)
        front_button.place(x=450, y=325, width=50, height=50)

        # 後退or減速ボタン
        back_button = ttk.Button(text='↓', command=self.on_back)
        back_button.place(x=450, y=425, width=50, height=50)

        # 左旋回ボタン
        left_button = ttk.Button(text='←', command=self.on_left)
        left_button.place(x=400, y=375, width=50, height=50)

        # 右旋回ボタン
        right_button = ttk.Button(text='→', command=self.on_right)
        right_button.place(x=500, y=375, width=50, height=50)

        # 停止ボタン
        stop_button = ttk.Button(text='■', command=self.on_stop)
        stop_button.place(x=450, y=375, width=50, height=50)
    
    # マッピング
    def on_mapping(self):
        messagebox.showinfo('answer', 'Mapping')
    
    # 耕運
    def on_cultivate(self):
        messagebox.showinfo('answer', 'Cultivate')
    
    # 自律走行／手動走行切替
    def on_auto(self):
        messagebox.showinfo('answer', 'Auto')
    
    # 耕耘機構上下
    def on_digging(self):
        messagebox.showinfo('answer', 'Digging')
    
    # ロボット可動フラグ
    def on_moveable(self):
        messagebox.showinfo('answer', 'Moveable')
    
    # 非常停止
    def on_emgstop(self):
        #messagebox.showinfo('answer', 'Emergency Stop')
        msg_cmd = 'emergency'

    # 前進or加速
    def on_front(self):
        #messagebox.showinfo('answer', 'front')
        msg_cmd = 'front'
    
    # 後退or減速
    def on_back(self):
        #messagebox.showinfo('answer', 'back')
        msg_cmd = 'back'
    
    # 左旋回
    def on_left(self):
        #messagebox.showinfo('answer', 'left')
        msg_cmd = 'left'
    
    # 右旋回
    def on_right(self):
        #messagebox.showinfo('answer', 'right')
        msg_cmd = 'right'

    # 停止
    def on_stop(self):
        #messagebox.showinfo('answer', 'stop')
        msg_cmd = 'stop'

if __name__=="__main__":
    #rospy.init_node('cultivator_controller')
    #pub = rospy.Publisher('cmd_control', command)

    msg_cmd = ''

    root = Tk()
    ControllerGUIMain(root)
    root.mainloop()

    print(msg_cmd)

    #pub.publish(msg_cmd)
