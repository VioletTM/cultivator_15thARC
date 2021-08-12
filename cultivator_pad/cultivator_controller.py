#!/usr/bin/env python
# -*- coding: utf8 -*-
from tkinter import *
from tkinter import messagebox, ttk

class ControllerGUIMain:
    # 初期化
    def __init__(self, root):
        root.title("Cultivator_Controller")
        root.geometry("800x600")

        # マッピングボタン
        mapping_button = ttk.Button(text='Mapping', command=self.on_mapping)
        mapping_button.place(x=20, y=450, width=150, height=50)

        # 耕運ボタン
        cultivate_button = ttk.Button(text='Cultivate', command=self.on_cultivate)
        cultivate_button.place(x=180, y=450, width=150, height=50)

        # 自律／手動切替ボタン
        auto_button = ttk.Button(text='Auto', command=self.on_auto)
        auto_button.place(x=180, y=510, width=150, height=50)

        # 非常停止ボタン
        emergency_button = ttk.Button(command=self.on_emgstop)
        emergency_button.place(x=350, y=450, width=100, height=100)

        # 前進or加速ボタン
        front_button = ttk.Button(text='↑', command=self.on_front)
        front_button.place(x=600, y=420, width=50, height=50)

        # 後退or減速ボタン
        back_button = ttk.Button(text='↓', command=self.on_back)
        back_button.place(x=600, y=520, width=50, height=50)

        # 左旋回ボタン
        left_button = ttk.Button(text='←', command=self.on_left)
        left_button.place(x=550, y=470, width=50, height=50)

        # 右旋回ボタン
        right_button = ttk.Button(text='→', command=self.on_right)
        right_button.place(x=650, y=470, width=50, height=50)

        # 停止ボタン
        stop_button = ttk.Button(text='■', command=self.on_stop)
        stop_button.place(x=600, y=470, width=50, height=50)
    
    # マッピング
    def on_mapping(self):
        messagebox.showinfo('answer', 'Mapping')
    
    # 耕運
    def on_cultivate(self):
        messagebox.showinfo('answer', 'Cultivate')
    
    # 自律走行／手動走行切替
    def on_auto(self):
        messagebox.showinfo('answer', 'Auto')
    
    # 非常停止
    def on_emgstop(self):
        messagebox.showinfo('answer', 'Emergency Stop')

    # 前進or加速
    def on_front(self):
        messagebox.showinfo('answer', 'front')
    
    # 後退or減速
    def on_back(self):
        messagebox.showinfo('answer', 'back')
    
    # 左旋回
    def on_left(self):
        messagebox.showinfo('answer', 'left')
    
    # 右旋回
    def on_right(self):
        messagebox.showinfo('answer', 'right')

    # 停止
    def on_stop(self):
        messagebox.showinfo('answer', 'stop')

root = Tk()
ControllerGUIMain(root)
root.mainloop()
