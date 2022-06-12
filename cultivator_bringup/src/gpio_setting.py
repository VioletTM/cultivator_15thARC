#!/usr/bin/env python
# -*- coding: utf8 -*-
#import RPi.GPIO as GPIO
import wiringpi

# GPIO設定スーパークラス
class GPIOSetting():
    def __init__(self):
        wiringpi.wiringPiSetupGpio()

    # GPIO出力設定
    def _setPinOutput(self, pin_no):
        wiringpi.pinMode(pin_no, 1)

    # HIGH出力
    def _OutHigh(self, pin_no):
        wiringpi.digitalWrite(pin_no, 1)

    # LOW出力
    def _OutLow(self, pin_no):
        wiringpi.digitalWrite(pin_no, 0)

# 非常停止設定クラス
class EmergencySwitch(GPIOSetting):
    def __init__(self):
        super().__init__()
        self.emg_pin = 25
        self._setPinOutput(self.emg_pin)
        self.switchOff()    # 初期状態ではOFFとする
    
    # 非常停止ON
    def switchOn(self):
        self._OutHigh(self.emg_pin)
    
    # 非常停止OFF
    def switchOff(self):
        self._OutLow(self.emg_pin)

# 運転準備設定クラス
class RunPrepare(GPIOSetting):
    def __init__(self):
        super().__init__()
        self.switch_pin = 17    # 運転準備スイッチ
        self.signal_pin = 18    # 運転準備完了
        self._setPinOutput(self.switch_pin)
        self._setPinOutput(self.signal_pin)
    
    # スイッチON
    def switchOn(self):
        self._OutHigh(self.switch_pin)
    
    # スイッチOFF
    def switchOff(self):
        self._OutLow(self.switch_pin)

    # 準備完了ON
    def signalOn(self):
        self._OutHigh(self.signal_pin)
    
    # 準備完了OFF
    def signalOff(self):
        self._OutLow(self.signal_pin)

# シグナルランプ設定クラス
class Signal(GPIOSetting):
    def __init__(self):
        super.__init__()
        self.blue_pin = 27      # シグナルランプ：青
        self.yellow_pin = 22    # シグナルランプ：黃
        self.red_pin = 23       # シグナルランプ：赤
        # 各色ランプのGPIO出力設定
        self._setPinOutput(self.blue_pin)
        self._setPinOutput(self.yellow_pin)
        self._setPinOutput(self.red_pin)
    
    # 全ランプ消灯
    def __lightAllOff(self):
        self._OutLow(self.blue_pin)
        self._OutLow(self.yellow_pin)
        self._OutLow(self.red_pin)

    # ランプ点灯
    def __lightOn(self, pin_no):
        self._OutHigh(pin_no)

    # 青色ランプ点灯（動作状態）
    def blueOn(self):
        self.__lightAllOff()
        self.__lightOn(self.blue_pin)

    # 黄色ランプ点灯（待機(初期)状態）
    def yellowOn(self):
        self.__lightAllOff()
        self.__lightOn(self.yellow_pin)

    # 赤色ランプ点灯（非常停止状態）
    def redOn(self):
        self.__lightAllOff()
        self.__lightOn(self.red_pin)
