#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# GPIO Input
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import os
import rospy
import rosparam
import RPi.GPIO as GPIO
import conf as c

# 生成したメッセージのヘッダファイル
from py_control.msg import gpio_mes


class Gpio_In:
    """
    GPIO入力
    """
    # ノード名
    SelfNode = "gpio_in"
    # トピック名
    SelfTopic = "mes_gpio_in"
    # GPIO入力対象のBCM番号
    GpioPin = []

    def __init__(self, argPin):
        """
        コンストラクタ
        Parameters
        ----------
        argPin : int
            入力ピン番号
        """
        # 初期化
        rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
        # ピン番号の設定
        self.GpioPin = argPin
        # GPIOの入力初期化
        self.initGpio()
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))
        # 指定のトピックからメッセージを送信
        self.pub = rospy.Publisher(self.SelfTopic, gpio_mes, queue_size=10)
        # 送信周期(Hz)を設定
        self.rate = rospy.Rate(2)

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def initGpio(self):
        """
        GPIOの初期化
        Parameters
        ----------
        """
        # GPIOへアクセスする番号をBCMの番号で指定することを宣言
        GPIO.setmode(GPIO.BCM)
        # ワーニングの抑制
        GPIO.setwarnings(False)
        # BCMのn番ピンを出力に設定
        for i in self.GpioPin:
            GPIO.setup(i, GPIO.IN)
            # イベント（立ち上がりを拾う）
            GPIO.add_event_detect(i, GPIO.RISING, bouncetime=100)
            # スイッチ入力端子の状態ををcallbackのトリガとして指定
            GPIO.add_event_callback(i, self.callback_input)

    def callback_input(self, gpio_pin):
        """
        スイッチ押下のコールバック関数
        Parameters
        ----------
        gpio_pin : int
            ピン番号
        """
        # 送信するメッセージの作成
        msg = gpio_mes()
        # BCMポート番号をX端子番号に読み替え
        msg.port = self.GpioPin.index(gpio_pin)
        msg.value = 1
        # 送信
        self.pub.publish(msg)
        # ログの表示
        rospy.logdebug("Port (IN): %s, Val: %s" % (msg.port, msg.value))


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """
    # 初期化
    rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
    try:
        # インスタンスを生成
        gpo = Gpio_In(c.GPIO_IN)
        # プロセス終了までアイドリング
        rospy.spin()
    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        # GPIOを解放
        GPIO.cleanup()
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
