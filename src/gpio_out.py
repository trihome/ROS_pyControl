#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# GPIO Output
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import os
import rospy
import rosparam
import RPi.GPIO as GPIO

# 生成したメッセージのヘッダファイル
from py_control.msg import gpio_mes


class Gpio_Out:
    # ノード名
    SelfNode = "gpio_out"
    # トピック名
    SelfTopic = "mes_gpio_out"
    # GPIO出力対象のBCM番号
    GpioPin = []

    def __init__(self, argPin):
        """
        コンストラクタ
        Parameters
        ----------
        """
        # 初期化
        rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
        self.GpioPin = argPin
        self.initGpio()
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))
        # 指定のトピックからメッセージを受信
        rospy.Subscriber(self.SelfTopic, gpio_mes, self.callback_output)

    def __del__(self):
        """
        デストラクタ
        Parameters
        ----------
        """
        #GPIOを解放
        GPIO.cleanup()

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
            GPIO.setup(i, GPIO.OUT)

    def callback_output(self, message):
        """
        コールバック関数
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        rospy.logdebug("Port(OUT): %s, Val: %s" % (message.port, message.value))
        # 受け取ったポート番号が、配列長を超えていないこと
        if message.port < len(self.GpioPin):
            if message.value == 1:
                # 指定の番号をON
                GPIO.output(self.GpioPin[message.port], GPIO.HIGH)
            else:
                # 指定の番号をOFF
                GPIO.output(self.GpioPin[message.port], GPIO.LOW)
        else:
            rospy.logwarn("Port %s is not found." % (message.port))


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        # インスタンスを生成
        gpo = Gpio_Out([26, 19, 13, 6, 5, 22, 27, 17])
        # プロセス終了までアイドリング
        rospy.spin()
    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
