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

# GPIO出力対象のBCM番号
Pin = [26, 19, 13, 6, 5, 22, 27, 17]


def initGpio():
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
    for i in Pin:
        GPIO.setup(i, GPIO.OUT)


def callback(message):
    """
    コールバック関数
    Parameters
    ----------
    message : gpio_mes
        メッセージ
    """

    # ログの表示
    rospy.loginfo("Port(OUT): %s, Val: %s" % (message.port, message.value))

    # 受け取ったポート番号が、配列長を超えていないこと
    if message.port < len(Pin):
        if message.value == 1:
            # 指定の番号をON
            GPIO.output(Pin[message.port], GPIO.HIGH)
        else:
            # 指定の番号をOFF
            GPIO.output(Pin[message.port], GPIO.LOW)
    else:
        rospy.logwarn("Port %s is not found." % (message.port))


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """

    # 初期化
    rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
    initGpio()

    try:
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))

        # ノードの初期化と名称設定
        rospy.init_node('gpio_out')

        # 指定のトピックからメッセージを受信
        rospy.Subscriber("mes_gpio_out", gpio_mes, callback)

        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        GPIO.cleanup()
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
