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

# 生成したメッセージのヘッダファイル
from py_control.msg import gpio_mes

# GPIO入力対象のBCM番号
Pin = [21, 20, 16, 12, 7, 8, 25, 24]

global pub


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
        GPIO.setup(i, GPIO.IN)
        # イベント（立ち上がりを拾う）
        GPIO.add_event_detect(i, GPIO.RISING, bouncetime=100)
        # スイッチ入力端子の状態ををcallbackのトリガとして指定
        GPIO.add_event_callback(i, switch_callback)


def switch_callback(gpio_pin):
    """
    スイッチ押下のコールバック関数
    Parameters
    ----------
    gpio_pin : int
        ピン番号
    """
    # 送信するメッセージの作成
    msg = gpio_mes()
    msg.port = gpio_pin
    msg.value = 1
    rospy.loginfo("Port (IN): %s, Val: %s" % (msg.port, msg.value))

    # 送信
    global pub
    pub.publish(msg)


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """

    # 初期化
    rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
    try:
        # 実行
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))

        initGpio()

        # ノードの初期化と名称設定
        # anonymousをTrueにして、ノード名を自動で変更して複数接続OKとする
        rospy.init_node('gpio_in', anonymous=True)

        # 指定のトピックからメッセージを送信
        pub = rospy.Publisher('mes_gpio_in', gpio_mes, queue_size=10)
        # 送信周期(Hz)を設定
        rate = rospy.Rate(2)

        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
