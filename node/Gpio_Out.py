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
# ローカル
import conf as c
import MyStat as s
# メッセージのヘッダファイル
from py_control.msg import gpio_mes


class Gpio_Out:
    """
    GPIO出力
    """
    # ノード名
    SELFNODE = "gpio_out"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE
    # GPIO出力対象のBCM番号
    __GpioPin = []

    def __init__(self, argPin, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        argPin : int
            入力ピン番号
        arg_verbose:bool
            メッセージの強制表示
        """
        # ステータスクラスの初期化
        self.__s = s.MyStat(self.SELFNODE, arg_verbose)
        # 初期化
        self.__GpioPin = argPin
        self.initGpio()
        # ノードの初期化と名称設定
        rospy.init_node(self.SELFNODE)
        # 指定のトピックからメッセージを受信
        rospy.Subscriber(self.SELFTOPIC, gpio_mes, self.callback_output)
        # 初期化完了
        self.__s.message("[%s] Do...%s" % (
            os.path.basename(__file__),
            ("(verbose)" if arg_verbose else "")), s.INFO)

    def __del__(self):
        """
        デストラクタ
        Parameters
        ----------
        """
        # GPIOを解放
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
        for i in self.__GpioPin:
            GPIO.setup(i, GPIO.OUT)
            # 出力状態をLOWに設定
            GPIO.output(i, GPIO.LOW)

    def callback_output(self, message):
        """
        コールバック関数
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # ログの表示
        self.__s.message("Port(OUT): %s, Val: %s" %
                         (message.port, message.value))
        # 出力の更新
        self.out_update(message.port, message.value)

    def out_update(self, arg_port, arg_val):
        """
        GPIO出力
        Parameters
        ----------
        arg_port : 
            ポート
        arg_val : 
            値
        """
        if arg_port < len(self.__GpioPin):
            # 受け取ったポート番号が、配列長を超えていないこと
            if arg_val == 1:
                # 指定の番号をON
                GPIO.output(self.__GpioPin[arg_port], GPIO.HIGH)
            else:
                # 指定の番号をOFF
                GPIO.output(self.__GpioPin[arg_port], GPIO.LOW)
        elif arg_port == 99:
            # ポート99番を指定されたときは、全ポートを同時操作
            for i in self.__GpioPin:
                if arg_val == 1:
                    # 全てON
                    GPIO.output(i, GPIO.HIGH)
                else:
                    # 全てOFF
                    GPIO.output(i, GPIO.LOW)
        else:
            # それ以外の時はエラー
            self.__s.message("Port %s is not found." % (arg_port), s.WARN)


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        # インスタンスを生成
        gpo = Gpio_Out(c.GPIO_OUT)
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
