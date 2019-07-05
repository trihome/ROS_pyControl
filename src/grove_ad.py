#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# AD Converter for Seeed Grove Base Hat
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import os
import rospy
import rosparam
import RPi.GPIO as GPIO
from grove.adc import ADC

# 生成したメッセージのヘッダファイル
from py_control.srv import grove_ad_srv, grove_ad_srvResponse


def handle(request):
    """
    サービス
    Parameters
    ----------
    """
    if request.Ch in range(0, 7):
        # ADの値を読み込み
        adc = ADC()
        val_v = adc.read_voltage(request.Ch)
        # ログの表示
        rospy.loginfo("Grove AD [%sch]: %s" % (request.Ch, val_v))
        # ADの値を返す
        return grove_ad_srvResponse(val_v)


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """

    # 初期化
    rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))

    try:
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))

        # ノードの初期化と名称設定
        rospy.init_node('grove_ad')

        service = rospy.Service('grove_ad_val', grove_ad_srv, handle)

        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
