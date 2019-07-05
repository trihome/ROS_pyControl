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
import time
import threading
import rospy
import rosparam
import RPi.GPIO as GPIO
from grove.adc import ADC

# 生成したメッセージのヘッダファイル
from py_control.srv import grove_ad_srv, grove_ad_srvResponse


class Grove_AD:
    # ノード名
    SelfNode = "grove_ad"
    # トピック名
    SelfTopic = "grove_ad_val"
    # AD変換の最大チャネル数
    AD_ChMax = 7
    # 現在のAD電圧値リスト
    L_Ad_Volt = []
    # AD変換の読み込み周期
    _PROCINTERVAL = 0.4

    def __init__(self):
        """
        コンストラクタ
        Parameters
        ----------
        """
        # 初期化
        rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))
        # サービスの登録
        self.service = rospy.Service(self.SelfTopic, grove_ad_srv, self.handle)

    def handle(self, request):
        """
        サービス
        Parameters
        ----------
        """
        #指定のチャネルが配列のサイズ以内か確認
        if request.Ch < len(self.L_Ad_Volt):
            # ログの表示
            rospy.logdebug("Grove AD [%sch]: %s" %
                           (request.Ch, self.L_Ad_Volt[request.Ch]))
            return grove_ad_srvResponse(self.L_Ad_Volt[request.Ch])
        else:
            rospy.logwarn("Grove AD ch out of range.")

    def thread_do(self):
        """
        スレッドのスタート
        Parameters
        ----------
        """
        # スレッドをデーモンモードで開始
        thread_obj = threading.Thread(target=self.thread)
        thread_obj.setDaemon(True)
        thread_obj.start()

    def thread(self):
        """
        スレッド：GroveADの読み込み
        Parameters
        ----------
        """
        while True:
            adc = ADC()
            # 一時格納リスト
            L_Ad_Volt_buf = []
            # 全チャネル読み込み
            for i in range(0, self.AD_ChMax):
                val = 0
                try:
                    # 読み込み
                    val = adc.read_voltage(i)
                except:
                    # エラーの時は強制-1
                    val = -1
                    rospy.logwarn("Grove AD %s ch error." % i)
                # 一時格納リストに追加
                L_Ad_Volt_buf.append(val)
            #スライスを使って、AD電圧値リストにコピー
            self.L_Ad_Volt = L_Ad_Volt_buf[:]
            #正常更新    
            rospy.logdebug("Grove AD Updated. (%s channels)" % len(self.L_Ad_Volt))
            # スリープ
            time.sleep(self._PROCINTERVAL)


if __name__ == '__main__':
    """
    メイン
    """

    try:
        # インスタンスを生成
        gad = Grove_AD()
        # スレッドのスタート
        gad.thread_do()
        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
