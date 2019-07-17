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
# ローカル
import conf as c
import MyStat as s
# メッセージのヘッダファイル
from py_control.srv import grove_ad_srv, grove_ad_srvResponse


class Grove_AD:
    # ノード名
    SELFNODE = "grove_ad"
    # トピック名
    SELFTOPIC = "srv_" + SELFNODE
    # 現在のAD電圧値リスト
    __L_Ad_Volt = []
    # # AD変換の読み込み周期
    # _PROCINTERVAL = 0.4

    def __init__(self, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        arg_verbose:bool
            メッセージの強制表示
        """
        # ステータスクラスの初期化
        self.__s = s.MyStat(self.SELFNODE, arg_verbose)
        # ノードの初期化と名称設定
        rospy.init_node(self.SELFNODE)
        # サービスの登録
        self.service = rospy.Service(self.SELFTOPIC, grove_ad_srv, self.handle)
        # 初期化完了
        self.__s.message("[%s] Do...%s" % (
            os.path.basename(__file__),
            ("(verbose)" if arg_verbose else "")), s.INFO)

    def handle(self, request):
        """
        指定chの問い合わせを受けたらAD値を回答
        Parameters
        ----------
        request : grove_ad_srv
            メッセージ
        """
        # 指定のチャネルが配列に含まれているか確認
        if request.Ch in c.AD_IN:
            # ログの表示
            self.__s.message("Grove AD [%sch]: %s" %
                             (request.Ch, self.__L_Ad_Volt[c.AD_IN.index(request.Ch)]))
            #ADの値を返す                             
            return grove_ad_srvResponse(self.__L_Ad_Volt[c.AD_IN.index(request.Ch)])
        else:
            # chの範囲を超えたらエラー
            self.__s.message(
                "Grove AD ch [%sch] out of range." % request.Ch, s.WARN)

    def do(self):
        """
        GroveADの読み込み処理
        """
        adc = ADC()
        # 一時格納リスト
        L_Ad_Volt_buf = []
        # 全チャネル読み込み
        for i in c.AD_IN:
            val = 0
            try:
                # 読み込み
                val = adc.read_voltage(i)
            except:
                # エラーの時は強制-1
                val = -1
                self.__s.message("Grove AD %s ch error." % i, s.WARN)
            # 一時格納リストに追加
            L_Ad_Volt_buf.append(val)
        # スライスを使って、AD電圧値リストにコピー
        self.__L_Ad_Volt = L_Ad_Volt_buf[:]
        # 正常更新
        self.__s.message("Grove AD Updated. (%s channels) : %s" %
                       (len(self.__L_Ad_Volt), self.__L_Ad_Volt))


if __name__ == '__main__':
    """
    メイン
    """

    # インスタンスを生成
    gad = Grove_AD()
    # 処理周期Hz
    r = rospy.Rate(c.GROVE_AD_DETECT_INTERVAL)
    try:
        # 繰り返し
        while not rospy.is_shutdown():
            gad.do()
            r.sleep()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
