#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Monitor the status of workers
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import os
import time
import threading
import rospy
import rosparam
import conf as c

# 生成したメッセージのヘッダファイル
from py_control.srv import mon_workerstat_srv, mon_workerstat_srvResponse
from py_control.msg import gpio_mes


class Mon_WorkerStat:
    """
    作業者状態ボタンの処理
    """
    # ノード名
    SelfNode = "mon_workerstat"
    # トピック名
    SelfTopic = "srv_" + SelfNode
    # GPIO入力対象のBCM番号
    GpioPin = []

    # 作業者ボタンの状態定義
    # - 未検知
    STAT_NONE = -1
    # - OFF
    STAT_OFF = 0
    # - ON
    STAT_ON = 1

    # 現在の作業者ボタンの状態
    # 0:RED  1:YELLOW  2:GREEN  3:BLUE  4:WHITE  5:UMBER  
    Stat = [STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE]

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
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        # ピン番号の設定
        self.GpioPin = argPin
        # 自身のサービスの登録
        self.service = rospy.Service(
            self.SelfTopic, mon_workerstat_srv, self.handle)
        # 指定のトピックからメッセージを受信：ボタン入力
        rospy.Subscriber("mes_gpio_in", gpio_mes, self.callback)

        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def handle(self, request):
        """
        問い合わせを受けたら作業者ステータスを回答
        Parameters
        ----------
        request : grove_ad_srv
            メッセージ
        """
        # 各ランプの状態を回答
        return mon_workerstat_srvResponse(
            self.Stat[0], self.Stat[1], self.Stat[2],
            self.Stat[3], self.Stat[4], self.Stat[5])

    def callback(self, arg_mes):
        """
        ボタン入力を受け取るコールバック関数
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # WORKSTATボタンの点灯を更新
        # 指定のトピックへメッセージを送信：ボタンランプ
        pub = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
        # 送信するメッセージの作成
        msg = gpio_mes()
        stat = []
        # GPIO_OUT_WORKERSTATをスキャン
        for i in c.GPIO_OUT_WORKERSTAT:
            # 点灯状態を変更するポート番号(X番号)を指定
            msg.port = i
            if i == arg_mes.port:
                # callbackで受け取ったポート番号(X番号)と一致していれば
                # 指定のランプをON
                msg.value = self.STAT_ON
            else:
                # それ以外のランプをOFF
                msg.value = self.STAT_OFF
            # メッセージ送信
            pub.publish(msg)
            # ステータスバッファに追加
            stat.append(msg.value)
            # ログの表示
            rospy.logdebug("PB Port:%s, Val:%s / PL Port:%s, Val:%s" %
                           (arg_mes.port, arg_mes.value, msg.port, msg.value))
        # 色ボタンごとのステータスの更新
        # 0:RED  1:GREEN  2:YELLOW  3:UMBER  4:BLUE  5:WHITE
        i = 0
        for j in c.GPIO_WORKERSTATORDER:
            if j == "RED":
                self.Stat[0] = stat[i]
            if j == "YELLOW":
                self.Stat[1] = stat[i]
            if j == "GREEN":
                self.Stat[2] = stat[i]
            if j == "BLUE":
                self.Stat[3] = stat[i]
            if j == "WHITE":
                self.Stat[4] = stat[i]
            if j == "UMBER":
                self.Stat[5] = stat[i]
            else:
                pass
            i += 1

    def StatusReset(self):
        """
        ステータスを全てリセット（上手く動いてない）
        """
        # 値をリセット
        for i in range(len(self.Stat)):
            self.Stat[i] = self.STAT_NONE
        # ボタン点灯もリセット
        for i in c.GPIO_OUT_WORKERSTAT:
            # 指定のトピックへメッセージを送信：ボタンランプ
            pub = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
            # 送信するメッセージの作成
            msg = gpio_mes()
            msg.port = i
            msg.value = self.STAT_OFF
            # メッセージ送信
            pub.publish(msg)


if __name__ == '__main__':
    """
    メイン
    """

    try:
        # インスタンスを生成
        mws = Mon_WorkerStat(c.GPIO_IN)
        # 初期化
        mws.StatusReset()
        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
