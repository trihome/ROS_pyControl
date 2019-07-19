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
# ローカル
import conf as c
import MyStat as s
# メッセージのヘッダファイル
from py_control.srv import mon_workerstat_srv, mon_workerstat_srvResponse
from py_control.msg import gpio_mes
from py_control.msg import mon_workerstat_mes


class Mon_WorkerStat:
    """
    作業者状態ボタンの処理
    """
    # ノード名
    SELFNODE = "mon_workerstat"
    # トピック名
    SELFTOPIC = "srv_" + SELFNODE

    # 作業者ボタンの状態定義
    # - 未検知
    STAT_NONE = -1
    # - OFF
    STAT_OFF = 0
    # - ON
    STAT_ON = 1

    # 現在の作業者ボタンの状態
    # 0:RED  1:YELLOW  2:GREEN  3:BLUE  4:WHITE  5:UMBER
    __worker_stat = [STAT_NONE, STAT_NONE,
                     STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE]

    # 作業者ステータス受付の有効・無効化
    __enabled = True

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
        # ノードの初期化と名称設定
        rospy.init_node(self.SELFNODE)
        # ピン番号の設定
        self.GpioPin = argPin
        # 自身のサービスの登録
        self.service = rospy.Service(
            "srv_" + self.SELFNODE, mon_workerstat_srv, self.handle)
        # 指定のトピックからメッセージを受信：ボタン入力
        rospy.Subscriber("mes_gpio_in", gpio_mes, self.callback_button)
        # 指定のトピックからメッセージを受信：有効／無効
        rospy.Subscriber("mes_mon_workerstat",
                         mon_workerstat_mes, self.callback_enable)
        # 初期化完了
        self.__s.message("[%s] Do...%s" % (
            os.path.basename(__file__),
            ("(verbose)" if arg_verbose else "")), s.INFO)

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
            self.__worker_stat[0], self.__worker_stat[1], self.__worker_stat[2],
            self.__worker_stat[3], self.__worker_stat[4], self.__worker_stat[5])

    def callback_button(self, arg_mes):
        """
        ボタン入力を受け取るコールバック関数
        Parameters
        ----------
        message : gpio_mes
            メッセージ
        """
        # 無効化されているときは、以降の処理をしない
        if not self.__enabled:
            return
        # ボタン受付処理
        try:
            #
            # WORKSTATボタンの点灯を更新
            #
            # 指定のトピックへメッセージを送信：ボタンランプ
            pub = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
            # 送信するメッセージの作成
            msg = gpio_mes()
            stat = []
            logmes = "<WorkStat PB P:V, PL P:V> "
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
                logmes += ("[ %s:%s,%s:%s ], " %
                           (arg_mes.port, arg_mes.value, msg.port, msg.value))
            # ログの表示
            self.__s.message(logmes)
        except:
            # 例外発生時
            self.__s.message("Grove AD Measure Error: %s" %
                             (sys.exc_info()[0]), s.WARN)
        finally:
            pass
        #
        # 色ボタンごとのステータスの更新
        # 0:RED  1:GREEN  2:YELLOW  3:UMBER  4:BLUE  5:WHITE
        #
        i = 0
        for j in c.GPIO_WORKERSTATORDER:
            if j == "RED":
                self.__worker_stat[0] = stat[i]
            if j == "YELLOW":
                self.__worker_stat[1] = stat[i]
            if j == "GREEN":
                self.__worker_stat[2] = stat[i]
            if j == "BLUE":
                self.__worker_stat[3] = stat[i]
            if j == "WHITE":
                self.__worker_stat[4] = stat[i]
            if j == "UMBER":
                self.__worker_stat[5] = stat[i]
            else:
                pass
            i += 1

    def callback_enable(self, arg_mes):
        """
        ボタン入力を受け取るコールバック関数
        Parameters
        ----------
        message : mon_workerstat_mes
            メッセージ
        """
        if arg_mes.enable:
            # 有効化
            self.__enabled = True
            self.__s.message("WorkerStat Enabled.")
        else:
            # 無効化
            self.__enabled = False
            self.StatusReset()
            self.__s.message("WorkerStat Disabled.")

    def StatusReset(self):
        """
        ステータスを全てリセット
        """
        # 値をリセット
        for i in range(len(self.__worker_stat)):
            self.__worker_stat[i] = self.STAT_NONE
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
            # 有効なボタンだけゼロリセット
            self.__worker_stat[i] = self.STAT_OFF


if __name__ == '__main__':
    """
    メイン
    """

    # インスタンスを生成
    mws = Mon_WorkerStat(c.GPIO_IN)
    try:
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
