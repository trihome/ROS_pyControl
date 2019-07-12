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
import sys
import conf as c

# 生成したメッセージのヘッダファイル
from py_control.msg import gpio_mes
from py_control.srv import mon_workerstat_srv, mon_workerstat_srvResponse
from py_control.srv import mon_sigtowerstat_srv, mon_sigtowerstat_srvResponse
from py_control.srv import grove_ad_srv, grove_ad_srvResponse
from py_control.srv import s_ct_srv, s_ct_srvResponse
from py_control.srv import s_sigtower_srv, s_sigtower_srvResponse
from py_control.srv import s_workerstat_srv, s_workerstat_srvResponse


class MainProc:
    """
    設備シグナルタワー状態検知の処理
    """
    # ノード名
    SelfNode = "main"
    # トピック名
    SelfTopic = "" + SelfNode

    def __init__(self):
        """
        コンストラクタ
        """
        # 初期化
        rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def do(self):
        """
        主処理
        """
        # ローカルプロキシにサービス名と型を設定
        grove_ad = rospy.ServiceProxy('srv_grove_ad', grove_ad_srv)
        mon_sigtowerstat = rospy.ServiceProxy(
            'srv_mon_sigtowerstat', mon_sigtowerstat_srv)
        mon_workerstat = rospy.ServiceProxy(
            'srv_mon_workerstat', mon_workerstat_srv)
        storage_st = rospy.ServiceProxy('srv_storage_st', s_sigtower_srv)
        storage_ws = rospy.ServiceProxy('srv_storage_ws', s_workerstat_srv)
        storage_ct = rospy.ServiceProxy('srv_storage_ct', s_ct_srv)
        # 処理周期(Hz)を設定
        rate = rospy.Rate(c.MAIN_INTERVAL)
        # シグナルタワーの色変更
        self.UpdateSigTower(2)
        # メインループ
        while not rospy.is_shutdown():
            try:
                #
                # ステップ１：設備シグナルタワーの状態
                #
                #問い合わせ
                sts = mon_sigtowerstat()
                rospy.loginfo("< SigTow > : R %s, Y %s, G %s, B %s, W %s" %
                              (sts.Red, sts.Yellow, sts.Green, sts.Blue, sts.White))
                #データベース書き込み
                sst = storage_st(sts.Red, sts.Yellow, sts.Green, sts.Blue, sts.White)
                #
                # ステップ２：作業者の状態
                #
                ws = mon_workerstat()
                rospy.loginfo("< Worker > : R %s, Y %s, G %s, B %s, W %s, U %s" %
                              (ws.Red, ws.Yellow, ws.Green, ws.Blue, ws.White, ws.Umber))
                #データベース書き込み
                sws = storage_ws(ws.Red, ws.Yellow, ws.Green, ws.Blue, ws.White, ws.Umber)
                #
                # ステップ３：CTの電流値
                #
                ad = grove_ad(0)
                rospy.loginfo("< CT     > : %s (x 0.01V)" % (ad.Val_VOLT))
                #データベース書き込み
                sct = storage_ct(0, ad.Val_VOLT)
                #
                # ステップ４：ストレージに書き込み
                #
                pass
            except:
                rospy.logwarn(sys.exc_info()[0])
                # シグナルタワーの色変更
                self.UpdateSigTower(0)
            finally:
                pass
                # ウェイト
            rate.sleep()

    def UpdateSigTower(self, arg_Color):
        """
        自分のシグナルタワーの状態を更新
        """
        #
        rospy.loginfo("ST: %s" % arg_Color)
        # 指定のトピックへメッセージを送信
        pub = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
        # 送信するメッセージの作成
        msg = gpio_mes()
        for i in c.GPIO_OUT_SIGTOWER:
            # 点灯状態を変更するポート番号(X番号)を指定
            msg.port = i
            if i == arg_Color:
                # callbackで受け取ったポート番号(X番号)と一致していれば
                # 指定のランプをON
                msg.value = 1
            else:
                # それ以外のランプをOFF
                msg.value = 0
            # メッセージ送信
            pub.publish(msg)

    def thread_init(self):
        """
        スレッドのスタート
        """
        # スレッドをデーモンモードで開始
        thread_obj = threading.Thread(target=self.thread_do)
        thread_obj.setDaemon(True)
        thread_obj.start()

    def thread_do(self):
        """
        スレッド
        """
        # 処理メインループ
        while True:
            # 次の測定までウェイト
            time.sleep(c.SIGTOWER_DETECT_INTERVAL)


if __name__ == '__main__':
    """
    メイン
    """

    # インスタンスを生成
    mp = MainProc()
    try:
        # スレッドのスタート
        # mp.thread_init()
        # メイン処理のスタート
        mp.do()
        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        # シグナルタワーの色変更（全消し）
        mp.UpdateSigTower(99)
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
