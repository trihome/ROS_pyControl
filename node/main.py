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
        # 処理周期(Hz)を設定
        rate = rospy.Rate(0.1)
        # シグナルタワーの色変更
        self.UpdateSigTower(2)
        # メインループ
        while not rospy.is_shutdown():
            try:
                #
                # ステップ１：設備シグナルタワーの状態
                #
                sts = mon_sigtowerstat()
                rospy.loginfo("< SigTow > : R %s, Y %s, G %s, B %s, W %s" %
                              (sts.Red, sts.Yellow, sts.Green, sts.Blue, sts.White))
                #
                # ステップ２：作業者の状態
                #
                ws = mon_workerstat()
                rospy.loginfo("< Worker > : R %s, G %s, Y %s, U %s, B %s, W %s" %
                              (ws.Red, ws.Green, ws.Yellow, ws.Umber, ws.Blue, ws.White))
                #
                # ステップ３：CTの電流値
                #
                ad = grove_ad(0)
                rospy.loginfo("< CT     > : %s" % (ad.Val_VOLT))
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
        thread_obj = threading.Thread(target=self.thread)
        thread_obj.setDaemon(True)
        thread_obj.start()

    def thread(self):
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
