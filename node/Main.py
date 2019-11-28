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
# ローカル
import conf as c
import MyStat as s
# メッセージのヘッダファイル
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
    SELFNODE = "main"
    # トピック名
    SELFTOPIC = "srv_" + SELFNODE

    def __init__(self, arg_verbose=False):
        """
        コンストラクタ
        """
        # ノードの初期化と名称設定
        rospy.init_node(self.SELFNODE)
        # ステータスクラスの初期化
        self.__s = s.MyStat(self.SELFNODE, arg_verbose)
        # 初期化完了
        self.__s.message("[%s] Do...%s" % (
            os.path.basename(__file__),
            ("(verbose)" if arg_verbose else "")), s.INFO)

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
        grove_ad = rospy.ServiceProxy(
            'srv_grove_ad', grove_ad_srv)
        mon_sigtowerstat = rospy.ServiceProxy(
            'srv_mon_sigtowerstat', mon_sigtowerstat_srv)
        mon_workerstat = rospy.ServiceProxy(
            'srv_mon_workerstat', mon_workerstat_srv)
        storage_st = rospy.ServiceProxy(
            'srv_storage_st', s_sigtower_srv)
        storage_ws = rospy.ServiceProxy(
            'srv_storage_ws', s_workerstat_srv)
        storage_ct = rospy.ServiceProxy(
            'srv_storage_ct', s_ct_srv)
        # メインループ
        try:
            #
            # ステップ１：設備シグナルタワーの状態
            #
            # 問い合わせ
            sts = mon_sigtowerstat()
            self.__s.message("< SigTow > : R %s, Y %s, G %s, B %s, W %s" %
                             (sts.Red, sts.Yellow, sts.Green, sts.Blue, sts.White))
            # データベース書き込み
            sst = storage_st(sts.Red, sts.Yellow,
                             sts.Green, sts.Blue, sts.White)
            #
            # ステップ２：作業者の状態
            #
            ws = mon_workerstat()
            self.__s.message("< Worker > : R %s, Y %s, G %s, B %s, W %s, U %s" %
                             (ws.Red, ws.Yellow, ws.Green, ws.Blue, ws.White, ws.Umber))
            # データベース書き込み
            sws = storage_ws(ws.Red, ws.Yellow, ws.Green,
                             ws.Blue, ws.White, ws.Umber)
            #
            # ステップ３：CTの電流値
            #
            # AD0値を読み込み（盤2・3号はCTセンサはAD0chに入力）
            ad = grove_ad(0)
            self.__s.message("< CT     > : %s (x 0.01V)" % (ad.Val_VOLT))
            # データベース書き込み
            sct = storage_ct(0, ad.Val_VOLT)
            #
            # ステップ４：ストレージに書き込み
            #
            pass
        except:
            # 例外メッセージ
            self.__s.message(sys.exc_info()[0], s.WARN)
        finally:
            pass


if __name__ == '__main__':
    """
    メイン
    """

    # インスタンスを生成
    mp = MainProc()
    # 処理周期Hz
    r = rospy.Rate(c.MAIN_INTERVAL)
    try:
        # 繰り返し
        while not rospy.is_shutdown():
            mp.do()
            r.sleep()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        # # シグナルタワーの色変更（全消し）
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
