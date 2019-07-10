#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Write the collected information to the storage
# (database: SQLServer)
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import os
import time
import threading
import rospy
import rosparam
import DbMssql
import conf as c
import conf_local as cl
# 生成したメッセージのヘッダファイル
from py_control.srv import s_ct_srv, s_ct_srvResponse
from py_control.srv import s_sigtower_srv, s_sigtower_srvResponse
from py_control.srv import s_workerstat_srv, s_workerstat_srvResponse


class Storage:
    """
    ストレージへの書き込み
    """
    # ノード名
    SelfNode = "storage"
    # トピック名
    SelfTopic = "srv_" + SelfNode

    def __init__(self):
        """
        コンストラクタ
        """
        # 初期化
        rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))
        # サービスの登録
        self.service_ct = rospy.Service(
            self.SelfTopic + "_ct", s_ct_srv, self.handle_ct)
        self.service_st = rospy.Service(
            self.SelfTopic + "_st", s_sigtower_srv, self.handle_st)
        self.service_ws = rospy.Service(
            self.SelfTopic + "_ws", s_workerstat_srv, self.handle_ws)

    def handle_st(self, request):
        """
        問い合わせを受けてシグナルタワーステータスを書き込み
        Parameters
        ----------
        request : s_sigtower_srv
            メッセージ
        """
        # 文字の組み立てとレコードの追加
        if request.White >= 0:
            state = ("%s%s%s%s%s" %
                     (request.White, request.Blue, request.Green, request.Yellow, request.Red))
            self.A_SigTower(state, 5)
        elif request.Blue >= 0:
            state = ("%s%s%s%s" %
                     (request.Blue, request.Green, request.Yellow, request.Red))
            self.A_SigTower(state, 4)
        elif request.Green >= 0:
            state = ("%s%s%s" % (request.Green, request.Yellow, request.Red))
            self.A_SigTower(state, 3)
        elif request.Yellow >= 0:
            state = ("%s%s" % (request.Yellow, request.Red))
            self.A_SigTower(state, 2)
        elif request.Red >= 0:
            state = ("%s" % (request.Red))
            self.A_SigTower(state, 1)
        return s_sigtower_srvResponse(0)

    def handle_ct(self, request):
        """
        問い合わせを受けてCT測定値を書き込み
        Parameters
        ----------
        request : s_ct_srv
            メッセージ
        """
        # 文字の組み立てとレコードの追加
        self.A_Ct(str(request.Volt))
        # 各ランプの状態を回答
        return s_ct_srvResponse(0)

    def handle_ws(self, request):
        """
        問い合わせを受けてCT測定値を書き込み
        Parameters
        ----------
        request : s_ct_srv
            メッセージ
        """
        # 文字の組み立てとレコードの追加
        state = ("%s%s%s%s" %
                    (request.Red, request.Yellow, request.Green, request.Blue))
        self.A_Ws(state)
        # 各ランプの状態を回答
        return s_workerstat_srvResponse(0)

    def A_SigTower(self, arg_State, arg_SigCount=3):
        """
        レコードの追加・シグナルタワー
        """
        # CCM識別子
        ccm = ["SigTower1.mLI", "SigTower2.mLI",
               "SigTower3.mLI", "SigTower4.mLI", "SigTower5.mLI"]
        arg_SigCount -= 1
        # インスタンス生成
        __db = DbMssql.DbMssql()
        # SQLServerに登録
        ret = __db.AddToMssql(("%s" % os.uname()[1]),
                              arg_State,
                              ccm[arg_SigCount],
                              cl._DETECT_LOCAL_ROOM,
                              cl._DETECT_LOCAL_REGION,
                              cl._DETECT_LOCAL_ORDER, 30, "")
        # ログ
        rospy.loginfo(" * sql : SigTower [%s] > %s" %
                      (ccm[arg_SigCount], arg_State))
        # データベースの異常があったとき
        if ret < 0:
            # 異常ランプを点滅
            pass

    def A_Ct(self, arg_CtVolt):
        """
        レコードの追加・CT
        """
        # インスタンス生成
        __db = DbMssql.DbMssql()
        # SQLServerに登録
        ret = __db.AddToMssql(("%s" % os.uname()[1]),
                              arg_CtVolt,
                              "CT.mLI",
                              cl._DETECT_LOCAL_ROOM,
                              cl._DETECT_LOCAL_REGION,
                              cl._DETECT_LOCAL_ORDER, 30, "")
        # ログ
        rospy.loginfo(" * sql : Ct > %s" % (arg_CtVolt))
        # データベースの異常があったとき
        if ret < 0:
            # 異常ランプを点滅
            pass

    def A_Ws(self, arg_State):
        """
        レコードの追加・作業者状態
        """
        # インスタンス生成
        __db = DbMssql.DbMssql()
        # SQLServerに登録
        ret = __db.AddToMssql(("%s" % os.uname()[1]),
                              arg_State,
                              "WorkerStat4.mLI",
                              cl._DETECT_LOCAL_ROOM,
                              cl._DETECT_LOCAL_REGION,
                              cl._DETECT_LOCAL_ORDER, 30, "")
        # ログ
        rospy.loginfo(" * sql : Worker > %s" % (arg_State))
        # データベースの異常があったとき
        if ret < 0:
            # 異常ランプを点滅
            pass


if __name__ == '__main__':
    """
    メイン
    """

    try:
        # インスタンスを生成
        sr = Storage()
        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
