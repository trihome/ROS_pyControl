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

import datetime
import os
import Queue
import rospy
import rosparam
import sqlite3
import time
import threading
# ローカル
import DbMssql
import DbSqlite
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

    # キュー
    qworkerstat = Queue.Queue()
    qsigtower = Queue.Queue()
    qct = Queue.Queue()

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
        # 現在時刻を取得
        dtnow = datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        # 文字の組み立てとレコードの追加
        if request.White >= 0:
            state = ("%s%s%s%s%s" %
                     (request.White, request.Blue, request.Green, request.Yellow, request.Red))
            # キューに追加
            self.qsigtower.put([state, 5, dtnow])
            # self.A_SigTower(state, 5)
        elif request.Blue >= 0:
            state = ("%s%s%s%s" %
                     (request.Blue, request.Green, request.Yellow, request.Red))
            self.qsigtower.put([state, 4, dtnow])
            # self.A_SigTower(state, 4)
        elif request.Green >= 0:
            state = ("%s%s%s" % (request.Green, request.Yellow, request.Red))
            self.qsigtower.put([state, 3, dtnow])
            # self.A_SigTower(state, 3)
        elif request.Yellow >= 0:
            state = ("%s%s" % (request.Yellow, request.Red))
            self.qsigtower.put([state, 2, dtnow])
            # self.A_SigTower(state, 2)
        elif request.Red >= 0:
            state = ("%s" % (request.Red))
            self.qsigtower.put([state, 1, dtnow])
            # self.A_SigTower(state, 1)
        # ログ
        rospy.loginfo(" * queue : SigTower [%s] > %s" % (0, state))
        return s_sigtower_srvResponse(0)

    def handle_ct(self, request):
        """
        問い合わせを受けてCT測定値を書き込み
        Parameters
        ----------
        request : s_ct_srv
            メッセージ
        """
        # 現在時刻を取得
        dtnow = datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        # キューに追加
        self.qct.put([str(request.Volt), dtnow])
        # 文字の組み立てとレコードの追加
        # self.A_Ct(str(request.Volt))
        # ログ
        rospy.loginfo(" * queue : Ct [%s] > %s" % (0, str(request.Volt)))
        return s_ct_srvResponse(0)

    def handle_ws(self, request):
        """
        問い合わせを受けてCT測定値を書き込み
        Parameters
        ----------
        request : s_ct_srv
            メッセージ
        """
        # 現在時刻を取得
        dtnow = datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        # 文字の組み立てとレコードの追加
        state = ("%s%s%s%s" %
                 (request.Red, request.Yellow, request.Green, request.Blue))
        # キューに追加
        self.qworkerstat.put([state, dtnow])
        # self.A_Ws(state)
        # ログ
        rospy.loginfo(" * queue : Ws [%s] > %s" % (0, state))
        return s_workerstat_srvResponse(0)

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
        # インスタンス生成
        self.__dbm = DbMssql.DbMssql()
        self.__dbs = DbSqlite.DbSqlite()
        # データベースへの書き込み処理
        while True:
            #
            # SQLServerの生死確認
            #
            if self.__dbm.is_connectable() == False:
                rospy.logwarn("SQLServer is dead.")
                time.sleep(c.DB_MSSQL_APPEND_INTERVAL)
                # ホストから応答無いときは以降の処理をしない
                continue
            #
            # STEP1:シグナルタワー
            #
            # CCM識別子の定義
            ccm = ["SigTower1.mLI", "SigTower2.mLI",
                   "SigTower3.mLI", "SigTower4.mLI", "SigTower5.mLI"]
            # キューを順次呼び出し
            while not self.qsigtower.empty():
                buf = self.qsigtower.get()
                # SQLServerに登録
                self.add_record(ccm[buf[1]], buf[0], buf[2])

            #
            # STEP2:作業者状態
            #
            # キューを順次呼び出し
            while not self.qworkerstat.empty():
                buf = self.qworkerstat.get()
                # SQLServerに登録
                self.add_record("WorkerStat4.mLI", buf[0], buf[1])
            #
            # STEP3:CT
            #
            # キューを順次呼び出し
            while not self.qct.empty():
                buf = self.qct.get()
                # SQLServerに登録
                self.add_record("CT.mLI", buf[0], buf[1])
            #
            # STEP4:書き込みできなかったデータを再書き込み
            #
            pass
            #
            # 次の処理までウェイト
            #
            time.sleep(c.DB_MSSQL_APPEND_INTERVAL)

    def add_record(self, arg_CCM, arg_val, arg_date):
        """
        レコードの追加
        """
        ret = self.__dbm.add_to_table(("%s" % os.uname()[1]),
                                      arg_val,
                                      ("%s" % (arg_CCM)),
                                      cl._DETECT_LOCAL_ROOM,
                                      cl._DETECT_LOCAL_REGION,
                                      cl._DETECT_LOCAL_ORDER, 30, "",
                                      arg_date)
        # 戻り値確認
        if ret < 0:
            # 負の値の時は書き込み失敗
            #SQLiteに退避
            self.__dbs.add_to_table([arg_date, arg_CCM, arg_val])
            rospy.logwarn("data saving")
        else:
            # 正常書き込み
            rospy.loginfo(" * sql   : [%s] (%s) > %s" %
                          (arg_CCM, arg_date, arg_val))
        return ret


if __name__ == '__main__':
    """
    メイン
    """

    try:
        # インスタンスを生成
        sr = Storage()
        # スレッドスタート
        sr.thread_init()
        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
