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
import MyStat as s
# メッセージのヘッダファイル
from py_control.srv import s_ct_srv, s_ct_srvResponse
from py_control.srv import s_sigtower_srv, s_sigtower_srvResponse
from py_control.srv import s_workerstat_srv, s_workerstat_srvResponse


class Storage:
    """
    ストレージへの書き込み
    """
    # ノード名
    SELFNODE = "storage"
    # トピック名
    SELFTOPIC = "srv_" + SELFNODE

    # キュー
    __qworkerstat = Queue.Queue()
    __qsigtower = Queue.Queue()
    __qct = Queue.Queue()

    # SQLiteに残っている件数
    __sqlite_remain = 0

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
        self.service_ct = rospy.Service(
            self.SELFTOPIC + "_ct", s_ct_srv, self.handle_ct)
        self.service_st = rospy.Service(
            self.SELFTOPIC + "_st", s_sigtower_srv, self.handle_st)
        self.service_ws = rospy.Service(
            self.SELFTOPIC + "_ws", s_workerstat_srv, self.handle_ws)
        # 初期化完了
        self.__s.message("[%s] Do...%s" % (
            os.path.basename(__file__),
            ("(verbose)" if arg_verbose else "")), s.INFO)

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
            # request.Whiteが0以上
            # （即ち、測定している状態。していなければ-1が格納されてる）
            # （以降、該当の点灯色が0以上なら、その灯番号を最大とする桁数に調整）
            state = ("%s%s%s%s%s" %
                     (request.White, request.Blue, request.Green, request.Yellow, request.Red))
            # キューに追加
            self.__qsigtower.put([state, 4, dtnow])
            # self.A_SigTower(state, 5)
        elif request.Blue >= 0:
            state = ("%s%s%s%s" %
                     (request.Blue, request.Green, request.Yellow, request.Red))
            self.__qsigtower.put([state, 3, dtnow])
            # self.A_SigTower(state, 4)
        elif request.Green >= 0:
            state = ("%s%s%s" % (request.Green, request.Yellow, request.Red))
            self.__qsigtower.put([state, 2, dtnow])
            # self.A_SigTower(state, 3)
        elif request.Yellow >= 0:
            state = ("%s%s" % (request.Yellow, request.Red))
            self.__qsigtower.put([state, 1, dtnow])
            # self.A_SigTower(state, 2)
        elif request.Red >= 0:
            state = ("%s" % (request.Red))
            self.__qsigtower.put([state, 0, dtnow])
            # self.A_SigTower(state, 1)
        else:
            state = "-"
        # ログ
        self.__s.message(" * queue : SigTower [%s] > %s" % (0, state))
        # SQLiteに残っている件数を返す
        return s_sigtower_srvResponse(self.__sqlite_remain)

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
        self.__qct.put([str(request.Volt), dtnow])
        # 文字の組み立てとレコードの追加
        # self.A_Ct(str(request.Volt))
        # ログ
        self.__s.message(" * queue : Ct [%s] > %s" % (0, str(request.Volt)))
        # SQLiteに残っている件数を返す
        return s_ct_srvResponse(self.__sqlite_remain)

    def handle_ws(self, request):
        """
        問い合わせを受けて作業者状態の測定値を書き込み
        Parameters
        ----------
        request : s_ct_srv
            メッセージ
        """
        # 現在時刻を取得
        dtnow = datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        # 文字の組み立てとレコードの追加
        state = ("%s%s%s%s" %
                 (1 if request.Blue > 0 else 0,
                  1 if request.Green > 0 else 0,
                  1 if request.Yellow > 0 else 0,
                  1 if request.Red > 0 else 0))
        # キューに追加
        self.__qworkerstat.put([state, dtnow])
        # self.A_Ws(state)
        # ログ
        self.__s.message(" * queue : Ws [%s] > %s" % (0, state))
        # SQLiteに残っている件数を返す
        return s_workerstat_srvResponse(self.__sqlite_remain)

    def do(self):
        """
        データベースの書き込み処理
        """
        # インスタンス生成
        self.__dbm = DbMssql.DbMssql()
        self.__dbs = DbSqlite.DbSqlite()
        #
        # SQLServerの生死確認
        #
        sqltype = 0
        if self.__dbm.is_connectable() == False:
            self.__s.message("SQLServer is dead.", s.ERROR)
            # ホストから応答無いときはSQLite行き
            sqltype = 1
        else:
            # SQLServer行き
            sqltype = 0
        #
        # STEP1:シグナルタワー
        #
        # CCM識別子の定義
        ccm = ["SigTower1.mLI", "SigTower2.mLI",
               "SigTower3.mLI", "SigTower4.mLI", "SigTower5.mLI"]
        # キューを順次呼び出し
        while not self.__qsigtower.empty():
            buf = self.__qsigtower.get()
            # SQLServerに登録
            self.add_record(ccm[buf[1]], buf[0], buf[2], sqltype)

        #
        # STEP2:作業者状態
        #
        # キューを順次呼び出し
        while not self.__qworkerstat.empty():
            buf = self.__qworkerstat.get()
            # SQLServerに登録
            self.add_record("WorkerStat4.mLI", buf[0], buf[1], sqltype)
        #
        # STEP3:CT
        #
        # キューを順次呼び出し
        while not self.__qct.empty():
            buf = self.__qct.get()
            # SQLServerに登録
            self.add_record("CTRaw.mLI", buf[0], buf[1], sqltype)
        #
        # STEP4:書き込みできなかったデータを再書き込み
        #
        if sqltype == 0:
            # SQLiteからデータを取り出してそのレコードを消す
            rows = self.__dbs.select_delete_from_table(
                c.DB_SQLITE_MSSQL_SEND_MAX)
            # 読み出したレコードをSQLServerに追加
            for row in rows:
                # SQLServerに追加
                self.add_record(row[2], row[3], row[1], sqltype)
        #
        # STEP5:SQLiteの残件数更新
        #
        # 残件問い合わせ
        self.__sqlite_remain = self.__dbs.count_from_table()
        # ログに表示
        self.__s.message(" * sqlite remain : %s" % (self.__sqlite_remain))

    def add_record(self, arg_CCM, arg_val, arg_date, arg_sqltype=0):
        """
        レコードの追加
        """
        ret = 0
        if arg_sqltype == 0:
            # SQLServerに書き込み
            ret = self.__dbm.add_to_table(("%s" % os.uname()[1]),
                                          arg_val,
                                          ("%s" % (arg_CCM)),
                                          cl._DETECT_LOCAL_ROOM,
                                          cl._DETECT_LOCAL_REGION,
                                          cl._DETECT_LOCAL_ORDER, 30, "",
                                          arg_date)
            mesmode = s.DEBUG

        # 戻り値確認
        if ret < 0 or arg_sqltype == 1:
            # 負の値の時は書き込み失敗、あるいはSQLServer応答無いとき
            # SQLiteに退避
            self.__dbs.add_to_table([arg_date, arg_CCM, arg_val])
            mesmode = s.WARN

        # ログ表示
        self.__s.message(" * sql   : [%s] (%s) > %s" %
                         (arg_CCM, arg_date, arg_val), mesmode)

        return ret


if __name__ == '__main__':
    """
    メイン
    """

    # インスタンスを生成
    sr = Storage()
    # 処理周期Hz
    r = rospy.Rate(c.DB_MSSQL_APPEND_INTERVAL)
    try:
        # 繰り返し
        while not rospy.is_shutdown():
            sr.do()
            r.sleep()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
