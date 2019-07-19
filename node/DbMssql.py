#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node (sub module)
# Write information to database, SQLServer
# (According to the protocol UECS)
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------
import datetime
import errno
import pymssql
import rospy
import subprocess
import sys
#ローカル
import MyStat as s

# ----------------------------------
# 定数
# ----------------------------------

# SQLServer接続情報
_MSSQL_HOST = '192.168.0.26'
_MSSQL_DATABASE = 'fman'
_MSSQL_USER = 'fmanadmin'
_MSSQL_PW = 'fmanadminpassword'

# ----------------------------------
# クラス
# ----------------------------------


class DbMssql:
    """
    設備稼働情報データベースに書き込む
    """
    pass
    # ----------------------------------
    # ローカル変数
    # ----------------------------------
    conn = None

    def __init__(self, arg_verbose=True):
        """
        コンストラクタ
        Parameters
        ----------
        arg_verbose:bool
            表示を有効化
        """
        # ステータスの初期化
        self.__s = s.MyStat(None)

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def is_connectable(self, arg_host=_MSSQL_HOST):
        """
        ホストの死活チェック
        Parameters
        ----------
        arg_host:string
            ホスト名
        Notes
        -----
        https://qiita.com/hamemi/items/917eb822cf5bd12552cc
        """
        # pingのタイムアウト-w=2, 回数-c=1とした
        try:
            ping = subprocess.Popen(["ping", "-w", "2", "-c", "1", arg_host],
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            ping.communicate()
            # aliveなら0が帰ってくるので、そのときはtrueを返す
            return ping.returncode == 0
        except Exception as e:
            self.__s.message("%s" % e, s.ERROR)
            return False

    def add_to_table(self, arg_IP, arg_data,
                   arg_type, arg_room, arg_region,
                   arg_order, arg_priority, arg_dataa,
                   arg_date='2019/1/1 0:0:0'):
        """
        1レコード書き込み
        """
        # SQLServerの生死確認
        if self.is_connectable(_MSSQL_HOST) == False:
            self.__s.message("%s is dead." % _MSSQL_HOST, s.WARN)
            # ホストから応答無いときは以降の処理をしない
            return -1

        try:
            # SQLServerに接続
            # python2は引数の変数指定が無いと以下のエラー
            # (Connection to the database failed for an unknown reason.)
            self.conn = pymssql.connect(host=_MSSQL_HOST,
                                        user=_MSSQL_USER,
                                        password=_MSSQL_PW,
                                        database=_MSSQL_DATABASE)
            # 現在時刻
            #dtnow = datetime.datetime.now()
            #日付の書き込みモードの判定
            if arg_date == '2019/1/1 0:0:0':
                #省略時はサーバ側の時刻で自動入力
                sqldate = "GETDATE()"
            else:
                #引数の時刻を流用
                sqldate = arg_date
            # 書き込み
            cursor = self.conn.cursor()
            cursor.execute(
                """
                    INSERT INTO Uecs_LRaw(
                        ClientIp    ,
                        DataVal     ,
                        DataType    ,
                        DataRoom    ,
                        DataRegion  ,
                        DataOrder   ,
                        DataPriority,
                        DataAVal    ,
                        TriggerDate
                    ) VALUES (
                        '%s',
                        %s  ,
                        '%s',
                        %s  ,
                        %s  ,
                        %s  ,
                        %s  ,
                        '%s',
                        '%s'
                    ) """ % (
                    arg_IP,
                    arg_data,
                    arg_type,
                    arg_room,
                    arg_region,
                    arg_order,
                    arg_priority,
                    arg_dataa,
                    sqldate
                )
            )
            self.conn.commit()
        #例外
        except Exception as e:
            self.__s.message("%s" % e, s.ERROR)
            return -2
        #最終
        finally:
            #閉じる
            if self.conn is not None:
                self.conn.close()
            return 0
