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
import pymssql
import errno
import sys
import datetime
import subprocess

# ----------------------------------
# 定数
# ----------------------------------

# SQLServer接続情報
_MSSQL_HOST = 'fdb-sv2.minoru-sangyo.co.jp'
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
        # 引数の代入
        self.__verbose = arg_verbose

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def prtmes(self, arg_message):
        """
        処理メッセージ
        Parameters
        ----------
        arg_message:string
            メッセージ
        """
        if self.__verbose:
            print(arg_message)

    def is_connectable(self, arg_host):
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
            self.prtmes(" ! Error : %s" % e)

    def AddToMssql(self, arg_IP, arg_data,
                   arg_type, arg_room, arg_region,
                   arg_order, arg_priority, arg_dataa):
        """
        1レコード書き込み
        """

        # SQLServerの生死確認
        if self.is_connectable(_MSSQL_HOST) == False:
            self.prtmes(" ! Worning : %s is dead." % _MSSQL_HOST)
            # ホストから応答無いときは移行の処理をしない
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
            dtnow = datetime.datetime.now()

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
                        GETDATE()
                    ) """ % (
                    arg_IP,
                    arg_data,
                    arg_type,
                    arg_room,
                    arg_region,
                    arg_order,
                    arg_priority,
                    arg_dataa
                )
            )
            self.conn.commit()

        except Exception as e:
            self.prtmes(" ! Error : %s" % e)
            # sys.exit(0)
            return -2

        finally:
            if self.conn is not None:
                self.conn.close()
            return 0
