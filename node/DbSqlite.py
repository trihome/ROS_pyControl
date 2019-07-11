#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node (sub module)
# Write information to database, SQLite
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------
# https://qiita.com/Brutus/items/383acea6b2d294b45b6b
#
# -----------------------------------------------
from contextlib import closing
import datetime
import errno
import os
import Queue
import rospy
import sqlite3
import subprocess
import sys
# ローカル
import ComFunctions

# ----------------------------------
# 定数
# ----------------------------------

# SQLite接続情報
_SQLITE_HOST = "/home/pi/catkin_ws/src/py_control/db/datasaving.db"

# テーブル生成
# https://www.dbonline.jp/sqlite/table/index9.html
# $ sqlite3 datasaving.db
# > create table Uecs_LRaw(id integer primary key, TriggerDate text, DataType text, DataVal int);
# $ cp -p datasaving.db datasaving.db.org

# ----------------------------------
# クラス
# ----------------------------------


class DbSqlite:
    """
    設備稼働情報データベースに書き込む
    """
    pass
    # ----------------------------------
    # private変数
    # ----------------------------------

    # ----------------------------------
    # public変数
    # ----------------------------------
    # キュー
    q_datasaving = Queue.Queue()

    def __init__(self, arg_verbose=True):
        """
        コンストラクタ
        Parameters
        ----------
        arg_verbose:bool
            表示を有効化
        """
        # 共通処理クラスの宣言と引数の代入
        self.__cf = ComFunctions.ComFunctions(arg_verbose)

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def is_connectable(self, arg_host=_SQLITE_HOST):
        """
        ホストの死活チェック
        Parameters
        ----------
        arg_host:string
            ホスト名
        """
        # DBファイルの存在をチェック
        if os.path.isfile(arg_host):
            return True
        else:
            rospy.logerror("DB File [%s] is not exist." % arg_host)
            return False

    def add_to_table(self, arg_row):
        """
        レコードを１件書き込み
        Parameters
        ----------
        arg_row:
            レコード
        Notes
        ----------
        https://qiita.com/mas9612/items/a881e9f14d20ee1c0703
        """
        with closing(sqlite3.connect(_SQLITE_HOST)) as conn:
            c = conn.cursor()
            #SQL文生成
            sql = 'insert into Uecs_LRaw (TriggerDate , DataType, DataVal) values (?,?,?)'
            data = arg_row
            #実行
            c.execute(sql, data)
            #コミット
            conn.commit()

