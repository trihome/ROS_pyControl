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
import MyStat as s

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
        # ステータスの初期化
        self.__s = s.MyStat(None)

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
            self.__s.message("DB File [%s] is not exist." % arg_host, s.ERROR)
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
        try:
            with closing(sqlite3.connect(_SQLITE_HOST)) as conn:
                c = conn.cursor()
                # SQL文生成
                sql = 'insert into Uecs_LRaw (TriggerDate , DataType, DataVal) values (?,?,?)'
                data = arg_row
                # 実行
                c.execute(sql, data)
                # コミット
                conn.commit()
        except:
            # 例外メッセージ
            self.__s.message(sys.exc_info()[0], s.WARN)
        finally:
            pass

    def select_delete_from_table(self, arg_max_rows):
        """
        レコードを読み込んで、その分のレコードをテーブルから消す
        Parameters
        ----------
        arg_max_rows:
            最大行数
        Returns
        ----------
        rows
            レコードの配列
        """
        rows = None
        try:
            with closing(sqlite3.connect(_SQLITE_HOST)) as conn:
                c = conn.cursor()

                # レコードを読み込み
                sql = "SELECT * FROM Uecs_LRaw ORDER BY id LIMIT %s" % arg_max_rows
                # 実行
                c.execute(sql)
                rows = c.fetchall()

                # 読み込んだレコードを削除
                for row in rows:
                    sql = "DELETE FROM Uecs_LRaw WHERE ID = %s" % row[0]
                    # 実行
                    c.execute(sql)
                # コミット
                conn.commit()
        except:
            # 例外メッセージ
            self.__s.message(sys.exc_info()[0], s.WARN)
        finally:
            pass
        #結果を返す
        return rows

    def count_from_table(self):
        """
        残りのレコード数を取得
        Parameters
        ----------
        Returns
        ----------
        rows
            レコード数
        """
        try:
            with closing(sqlite3.connect(_SQLITE_HOST)) as conn:
                c = conn.cursor()
                # レコード件数
                sql = "SELECT COUNT(*) FROM Uecs_LRaw;"
                # 実行
                for row in c.execute(sql):
                    #結果を返す
                    return int(row[0])
        except:
            # 例外メッセージ
            self.__s.message(sys.exc_info()[0], s.WARN)
        finally:
            pass

        return -1
