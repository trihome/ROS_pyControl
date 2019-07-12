#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# Common Functions
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------
import datetime
import errno
import rospy
import subprocess
import sys

# ----------------------------------
# 定数
# ----------------------------------


# ----------------------------------
# クラス
# ----------------------------------

class ComFunctions:
    """
    設備稼働情報データベースに書き込む
    """
    pass
    # ----------------------------------
    # ローカル定数
    # ----------------------------------

    #エラーレベル
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4
    
    # ----------------------------------
    # ローカル変数
    # ----------------------------------

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

    def log(self, arg_message, arg_Type=DEBUG):
        """
        処理メッセージ
        Parameters
        ----------
        arg_message:string
            メッセージ
        arg_Type:int
            警告レベル（未使用）
        """
        #メッセージの加工
        message = ("%s @%s" % (arg_message, os.path.basename(__file__)))
        if self.__verbose:
            #画面出力優先
            rospy.loginfo(message)
        else:
            #画面出力しない
            rospy.logdebug(message)

