#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Common config file
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

# --------------------------------------
# GPIO入出力設定
# --------------------------------------
# GPIO出力
GPIO_OUT = [26, 19, 13, 6, 5, 22, 27, 17]

# GPIO入力
GPIO_IN = [21, 20, 16, 12, 7, 8, 25, 24]

# AD入力
AD_IN = range(0, 7)

# --------------------------------------
# IO機器の入出力割り当て
# --------------------------------------
# GPIO IN/OUT割り当て範囲:作業者状態ボタン
GPIO_OUT_WORKERSTAT = range(0, 4)
GPIO_IN_WORKERSTAT = range(0, 4)
GPIO_WORKERSTATORDER = ["RED", "YELLOW", "GREEN", "BLUE"]

# AD割り当て範囲:監視対象設備の電流センサ
GROVE_AD_CT = range(0, 2)
GROVE_AD_CTORDER = ["CT1", "CT2"]
# AD測定間隔(Hz)
GROVE_AD_DETECT_INTERVAL = 2.5

# AD割り当て範囲：監視対象設備のシグナルタワー
GROVE_AD_SIGTOWER = range(2, 6)
GROVE_AD_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE"]

# GPIO OUT割り当て範囲：本装置のシグナルタワー
GPIO_OUT_SIGTOWER = range(4, 8)
GPIO_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE"]


# --------------------------------------
# その他設定：メイン
# --------------------------------------

# 主処理の測定間隔(Hz)
#MAIN_INTERVAL = 0.033333
MAIN_INTERVAL = 0.05

# --------------------------------------
# その他設定：シグナルタワー
# --------------------------------------

# シグナルタワーのランプが点灯と判定する閾値 0-2048
#                          赤    黄    緑    青    白
SIGTOWER_LIGHT_THRESHOLD = (700, 700, 700, 700, 700)

# シグナルタワーの測定時間（秒）
# この秒数測定して、そのデータから点灯・点滅・消灯を判断
SIGTOWER_DETECT_BETWEEN = 6

# シグナルタワーの測定間隔（Hz）
SIGTOWER_DETECT_INTERVAL = 0.05

# --------------------------------------
# その他設定：データベース
# --------------------------------------

# データベースサーバへの書き込み間隔（Hz）
#DB_MSSQL_APPEND_INTERVAL = 0.016666
DB_MSSQL_APPEND_INTERVAL = 0.033333


# --------------------------------------
# その他設定：動作監視Mystat
# --------------------------------------

# 測定間隔（Hz）
MYSTAT_DETECT_INTERVAL = 0.1
