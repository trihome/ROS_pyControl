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

# --------------------------------------
# IO機器の入出力割り当て
# --------------------------------------
# 作業者状態ボタンの割り当て範囲
GPIO_OUT_WORKERSTAT = range(0, 4)
GPIO_IN_WORKERSTAT = range(0, 4)
GPIO_WORKERSTATORDER = ["RED", "YELLOW", "GREEN", "BLUE"]

# 監視対象設備の電流センサの割り当て範囲
GROVE_AD_CT = range(0, 2)
GROVE_AD_CTORDER = ["CT1", "CT2"]

# 監視対象設備のシグナルタワーの割り当て範囲
GROVE_AD_SIGTOWER = range(2, 6)
GROVE_AD_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE"]

# 本装置のシグナルタワーの割り当て範囲
GPIO_OUT_SIGTOWER = range(4, 8)
GPIO_SIGTOWERORDER = ["RED", "YELLOW", "GREEN", "BLUE"]


# --------------------------------------
# その他設定：シグナルタワー
# --------------------------------------

# シグナルタワーのランプが点灯と判定する閾値 0-2048
#                          赤    黄    緑    青    白
SIGTOWER_LIGHT_THRESHOLD = (700, 700, 700, 700, 700)

#シグナルタワーの測定時間（秒）
#この秒数測定して、そのデータから点灯・点滅・消灯を判断
SIGTOWER_DETECT_BETWEEN = 6

#シグナルタワーの測定間隔（秒）
SIGTOWER_DETECT_INTERVAL = 20

