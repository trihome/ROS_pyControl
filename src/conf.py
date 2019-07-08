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
GPIO_WORKERSTAT = ["RED", "YELLOW", "GREEN", "BLUE"]

# 装置シグナルタワーの割り当て範囲
GPIO_OUT_SIGTOWER = range(4, 4)
GPIO_SIGTOWER = ["RED", "YELLOW", "GREEN", "BLUE"]

# --------------------------------------
# その他設定
# --------------------------------------
