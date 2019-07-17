#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node (sub module)
# Know the status of the device by looking at the lamp
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import datetime
import os
import numpy as np
import time
import threading
import conf as c
import Mon_Sigtowerstat as sts

class SigTowDetector:
    """
    シグナルタワー状態判定
    """

    # AD変換の元データ
    # [ [0ch, 1ch, 2ch, 3ch], [0ch, 1ch, 2ch, 3ch], [0ch, 1ch, 2ch, 3ch], ...]
    Data_AD_Raw = []

    def __init__(self, arg_Data_AD_Raw):
        """
        コンストラクタ
        Parameters
        ----------
        arg_Data_AD_Raw : int[]
            元データ
        """
        self.Data_AD_Raw = arg_Data_AD_Raw
        pass

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def JudgeOnOff(self, arg_Source):
        """
        点灯・消灯を判定
        Parameters
        ----------
        arg_Source : int[]
            元データ
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        Returns
        -------
        return LampStateOnOff : int[][]
            ランプの点灯状態
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        """
        # chごとの値のリストを取り出し
        count_ch = 0
        LampStateOnOff = []
        for list_ch in arg_Source:
            # 値のリストから各値を取り出し
            list_dist = []
            for data in list_ch:
                # 各chの点灯消灯閾値から判定
                if data > c.SIGTOWER_LIGHT_THRESHOLD[count_ch]:
                    # 点灯と判定した
                    list_dist.append('H')
                else:
                    # 消灯と判定した
                    list_dist.append('L')
            # 判定結果を代入
            LampStateOnOff.append(list_dist)
            # ch番号をインクリメント
            count_ch += 1
        return LampStateOnOff

    def JudgeStateChange(self, arg_Source):
        """
        点灯・消灯の状態変化を判定
        Parameters
        ----------
        arg_Source : int[]
            元データ
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        Returns
        -------
        LampStateChange : int[][]
            点灯・点滅と状態変化
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        """
        # chごとの状態のリストを取り出し
        count_ch = 0
        state_prev = ''
        LampStateChange = []
        for list_ch in arg_Source:
            # 値のリストから各値を取り出し
            list_dist = []
            count_skip_1st = False
            for data in list_ch:
                # １個目の判定では、"U""D"の判断をしない
                if not count_skip_1st:
                    # 元のデータをそのまま代入
                    list_dist.append(data)
                    # 1個目判定済フラグを立てる
                    count_skip_1st = True
                else:
                    # 各chの状態が、前回と変わったかを比較
                    if (state_prev == 'L') and (data == 'H'):
                        # 消灯→点灯と判定したら
                        list_dist.append('U')
                    elif (state_prev == 'H') and (data == 'L'):
                        # 点灯→消灯と判定したら
                        list_dist.append('D')
                    else:
                        # 変化無しと判断したら
                        # 元のデータをそのまま代入
                        list_dist.append(data)
                # 今回の状態を代入し次回の比較とする
                state_prev = data
            # 判定結果をリストに追加
            LampStateChange.append(list_dist)
            # ch番号をインクリメント
            count_ch += 1
        return LampStateChange

    def JudgeStateProbability(self, arg_Source):
        """
        点灯・点滅・消灯の”重み”を判定
        Parameters
        ----------
        arg_Source : int[]
            元データ
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        Returns
        -------
        LampStateProbability : int[][]
            ランプが”点灯・点滅・消灯”と判断する値の重み
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        """
        # chごとの状態のリストを取り出し
        LampStateProbability = []
        for list_ch in arg_Source:
            # 各点灯状態のカウント
            # 全数
            all = float(len(list_ch))
            # 点灯
            on = round(list_ch.count("H") / all, 1)
            # 点滅
            blink = round((list_ch.count("U") + list_ch.count("D")) / all, 1)
            # 消灯
            off = round((1 - (on + blink)), 1)
            # 代入
            LampStateProbability.append([on, blink, off])
        return LampStateProbability

    def JudgeState(self, arg_Source):
        """
        点灯・点滅・消灯の状態を判定
        Parameters
        ----------
        arg_Source : int[]
            元データ
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        Returns
        -------
        LampState : int[][]
            ランプが”点灯・点滅・消灯”と判断する値の重み
            [ [0ch......], [1ch......], [2ch......], [3ch......] ]
        """
        # chごとの状態のリストを取り出し
        LampState = []
        for list_ch in arg_Source:
            # 一旦値を取り出し
            on = list_ch[0]
            flash = list_ch[1]
            off = list_ch[2]
            # 判定
            if (on > flash) and (on >= off):
                # 点灯と判断する条件
                ret = sts.Mon_SigTowerStat.STAT_ON
            elif (flash >= on) and (flash >= off):
                # 点滅と判断する条件
                ret = sts.Mon_SigTowerStat.STAT_BLINK
            else:
                # それ以外は消灯
                ret = sts.Mon_SigTowerStat.STAT_OFF
            # 代入
            LampState.append(ret)
        return LampState

    def Do(self):
        """
        判定の一括実行
        """
        # リストを転置
        arr_t = np.array(self.Data_AD_Raw).T
        # 閾値判定
        ret1 = self.JudgeOnOff(arr_t)
        # 変化判定
        ret2 = self.JudgeStateChange(ret1)
        # ”重み”判定
        ret3 = self.JudgeStateProbability(ret2)
        # 最終判定
        ret4 = self.JudgeState(ret3)
        # 動作確認用表示
        # print(arr_t)
        # print(ret4)
        # 結果を返す
        return ret4

