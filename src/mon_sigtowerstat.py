#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Monitor the status of workers
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import datetime
import os
import numpy as np
import rospy
import rosparam
import time
import threading
import conf as c

# 生成したメッセージのヘッダファイル
from py_control.srv import mon_sigtowerstat_srv, mon_sigtowerstat_srvResponse
from py_control.srv import grove_ad_srv, grove_ad_srvResponse


class Mon_SigTowerStat:
    """
    設備シグナルタワー状態検知の処理
    """
    # ノード名
    SelfNode = "mon_sigtowerstat"
    # トピック名
    SelfTopic = "srv_" + SelfNode
    # GPIO入力対象のBCM番号
    GpioPin = []

    # AD変換の読み込み周期
    _PROCINTERVAL = 0.3

    # 設備シグナルタワーの状態定義
    # - 未検知
    STAT_NONE = -1
    # - 消灯
    STAT_OFF = 0
    # - 点滅
    STAT_BLINK = 1
    # - 点灯
    STAT_ON = 2

    # 設備シグナルタワーの状態
    # 0:RED  1:YELLOW  2:GREEN  3:BLUE  4:WHITE
    Stat = [STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE]

    def __init__(self):
        """
        コンストラクタ
        Parameters
        ----------
        argPin : int
            入力ピン番号
        """
        # 初期化
        rospy.loginfo("[%s] Initializing..." % (os.path.basename(__file__)))
        # ノードの初期化と名称設定
        rospy.init_node(self.SelfNode)
        # 自身のサービスの登録
        self.service = rospy.Service(
            self.SelfTopic, mon_sigtowerstat_srv, self.handle)

        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def handle(self, request):
        """
        問い合わせを受けたら設備シグナルタワーのステータスを回答
        Parameters
        ----------
        request : mon_sigtowerstat_srv
            メッセージ
        """
        # 各ランプの状態を回答
        return mon_sigtowerstat_srvResponse(
            self.Stat[0], self.Stat[1], self.Stat[2],
            self.Stat[3], self.Stat[4])

    def StatusReset(self):
        """
        ステータスを全てリセット
        """
        pass

    def thread_init(self):
        """
        スレッドのスタート
        """
        # スレッドをデーモンモードで開始
        thread_obj = threading.Thread(target=self.thread)
        thread_obj.setDaemon(True)
        thread_obj.start()

    def thread(self):
        """
        スレッド：シグナルタワーの状態を定期的に読み込んで更新
        """
        # 測定のメインループ
        while True:
            #
            # ステップ1：AD値を読み込み
            #
            # 読み込み開始時刻
            tm_start = datetime.datetime.now()
            # 測定N回分のAD値格納バッファ
            ad_dataN = []
            rospy.logdebug("Starting measure... %s" % (tm_start))
            while True:
                try:
                    # 測定1回分のAD値格納バッファ
                    ad_data1 = []
                    # ローカルプロキシにサービス名(srv_grove_ad)と型(grove_ad_srv)を設定
                    grove_ad = rospy.ServiceProxy('srv_grove_ad', grove_ad_srv)
                    # 対象チャネルをスキャン
                    for i in c.GROVE_AD_SIGTOWER:
                        # AD読み込み
                        ad = grove_ad(i)
                        # メッセージ
                        rospy.logdebug("Grove AD [%sch]:%s" % (i, ad.Val_VOLT))
                        # バッファに追加
                        ad_data1.append(ad.Val_VOLT)
                    # バッファに１回分の測定を追加
                    ad_dataN.append(ad_data1)
                except:
                    rospy.logwarn("Grove AD Measure Error")
                    pass
                finally:
                    pass
                # 現在時刻の取得
                tm_now = datetime.datetime.now()
                # 読み込みの終了条件の判定
                if (tm_now - tm_start).total_seconds() > c.SIGTOWER_DETECT_BETWEEN:
                    # 指定秒数を越えていれば読み込み終了
                    break
                # 次の読み込みまでスリープ
                time.sleep(self._PROCINTERVAL)
            #
            # ステップ2：
            #
            # 判定用クラスのインスタンス生成
            std = SigTowDetector(ad_dataN)
            # シグナルタワーの判定の一括実行
            ans = std.Do()
            # ステータスの更新と測定結果の表示
            message = ""
            count = 0
            for i in ans:
                if (len(self.Stat) < count):
                    # Stat配列サイズ以下なら測定結果を書き込み
                    self.Stat[count] = i
                # メッセージの組み立て
                message += ("[%sch]:%s " % (count, i))
                count += 1
            # メッセージ表示
            rospy.logdebug(message)
            # 次の測定までウェイト
            time.sleep(c.SIGTOWER_DETECT_INTERVAL)


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
                ret = Mon_SigTowerStat.STAT_ON
            elif (flash >= on) and (flash >= off):
                # 点滅と判断する条件
                ret = Mon_SigTowerStat.STAT_BLINK
            else:
                # それ以外は消灯
                ret = Mon_SigTowerStat.STAT_OFF
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


if __name__ == '__main__':
    """
    メイン
    """

    try:
        # インスタンスを生成
        mss = Mon_SigTowerStat()
        # 初期化
        mss.StatusReset()
        # スレッドのスタート
        mss.thread_init()
        # プロセス終了までアイドリング
        rospy.spin()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
