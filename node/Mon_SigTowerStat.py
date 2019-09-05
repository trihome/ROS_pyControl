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
import rospy
import rosparam
import sys
import time
import threading
# ローカル
import conf as c
import MyStat as s
import SigTowDetector as sd
# メッセージのヘッダファイル
from py_control.srv import mon_sigtowerstat_srv, mon_sigtowerstat_srvResponse
from py_control.srv import grove_ad_srv, grove_ad_srvResponse
from py_control.msg import mon_workerstat_mes


class Mon_SigTowerStat:
    """
    設備シグナルタワー状態検知の処理
    """
    # ノード名
    SELFNODE = "mon_sigtowerstat"
    # トピック名
    SELFTOPIC = "srv_" + SELFNODE

    # AD変換の読み込み周期
    #_PROCINTERVAL = 0.3

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
    # （5灯タイプ）0:RED  1:YELLOW  2:GREEN  3:BLUE  4:WHITE
    __signaltow_stat = [STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE, STAT_NONE]

    def __init__(self, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        arg_verbose:bool
            メッセージの強制表示
        """
        # ステータスクラスの初期化
        self.__s = s.MyStat(self.SELFNODE, arg_verbose)
        # ノードの初期化と名称設定
        rospy.init_node(self.SELFNODE)
        # 自身のサービスの登録
        self.service = rospy.Service(
            self.SELFTOPIC, mon_sigtowerstat_srv, self.handle)
        # 初期化完了
        self.__s.message("[%s] Do...%s" % (
            os.path.basename(__file__),
            ("(verbose)" if arg_verbose else "")), s.INFO)

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
            self.__signaltow_stat[0], self.__signaltow_stat[1], self.__signaltow_stat[2],
            self.__signaltow_stat[3], self.__signaltow_stat[4])

    def StatusReset(self):
        """
        ステータスを全てリセット
        """
        for i in range(0, len(self.__signaltow_stat)):
            self.__signaltow_stat[i] = self.STAT_NONE

    def do(self):
        """
        シグナルタワーの状態を観測して判断
        """
        #
        # ステップ1：AD値を読み込み
        #
        # 読み込み開始時刻
        tm_start = datetime.datetime.now()
        # 測定N回分のAD値格納バッファ
        ad_dataN = []
        #
        self.__s.message("Starting measure... %s" % (tm_start))
        # 指定の時間の間、シグナルタワーを観測
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
                # 例外発生時
                self.__s.message("Grove AD Measure Error: %s" %
                                 (sys.exc_info()[0]), s.WARN)
            finally:
                pass
            # 現在時刻の取得
            tm_now = datetime.datetime.now()
            # 読み込みの終了条件の判定
            if (tm_now - tm_start).total_seconds() > c.SIGTOWER_DETECT_BETWEEN:
                # 指定秒数を越えていれば読み込み終了
                break
            # 次の読み込みまでスリープ
            time.sleep(c.SIGTOWER_DETECT_BETWEEN)
        #
        # ステップ2：
        #
        # 判定用クラスのインスタンス生成
        std = sd.SigTowDetector(ad_dataN)
        # シグナルタワーの判定の一括実行
        ans = std.Do()
        # print(ans)
        # ステータスの更新と測定結果の表示
        message = "<SigTStat> "
        count = 0
        for i in ans:
            if (count < len(self.__signaltow_stat)):
                # Stat配列サイズ以下なら測定結果を書き込み
                self.__signaltow_stat[count] = i
            # メッセージの組み立て
            message += ("[%sch]:%s " % (count, i))
            count += 1
        # メッセージ表示
        self.__s.message(message)
        # #
        # # ステップ3：緑ランプ点灯時だけ、WorkerStatを無効化 （7/24機能を無効化）
        # #
        # # 指定のトピックへメッセージを送信：
        # pub = rospy.Publisher('mes_mon_workerstat',
        #                       mon_workerstat_mes, queue_size=10)
        # msg = mon_workerstat_mes()
        # # 緑ランプが点灯・点滅してるとき
        # if (self.__signaltow_stat[2] == self.STAT_BLINK) or (self.__signaltow_stat[2] == self.STAT_ON):
        #     # 無効化のメッセージ送信
        #     msg.enable = False
        #     pub.publish(msg)
        # else:
        #     # 有効化のメッセージ送信
        #     msg.enable = True
        #     pub.publish(msg)


if __name__ == '__main__':
    """
    メイン
    """

    # インスタンスを生成
    mss = Mon_SigTowerStat()
    # 処理周期Hz
    r = rospy.Rate(c.SIGTOWER_DETECT_INTERVAL)
    try:
        # 初期化
        mss.StatusReset()
        # 繰り返し
        while not rospy.is_shutdown():
            mss.do()
            r.sleep()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
