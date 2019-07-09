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
import conf as c
import SigTowDetector as sd

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
                    rospy.logwarn(sys.exc_info()[0])
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
            std = sd.SigTowDetector(ad_dataN)
            # シグナルタワーの判定の一括実行
            ans = std.Do()
            # print(ans)
            # ステータスの更新と測定結果の表示
            message = ""
            count = 0
            for i in ans:
                if (count < len(self.Stat)):
                    # Stat配列サイズ以下なら測定結果を書き込み
                    self.Stat[count] = i
                # メッセージの組み立て
                message += ("[%sch]:%s " % (count, i))
                count += 1
            # メッセージ表示
            rospy.logdebug(message)
            # 次の測定までウェイト
            # print(self.Stat)
            time.sleep(c.SIGTOWER_DETECT_INTERVAL)


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
