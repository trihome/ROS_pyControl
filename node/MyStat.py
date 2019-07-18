#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Monitor the operating status of nodes
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------
import datetime
import errno
import os
import re
import rospy
import rosservice
import sys
import time
# ローカル
import conf as c
import conf_local as cl
# メッセージのヘッダファイル
from py_control.srv import mystat_srv, mystat_srvResponse
from py_control.msg import gpio_mes

# ----------------------------------
# 定数
# ----------------------------------

# ログメッセージとノードのエラーレベル
NONE = -1  # ノード用
NORMAL = 0  # ノード用
DEBUG = 0  # ログメッセージ用
INFO = 1  # ログメッセージ用
WARN = 2  # 共用
ERROR = 3  # 共用
FATAL = 4  # 共用


# ----------------------------------
# クラス
# ----------------------------------


class MyStat:
    """
    ノードのステータス管理
    （各ノードでインスタンスを作って使用する）
    """
    # ----------------------------------
    # 定数
    # ----------------------------------

    # ----------------------------------
    # ローカル変数
    # ----------------------------------
    __status_now = NONE
    __status_last_debug = None
    __status_last_info = None
    __status_last_warn = None
    __status_last_error = None
    __status_last_fatal = None
    __verbose = False
    # ----------------------------------
    # 公開変数
    # ----------------------------------
    @property
    def __status_now(self):
        """
        ノードの現在ステータス
        """
        return self.__status_now

    @__status_now.setter
    def __status_now(self, value):
        """
        ノードの現在ステータス
        """
        self.__status_now = value

    # ----------------------------------
    # メソッド
    # ----------------------------------

    def __init__(self, arg_parent_node, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        arg_parent_node:
            親のrospy
        arg_verbose:bool
            ログメッセージ表示の強制を有効化
        """
        # ステータスの初期化
        self.__status_now = NORMAL
        # ログメッセージ表示を強制する
        self.__verbose = arg_verbose
        # ステータスサービスの登録
        if arg_parent_node is not None:
            self.service = rospy.Service(
                ("srv_%s_stat" % (arg_parent_node)),
                mystat_srv, self.handle)

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def handle(self, mystat_srv):
        """
        問い合わせを受けたらステータスを回答
        Parameters
        ----------
        Returns
        -------
        : mystat_srv
            メッセージ
        """
        # 回答する
        return mystat_srvResponse(
            self.__status_now, self.__status_last_debug, self.__status_last_info,
            self.__status_last_warn, self.__status_last_error, self.__status_last_fatal)

    def message(self, arg_message, arg_level=DEBUG):
        """
        処理メッセージ
        Parameters
        ----------
        arg_message:string
            ログメッセージ
        arg_Type:int
            警告レベル
        """
        # ステータス変更
        self.__status_now = arg_level
        # ログメッセージの加工
        message = ("%s" % (arg_message))
        # ログメッセージの表示
        if arg_level == DEBUG:
            # 現在時刻取得
            self.__status_last_debug = rospy.Time.now()
            if self.__verbose:
                # 画面出力
                rospy.loginfo(message)
            else:
                # 画面出力しない
                rospy.logdebug(message)
        elif arg_level == INFO:
            # 現在時刻取得
            self.__status_last_info = rospy.Time.now()
            # 画面出力
            rospy.loginfo(message)
        elif arg_level == WARN:
            # 現在時刻取得
            self.__status_last_warn = rospy.Time.now()
            # 警告表示
            rospy.logwarn(message)
        elif arg_level == ERROR:
            # 現在時刻取得
            self.__status_last_error = rospy.Time.now()
            # エラー表示
            rospy.logerr(message)
        elif arg_level == FATAL:
            # 現在時刻取得
            self.__status_last_fatal = rospy.Time.now()
            # 致命的エラー
            rospy.logfatal(message)
        else:
            # エラーレベルの設定ミス
            rospy.logwarn(" ! Error level miss")
            pass


class MystatMaster(MyStat):
    """
    ノードのステータス監視
    （各ノードを順番に監視）
    """
    # ----------------------------------
    # 定数
    # ----------------------------------
    # ノード名
    SELFNODE = "mystatm"
    # トピック名
    SELFTOPIC = "srv_" + SELFNODE

    # ----------------------------------
    # ローカル変数
    # ----------------------------------

    # ----------------------------------
    # 公開変数
    # ----------------------------------

    def __init__(self, arg_verbose=False):
        """
        コンストラクタ
        Parameters
        ----------
        arg_verbose:bool
            メッセージ表示を有効化
        """
        # ステータスの初期化
        self.__verbose = arg_verbose
        self.s = MyStat(None, arg_verbose)

    def __del__(self):
        """
        デストラクタ
        """
        pass

    def update_signaltower(self, arg_Color):
        """
        シグナルタワーの色変更
        Parameters
        ----------
        arg_Color:int
            シグナルタワーの色番号
        """
        # 指定のトピックへメッセージを送信
        pub = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
        # 送信するメッセージの作成
        msg = gpio_mes()
        for i in c.GPIO_OUT_SIGTOWER:
            # 点灯状態を変更するポート番号(X番号)を指定
            msg.port = i
            if i == arg_Color:
                # callbackで受け取ったポート番号(X番号)と一致していれば
                # 指定のランプをON
                msg.value = 1
            else:
                # それ以外のランプをOFF
                msg.value = 0
            # メッセージ送信
            pub.publish(msg)


def update_signaltower(arg_Color):
    """
    シグナルタワーの色変更
    Parameters
    ----------
    arg_Color:int
        シグナルタワーの色番号
    """
    # 指定のトピックへメッセージを送信
    pub = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
    # 送信するメッセージの作成
    msg = gpio_mes()
    for i in c.GPIO_OUT_SIGTOWER:
        # 点灯状態を変更するポート番号(X番号)を指定
        msg.port = i
        if i == arg_Color:
            # 引数に受け取ったポート番号(X番号)と一致していれば
            # 指定のランプをON
            msg.value = 1
        else:
            # それ以外のランプをOFF
            msg.value = 0
        # メッセージ送信
        pub.publish(msg)


if __name__ == '__main__':
    """
    メイン
    """

    # ノードの初期化と名称設定
    rospy.init_node(MystatMaster.SELFNODE)
    # インスタンスを生成
    msm = MystatMaster(True)
    rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))
    # 処理周期Hz
    r = rospy.Rate(c.MYSTAT_DETECT_INTERVAL)
    try:
        # #点灯テスト
        # while not rospy.is_shutdown():
        #         for i in range(4, 8):
        #             update_signaltower(i)
        #             r.sleep()

        # 起動時点のサービス一覧を取得
        service_list_boot = rosservice.get_service_list()

        # 警告以上のランプ点灯の保持用タイマー
        time_last_warn = None
        time_last_error = None

        # 繰り返し
        while not rospy.is_shutdown():
            # シグナルタワーを青にしてから処理開始
            update_signaltower(7)
            #
            # STEP1：各ノードの状態を取得
            #
            # サービス一覧を取得
            service_list = rosservice.get_service_list()
            # 監視対象のサービスだけ抽出
            service_list_match = [
                s for s in service_list if re.match('^/srv.*_stat$', s)]
            # 監視対象のサービスのステータス
            stat_list = []
            # 監視対象のサービスの警告以上の発生した最終時刻
            stat_list_warn = []
            stat_list_error = []
            stat_list_fatal = []
            # 各サービスに問い合わせる
            for service in service_list_match:
                # ローカルプロキシにサービス名と型を設定
                statsv = rospy.ServiceProxy(service, mystat_srv)
                # 問い合わせ
                statval = statsv()
                # ステータス値、最終警告発生時刻をリストに追加
                stat_list.append(statval.status_now)
                stat_list_warn.append(statval.status_last_warn)
                stat_list_error.append(statval.status_last_error)
                stat_list_fatal.append(statval.status_last_fatal)
                # 状態表示
                msm.s.message(("< MyStat > %s > %s" %
                               (service, statval.status_now)))
            # リスト、最終警告発生時刻の値の最大値を取得
            stat_max = max(stat_list)
            stat_last_warn = max(stat_list_warn)
            stat_last_error = max(stat_list_error)
            stat_last_fatal = max(stat_list_fatal)
            #
            # STEP2：警告状態を一定時間保持する小細工
            #
            # 現在時刻
            time_now = rospy.Time.now()
            #
            flug_warn = False
            flug_error = False
            # 警告の最終時刻が、"現在時刻 - MYSTAT_KEEP_LAMPON_TIME"以内の時
            print(time_now)
            print(stat_last_warn)
            print(rospy.Duration(c.MYSTAT_KEEP_LAMPON_TIME))
            if time_now - stat_last_warn < rospy.Duration(c.MYSTAT_KEEP_LAMPON_TIME):
                msm.s.message("< MyStat > A WARN has occurred ( %s sec ago)" % str((time_now - stat_last_warn) / pow(10,9)))
                flug_warn = True
            # エラーの最終時刻が・・・（同様
            if time_now - stat_last_error < rospy.Duration(c.MYSTAT_KEEP_LAMPON_TIME):
                msm.s.message("< MyStat > A ERROR has occurred ( %s sec ago)" % str((time_now - stat_last_error) / pow(10,9)))
                flug_error = True
            # エラーの最終時刻が・・・（同様
            if time_now - stat_last_fatal < rospy.Duration(c.MYSTAT_KEEP_LAMPON_TIME):
                flug_error = True
                msm.s.message("< MyStat > A FATAL has occurred ( %s sec ago)" % str((time_now - stat_last_fatal) / pow(10,9)))
            #
            # STEP3：ランプの点灯制御
            #
            # 警告レベルがエラー以上の時・かつ赤色点灯の保持時間内の場合
            if stat_max >= ERROR or flug_error:
                # シグナルタワーを赤点灯
                update_signaltower(4)
            # 警告レベルが警告以上の時・かつ黄色点灯の保持時間内の場合
            elif stat_max == WARN or flug_warn:
                # シグナルタワーを黄点灯
                update_signaltower(5)
            else:
                # それ以外ならシグナルタワーを緑点灯
                update_signaltower(6)
            # 待機
            r.sleep()

    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
