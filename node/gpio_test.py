#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-

# -----------------------------------------------
# GPIO Test
# by myasu 2019
# -----------------------------------------------

import os
import rospy
import rosparam
# 生成したメッセージのヘッダファイル
from py_control.msg import gpio_mes
from py_control.srv import grove_ad_srv


def callback(message):
    """
    コールバック関数
    Parameters
    ----------
    message : gpio_mes
        メッセージ
    """

    # ログの表示
    rospy.loginfo("Button Port: %s, Val: %s" % (message.port, message.value))


def do():
    """
    publish関数
    Parameters
    ----------
    """

    # ノードの初期化と名称設定
    # anonymousをTrueにして、ノード名を自動で変更して複数接続OKとする
    rospy.init_node('gpio_test', anonymous=True)

    # 指定のトピックへメッセージを送信
    pub_out = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)
    # 指定のトピックからメッセージを受信
    rospy.Subscriber("mes_gpio_in", gpio_mes, callback)

    # ローカルプロキシにサービス名(srv_grove_ad)と型(grove_ad_srv)を設定
    grove_ad = rospy.ServiceProxy('srv_grove_ad', grove_ad_srv)

    # 送信周期(Hz)を設定
    rate = rospy.Rate(1)

    count = 0
    port = 0
    while not rospy.is_shutdown():

        # ------------------------------
        # テスト：GPIO
        # ------------------------------

        # 送信するメッセージの作成
        msg = gpio_mes()
        msg.port = port
        msg.value = 1
        rospy.loginfo("[%s]Lamp Port: %s, Val: %s" %
                       (count, msg.port, msg.value))

        # 送信
        pub_out.publish(msg)
        rate.sleep()
        count += 1

        # 送信するメッセージの作成
        msg = gpio_mes()
        msg.port = port
        msg.value = 0
        rospy.loginfo("[%s]Lamp Port: %s, Val: %s" %
                       (count, msg.port, msg.value))

        # 送信
        pub_out.publish(msg)
        count += 1

        port += 1
        if port > 7:
            port = 0

        # ------------------------------
        # テスト：GROVE AD
        # ------------------------------
        # word_counterが呼び出されたときにサービスコールが行われる
        try:
            ad_0 = grove_ad(0)
            ad_1 = grove_ad(1)
            ad_2 = grove_ad(2)
            ad_3 = grove_ad(3)
            ad_4 = grove_ad(4)
            ad_5 = grove_ad(5)
            rospy.loginfo("Grove AD [%sch]:%s [%sch]:%s [%sch]:%s [%sch]:%s" %
                        (0, ad_0.Val_VOLT, 2, ad_2.Val_VOLT, 3, ad_3.Val_VOLT, 4, ad_4.Val_VOLT))
        except:
            pass
        finally:
            pass


if __name__ == '__main__':
    """
    メイン関数
    Parameters
    ----------
    """

    # 初期化
    rospy.logdebug("[%s] Initializing..." % (os.path.basename(__file__)))
    try:
        # 実行
        rospy.loginfo("[%s] Do..." % (os.path.basename(__file__)))
        do()
    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
