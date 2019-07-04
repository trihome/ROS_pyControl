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


def publisher():
    """
    publish関数
    Parameters
    ----------
    """

    # ノードの初期化と名称設定
    # anonymousをTrueにして、ノード名を自動で変更して複数接続OKとする
    rospy.init_node('gpio_test', anonymous=True)

    # 指定のトピックからメッセージを送信
    pub_out = rospy.Publisher('mes_gpio_out', gpio_mes, queue_size=10)

    # 指定のトピックからメッセージを受信
    rospy.Subscriber("mes_gpio_in", gpio_mes, callback)

    # 送信周期(Hz)を設定
    rate = rospy.Rate(2)

    count = 0
    port = 0
    while not rospy.is_shutdown():
        # 送信するメッセージの作成
        msg = gpio_mes()
        msg.port = port
        msg.value = 1
        rospy.logdebug("[%s]Lamp Port: %s, Val: %s" % (count, msg.port, msg.value))

        # 送信
        pub_out.publish(msg)
        rate.sleep()
        count += 1

        # 送信するメッセージの作成
        msg = gpio_mes()
        msg.port = port
        msg.value = 0
        rospy.logdebug("[%s]Lamp Port: %s, Val: %s" % (count, msg.port, msg.value))

        # 送信
        pub_out.publish(msg)
        rate.sleep()
        count += 1

        port += 1
        if port > 7:
            port = 0


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
        publisher()
    except rospy.ROSInterruptException:
        # 停止
        pass
    finally:
        # 終了
        rospy.loginfo("[%s] Done." % (os.path.basename(__file__)))
