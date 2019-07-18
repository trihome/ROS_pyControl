#!/bin/bash
#--------------------------------------------------------------------
#バックグラウンド実行用のスクリプト
#--------------------------------------------------------------------
#
#install:
# $ sudo ln -s /home/pi/catkin_ws/src/py_control/py_control.service /etc/systemd/system
# $ sudo systemctl enable py_control.service
# $ sudo systemctl start py_control.service
# $ systemctl status py_control.service
#--------------------------------------------------------------------

#変数の設定
SCRIPTDIR=/home/pi/catkin_ws/src/py_control
LOGDIR=$SCRIPTDIR/log
ROS_HOSTNAME=raspibp
ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311

#実行
source /home/pi/catkin_ws/devel/setup.bash
exec roslaunch py_control py_control.launch >> ${LOGDIR}/sigTowerDetector.log 2>&1
