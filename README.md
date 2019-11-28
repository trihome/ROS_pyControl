# ROS pyControl

I'm looking for a way to use ROS in different situations.

In this project, Raspberry Pi and ROS were used instead of using PLC to control factory production equipment.

## summary

This is a device that automatically observes what is happening at the work site.

These information obtained by this device are used to improve the production site.

- A light sensor detects that the indicator light installed in the manufacturing facility is on.

- The CT sensor detects the amount of current flowing to the manufacturing facility.

- By pressing one of a plurality of push buttons arranged within reach of the operator, it is possible to know what the operator is currently doing.

- These measurements are periodically transferred to the database server installed at the factory.

# Install

## Python Packages

```
$ sudo apt-get install freetds-dev -y
$ sudo apt install python-pymssql
```

## grove.py

https://github.com/Seeed-Studio/grove.py

```
$ curl -sL https://github.com/Seeed-Studio/grove.py/raw/master/install.sh | sudo bash -s -
```

## SQLite

```
$ sudo apt install sqlite3
```

## Scripts

### Power Watcher

ex) Watching GPIO 4(BCM) Fall down

```
$ sudo ln -s /[INSTALL DIR]/setPowerWatcher.service /etc/systemd/system/
$ sudo systemctl enable setPowerWatcher.service
$ sudo systemctl start setPowerWatcher.service
```

### System clock

```
$ sudo ln -s /[INSTALL DIR]/setSystimeRTC.service /etc/systemd/system/
$ sudo systemctl enable setSystimeRTC.service
$ sudo systemctl start setSystimeRTC.service
```

### Log Rotate

```
 $ cd /etc/logrotate.d
 $ chmod 644 /[INSTALL DIR]/catkin_ws/src/py_control/py_control.logrotate
 $ sudo cp /[INSTALL DIR]/catkin_ws/src/py_control/py_control.logrotate ./py_control
 $ sudo logrotate -f ./py_control
```
(check)
```
 $ logrotate -dv /etc/logrotate.conf
```

## ROS

https://hotch-potch.hatenadiary.jp/entry/2019/08/30/213000



## py_control

```
$ sudo ln -s /[INSTALL DIR]/catkin_ws/src/py_control/py_control.service /etc/systemd/system/
$ sudo systemctl enable py_control
$ sudo systemctl start py_control
  
```

### Tables

#### Uecs_LRaw

```
--検出端末から受け取った生情報（UECSライクのデータ形式）
create table [dbo].[Uecs_LRaw] (
  EqLogID int identity not null --ログのID
  , ClientIp nvarchar(45) default '0.0.0.0' not null --端末IPアドレス
  , DataVal decimal(9, 3) --データ
  , DataType nvarchar(20) --CCM識別子の値
  , DataRoom tinyint default 0 not null --部屋・棟番号
  , DataRegion smallint default 0 not nul --系統番号
  , DataOrder smallint default 0 not nulll --通し番号
  , DataPriority tinyint default 30 not null --優先順位
  , DataAVal nvarchar(255) --データの値・拡張型
  , TriggerDate datetime not null --検出時刻
  , ProcTime smallint default 0 not null --処理回数
  , primary key (EqLogID)
);
```

#### Uecs_Status

```
--データの解釈
create table [dbo].[Uecs_Status] (
  id int identity not null --解釈名ID
  , Status smallint default 0 not null --状態番号
  , StatusName nvarchar(50) not null --状態名
  , Comment ntext --状態名の説明
  , Graph_Pie nvarchar(50) not null --円グラフの色
  , Reserved1 smallint --予約
  , Reserved2 smallint --予約
  , Reserved3 nvarchar(50) --予約
  , primary key (id)
);
```

#### Uecs_StatusLink

```
--データの解釈の紐付け
create table [dbo].[Uecs_StatusLink] (
  id int identity not null --解釈の紐づけID
  , DataOrder smallint default 0 not null --通し番号
  , DataVal decimal(9, 3) --データの値
  , DataType nvarchar(20) --CCM識別子
  , Status smallint default 0 not null --状態番号
  , primary key (id)
);
```


### start

```
$ roslaunch py_control py_control.launch
... logging to /home/pi/.ros/log/3da09b9e-1177-11ea-9380-b827eb5656f4/roslaunch-raspizwh-15437.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://raspizwh:36819/

SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.6

NODES
  /
    gpio_in (py_control/Gpio_In.py)
    gpio_out (py_control/Gpio_Out.py)
    grove_ad (py_control/Grove_AD.py)
    main (py_control/Main.py)
    mon_sigtowerstat (py_control/Mon_SigTowerStat.py)
    mon_workerstat (py_control/Mon_WorkerStat.py)
    mystat (py_control/MyStat.py)
    storage (py_control/Storage.py)

auto-starting new master
process[master]: started with pid [15473]
ROS_MASTER_URI=http://raspizwh:11311

setting /run_id to 3da09b9e-1177-11ea-9380-b827eb5656f4
process[rosout-1]: started with pid [15502]
started core service [/rosout]
process[gpio_in-2]: started with pid [15511]
process[gpio_out-3]: started with pid [15522]
process[grove_ad-4]: started with pid [15525]
process[storage-5]: started with pid [15527]
process[mon_workerstat-6]: started with pid [15529]
process[mon_sigtowerstat-7]: started with pid [15532]
process[main-8]: started with pid [15536]
process[mystat-9]: started with pid [15540]
...
```

### check
```
$ rosnode list
/gpio_in
/gpio_out
/grove_ad
/main
/mon_sigtowerstat
/mon_workerstat
/mystat
/rosout
/storage
$ rostopic list
/mes_gpio_in
/mes_gpio_out
/mes_mon_workerstat
/rosout
/rosout_agg
 $ rosservice list
/gpio_in/get_loggers
/gpio_in/set_logger_level
/gpio_out/get_loggers
/gpio_out/set_logger_level
/grove_ad/get_loggers
/grove_ad/set_logger_level
/main/get_loggers
/main/set_logger_level
/mon_sigtowerstat/get_loggers
/mon_sigtowerstat/set_logger_level
/mon_workerstat/get_loggers
/mon_workerstat/set_logger_level
/mystat/get_loggers
/mystat/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/srv_gpio_in_stat
/srv_gpio_out_stat
/srv_grove_ad
/srv_grove_ad_stat
/srv_main_stat
/srv_mon_sigtowerstat
/srv_mon_sigtowerstat_stat
/srv_mon_workerstat
/srv_mon_workerstat_stat
/srv_storage_ct
/srv_storage_st
/srv_storage_stat
/srv_storage_ws
/storage/get_loggers
/storage/set_logger_level
```
