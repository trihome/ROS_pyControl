#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
import datetime
import DbMssql
import DbSqlite
import os
import conf_local as cl

# # インスタンス生成
# __db = DbMssql.DbMssql()
# # SQLServerに登録
# ret = __db.AddToTable(("%s" % os.uname()[1]),
#                         "11111",
#                         "SigTower5.mLI",
#                         cl._DETECT_LOCAL_ROOM,
#                         cl._DETECT_LOCAL_REGION,
#                         cl._DETECT_LOCAL_ORDER, 30, "")

# インスタンス生成
__db = DbSqlite.DbSqlite()
# SQLiteに登録
ret = __db.add_to_table(
    [datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S"), "CT.mLI", "10"])
