#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
import os
import DbMssql
import conf_local as cl

# インスタンス生成
__db = DbMssql.DbMssql()
# SQLServerに登録
ret = __db.AddToMssql(("%s" % os.uname()[1]),
                        "11111",
                        "SigTower5.mLI",
                        cl._DETECT_LOCAL_ROOM,
                        cl._DETECT_LOCAL_REGION,
                        cl._DETECT_LOCAL_ORDER, 30, "")
