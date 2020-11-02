#!/usr/bin/bash

export LD_LIBRARY_PATH=/data/data/com.termux/files/usr/lib
export HOME=/data/data/com.termux/files/home
export PATH=/usr/local/bin:/data/data/com.termux/files/usr/bin:/data/data/com.termux/files/usr/sbin:/data/data/com.termux/files/usr/bin/applets:/bin:/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin:/data/data/com.termux/files/usr/bin/python
export PYTHONPATH=/data/openpilot


cd /data/openpilot/_backup/common && cp can.h /data/openpilot/panda/board/drivers;

cd /data/openpilot/_backup/common && cp main.c /data/openpilot/panda/board;

cd /data/openpilot/_backup/common && cp safety_declarations.h /data/openpilot/panda/board;

cd /data/openpilot/_backup/common && cp safety_defaults.h /data/openpilot/panda/board/safety;

cd /data/openpilot/panda; pkill -f boardd; PYTHONPATH=..; python -c "from panda import Panda; Panda().flash()"; reboot
