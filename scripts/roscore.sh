###
 # @Author: LitoNeo
 # @Date: 2020-01-13 12:39:26
 # @LastEditTime : 2020-01-13 12:49:25
 # @LastEditors  : LitoNeo
 # @Description: In User Settings Edit
 # @FilePath: /gitspace/src/SmartCar-Tools/scripts/roscore.sh
 ###
#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source scripts/smartcar_base.sh

function start() {
    echo "Start roscore..."
    ROSCORELOG="${SMARTCAR_ROOT_DIR}/data/log/roscore.out"
    nohup roscore </dev/null >"${ROSCORELOG}" 2>&1 &
}

function stop() {
    pkill -SIGTERM -f roscore
    echo "roscore: stopped"
}

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    echo "choose start | stop"
    ;;
esac