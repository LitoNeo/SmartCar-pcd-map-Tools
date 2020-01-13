###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 # @Date: 2020-01-13 12:58:37
 # @LastEditors  : LitoNeo
 # @LastEditTime : 2020-01-13 13:01:22
 ###
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source scripts/smartcar_base.sh

function start() {
    echo "Start rviz..."
    RVIZ_CONFIG_FILE="$2"
    nohup rosrun rviz rviz -d "${RVIZ_CONFIG_FILE}" 2>&1 &
}

function stop() {
    pkill -SIGTERM -f rviz
    echo "rviz Stopped."
}

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    echo "[rviz] please choose config file & start stop"
    ;;
esac