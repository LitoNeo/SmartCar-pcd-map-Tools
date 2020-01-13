###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 # @Date: 2020-01-13 12:44:11
 # @LastEditors  : LitoNeo
 # @LastEditTime : 2020-01-13 13:58:10
 ###
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source scripts/smartcar_base.sh

function start(){
    echo "Start grid map generator..."
    nohup rosrun grid_map_generator pcd_grid_divider 2>&1 &
}

function stop(){
    pkill -SIGTERM -f pcd_grid_divider
    echo "grid map generator: stopped"
}

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    echo "choose start or stop"
    ;;
esac
