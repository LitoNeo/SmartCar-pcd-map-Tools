###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 # @Date: 2020-01-13 12:44:11
 # @LastEditors  : LitoNeo
 # @LastEditTime : 2020-01-13 13:13:49
 ###
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source scripts/smartcar_base.sh

function start(){
    echo "Start roscore..."
    scripts/roscore.sh start

    echo "Start rviz..."
    scripts/rviz.sh start ${DIR}/../map_tools/modules/trajectory_generator/rviz/rviz_config_traj_gene.rviz

    echo "Start traj_generator..."
    nohup rosrun traj_generator traj_generator 2>&1 &
}

function stop(){
    pkill -SIGTERM -f traj_generator
    echo "traj_generator: stopped"

    scripts/rviz.sh stop
    echo "rviz: stopped"

    scripts/roscore.sh stop
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
    echo "choose start or stop"
    ;;
esac
