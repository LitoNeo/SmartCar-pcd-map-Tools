###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 ###
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASE_DIR="cd "${SCRIPT_DIR}/.."; pwd"
GRIDMAP_GENERATOR_DIR="cd "${BASE_DIR}/map_tools/grid_map_generator"; pwd"
TRAJ_GENERATOR_DIR="cd "${BASE_DIR}/map_tools/trajectory_generator"; pwd"

source "${SCRIPT_DIR}/smartcar_base.sh"

function start(){
    echo "Start roscore..."
    ${SCRIPT_DIR}/roscore.sh start
    sleep 1

    echo "Start rviz..."
    ${SCRIPT_DIR}/rviz.sh start

    echo "Start traj_generator..."
    nohup rosrun traj_generator traj_generator 2>&1 &
}

function stop(){
    pkill -SIGTERM -f traj_generator
    echo "traj_generator: stopped"

    ${SCRIPT_DIR}/rviz.sh stop
    echo "rviz: stopped"

    ${SCRIPT_DIR}/roscore.sh stop
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
