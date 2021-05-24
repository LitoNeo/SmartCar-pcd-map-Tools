###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 ###
# set -x

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"; pwd )"
BASE_DIR="$( cd ${SCRIPT_DIR}/..; pwd )"
GRIDMAP_GENERATOR_DIR="$( cd "${BASE_DIR}/map_tools/modules/grid_map_generator"; pwd )"
TRAJ_GENERATOR_DIR="$( cd "${BASE_DIR}/map_tools/modules/trajectory_generator"; pwd )"

source "${SCRIPT_DIR}/smartcar_base.sh"

CATKIN_DIR="$( cd ${BASE_DIR}/../..; pwd )"
ENV_BASH_FILE="${CATKIN_DIR}/devel/setup.bash"
if [ ! -f ${ENV_BASH_FILE} ];then
    echo "${ENV_BASH_FILE} not found."
    echo "Please build this project first"
    exit 1
fi
source ${ENV_BASH_FILE}

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
    echo "choose start or stop argument"
    ;;
esac
