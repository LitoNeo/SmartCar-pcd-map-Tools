###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 # @Date: 2020-01-13 12:58:37
 # @LastEditors  : LitoNeo
 # @LastEditTime : 2020-01-13 13:01:22
 ###

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"; pwd )"
BASE_DIR="$( cd ${SCRIPT_DIR}/..; pwd )"
TRAJ_GENERATOR_DIR="$( cd "${BASE_DIR}/map_tools/modules/trajectory_generator"; pwd )"
RVIZ_CONFIG_FILE=${TRAJ_GENERATOR_DIR}/rviz/rviz_config_traj_gene.rviz


# source scripts/smartcar_base.sh

function start() {
    echo "Start rviz..."
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