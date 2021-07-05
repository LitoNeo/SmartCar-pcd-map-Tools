<!--
 * @Descripttion: 
 * @version: 
 * @Author: LitoNeo
 * @Date: 2020-01-13 13:26:13
 * @LastEditors  : LitoNeo
 * @LastEditTime : 2020-01-13 14:10:25
 -->
# SmartCar-Tools
Light-weight tools for self-driving car

## Description
For the needed of our project, I designed two small tools currently:
1. [grid_map_generator](https://github.com/LitoNeo/SmartCar-pcd-map-Tools/tree/master/map_tools/modules/grid_map_generator): Used to downsample the pcd maps and divide one big pcd map to small one, which are the input of `pcd_map_manager`.
2. [trajectory_generator](https://github.com/LitoNeo/SmartCar-pcd-map-Tools/tree/master/map_tools/modules/trajectory_generator): Used to extract defined driving paths (trajectoies), which is a light-weight description of roads.

**For the useage of each tool, refer to their own `README` document**

## How to Build
This repo depends on QT-5.9 and ROS1.0, be sure it is correctly installed on your pc.
I tested it on `Ubuntu1604` and 'Ubuntu1804'

1. `mkdir -p catkin_ws/src && cd catkin_ws/src`
2. `git clone https://github.com/LitoNeo/SmartCar-Tools.git`
3. `cd .. && catkin_make`

> Note: Just as general ROS modules, you need to `source devel/setup.bash` before running the tools

## map_tool modules
> Tools used to process map data.(especially for pcd files)
1. **trajectory_generator**
A qt-based tools used to define and extract drivable trajectory.
Currently it only extracts drivable trajectories, that is, a minimum subset of high-precision maps. But it is flexible to extend.

2. **grid_map_generator**
A qt-based tools to downsample and mesh the big pcd maps to small one.   
For more details, refer [here](https://zhuanlan.zhihu.com/p/77745476)

