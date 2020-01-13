<!--
 * @Descripttion: 
 * @version: 
 * @Author: LitoNeo
 * @Date: 2020-01-13 13:27:48
 * @LastEditors  : LitoNeo
 * @LastEditTime : 2020-01-13 14:05:45
 -->
## PCD Grid Divider
`PCD Grid Divider` is used to divide huge point cloud files into regularly arranged gridded point cloud files.

### How to use
```shell
user:~ cd Smartcar-Tools
user:~/Smartcar-tools ./scripts/grid_map_generator.sh start # start
# this will start 3 process, roscore/rviz and traj_generator

user:~/Smartcar-tools ./scripts/grid_map_generator.sh stop  # stop
```

<img src="https://user-images.githubusercontent.com/44689665/72235385-e05e3800-360c-11ea-9994-03f4490315ee.png" />

As shown, it is very simple to use.
1. **input**
folder contains origin pcd files

2. **output**
folder to save processed pcd files.

3. **grid size**
the grid size you want.
> the output pcd files will arranged in grid_size x grid_size

4. **voxel size**
used to downsample the pcd files.
set to 0 to forbidden downsampling.

