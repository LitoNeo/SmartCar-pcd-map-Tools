# Trajectory_generator
A qt-based tools used to define and extract drivable trajectory.

Currently it only extracts drivable trajectories, that is, a minimum subset of high-precision maps. But it is flexible to extend.

## Lane & Cross
<img src="https://user-images.githubusercontent.com/44689665/72234852-ec94c600-3609-11ea-9496-aa93a9cb1108.jpg" />

For simple, we only define two types of trajectory - Lane and Cross, as shown in the image.

Lane and Cross are connected one by one, and should be defined in advance.

## How to use

```shell
user:~ cd Smartcar-Tools
user:~/Smartcar-tools ./scripts/traj_generator.sh start # start
# this will start 3 process, roscore/rviz and traj_generator

user:~/Smartcar-tools ./scripts/traj_generator.sh stop  # stop
```

<img src="https://user-images.githubusercontent.com/44689665/72234872-0e8e4880-360a-11ea-8841-fb099090223e.png" />


1. **input Folder**
folder contains pcd files

2. **output Folder**
folder to save trajectory files(.csv)

3. **trajectory info**
each lane/cross contains 5 elements:
```shell
type: lane/cross
id: its only identity
pre_ids: the crosses'/lanes' id before it  # use english dot(,) to distinguish
next_ids: the crosses'/lanes' id after it
reverse: define if it is reversable
```
4. **weight_data/weight_smooth/tolerance**
controls the smooth level of the trajectory

5. **Start**
when input_folder and output_folder is correctly set, press start

6. **->create**
In the rviz pannel, use `2D pose estimate` button to define your trajectory, when finished, use `trajectory_info` to define the attributes of the trajectory, and then pre `->create` to create the trajectory. that's simple.

7. **del one point**
A regret-button, one point at one press.

8. **Finish**
When you finish all the trajectories, press `Finish`, this will write all trajectory_.csv files to `output_folder`

