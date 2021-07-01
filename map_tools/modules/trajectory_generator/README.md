Trajectory_generator
---
Used to define and extract drivable trajectory.

Currently it only extracts drivable trajectories, that is, a minimum subset of high-precision maps. But it is flexible to extend.

**For prior Lane&Cross defination, refer to `README_lane_cross.md`, this tool will still be compatible with that.**

---

## Defination
![](https://user-images.githubusercontent.com/44689665/124111704-28d91200-da9c-11eb-9aad-8495bbbb821e.png)

Same with general graph defination, there are two definations:
* `node` is where two edges connected, it has only id property;
* `edge` is our wanted trajectories, it has 5 properties currently which can be found at next column;
> Note: No two nodes should share more than one direct link

Our goal is to get the defined edge to suppot planning module.

## Lane & Cross
each lane/cross contains 5 elements:
```shell
type: string   #lane/cross
id: int        # its only identity
pre_ids: int   # prior node id   
next_ids: int  # next node id
reverse: bool  # define if it is reversable
```
**`pre_ids` and `next_ids` are defined according to the trajectory's drawing direction**

## How to use
0. Define your default trajectories, includes its ids and connection relationship.
And then launch this tool by:
```bash
roslaunch traj_generator run_trajectory_generator.launch
```

1. set `input foloder` and `output folder`
   * the `input folder` should contain only the .pcds of your built map
   * all the output will be saved under `output folder`, it's recommended that it is empty

2. press `start` and waitting for the map to be loaded
After map loaded, the left buttons will be changed to status AVAILABLE

3. Draw your defined trajectories
In the rviz pannel, use `2D pose estimate` button to define your trajectory from one side to another. This tool will automatically fill the points between your drawing positions, and smooth it.
You can use `del one point` button to delete current drawing trajectory's point from end side, one press for one point.
when finished, goto `trajectory_info` to define the attributes of the trajectory, and then pre `->create` to create the trajectory. that's all.

4. Press `Finish` button after all trajectories finished
**Only by this way, your trajetories will be saved under `output folder`, so do not forget it**