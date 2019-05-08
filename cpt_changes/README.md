# CPT Changes

Restore functionality from cadify integrated in cad-percept.

### To Do:

- Decide whether using cpt_utils or MeshLocalizer
- 

### Running Cadify:

#### Basic:
Running on bag files that already contain recorded mapping data requires only the cadify node to be launched.
You should run the following in several terminals:
##### Terminal A:
```
roscore
```
##### Terminal B:
```
rosparam set use_sim_time true
rosbag play --clock <path_to_bag_file>/cla_garage_slam_1.bag
```
##### Terminal C:
```
rosrun rviz rviz
```
##### Terminal D:
```
roslaunch cpt_changes changes.launch
```
##### Terminal E:
```
rosservice call /transformModel
```