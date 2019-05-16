# CPT Changes

- Restored functionality from cadify ROS node integrated in cad-percept
- Added visualization of distances (point-to-mesh) as colored triangles (discrete)
- Added "heatmap" visualization of distances with color gradient by setting ROS parameter "discrete_color" to "false"

### To Do:

- Add ray-casting from robot and not just to nearest facet
- Decide whether using cpt_utils or MeshLocalizer -> it's just one function now: associatePointCloud()
- Add gtests
- If necessary make visualization subclass
- start rviz from launch file and add default.rviz
- check if current visualization result makes sense
- Decide if additional rviz Display type is necessary

### Running cpt_changes:

#### Basic:
Running on bag files that already contain recorded mapping data requires only the changes node to be launched.
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