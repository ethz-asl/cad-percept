# CPT Changes

- Restored functionality from cadify ROS node integrated in cad-percept

### To Do:

- Decide whether using cpt_utils or MeshLocalizer -> it just one function now: associatePointcloud()
- Add gtests
- Move visualization stuff in cgal_visualizations (if possible)
- Visualization of triangles in color according to distance
- start rviz from launch file and add default.rviz
- check if current visualization result makes sense

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