*SRD pipeline is implemented in separate package.*

# Relative Deviations

The package "Relative Deviations" implements the necessary tools and pipeline to detect deviations between a mesh model and a building (as-built). Currently, this tool focuses on flat walls (planes in general), but will be extended by other geometric primitives. Deviations are computed relatively to given references for a task. This is realized by doing "Selective ICP". The computed transformation from ICP can be used to update the pose estimation from the robot.

## Dependencies

Requires CGAL-4.13.1 (set in cgal_catkin)

## GTest

Run GTests:

```
catkin run_tests <package_name>
```

## Issues

Un-tick and tick backface culling for correct visualization.

## Installation

Pay attention not to overlay or chain other workspaces by sourcing ROS installation first:
```
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

```
mkdir -p ~/megabot_ws/src
cd ~/megabot_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --merge-devel
catkin config -DCMAKE_BUILD_TYPE=Release
```

```
cd ~/megabot_ws/src/
git clone git@github.com:ethz-asl/cad-percept.git
wstool init
wstool merge cad-percept/dependencies.rosinstall
wstool merge cad-percept/cpt_deviation_analysis/dependencies.rosinstall
wstool update
wstool merge eth_robotics_summer_school_2019/dependencies.rosinstall
wstool update
```

```
cd ~/megabot_ws/
catkin build cpt_deviation_analysis
```

```
source devel/setup.bash
echo "source ~/megabot_ws/devel/setup.bash" >> ~/.bashrc
```

Currently, from summer school only following packages and dependencies needed:

```
catkin build ethzasl_icp_mapper smb_tf_publisher smb_state_estimator
```

If necessary install other dependencies of summer school by apt.



## Launch

Terminal A:
```
roscore
```

Terminal B:

```
rosparam set use_sim_time true
rosbag play --clock <path_to_bag_file>/cla_garage_slam_1.bag
```

Terminal C:

```
roslaunch smb_state_estimator smb_state_estimator_standalone.launch
```

Terminal D:

```
roslaunch cpt_selective_icp supermegabot_selective_icp.launch
```

Terminal E:

```
roslaunch cpt_deviation_analysis online.launch
```

Terminal F:
** Why is Marker and Model only shown when rosbag is playing?**

```
roslaunch cad_interface cad_interface.launch
```

Now align /velodyne_points and then right-click on Marker and "Load CAD".
Now observe /corrected_scan and /ref_corrected_scan (only for selective ICP)

Hint:
- For better alignment of Marker, deactivate Mesh Model. Marker can not be accessed through mesh.
- There is a second interactive marker (red), which can be used to find closest facet ID to this marker. It is important to load the CAD first. Somehow the first projection on the facet is not shown in rviz.
- Pay attention when looking at visualizations in rviz since sometimes rviz is inaccurate with showing current message when simulation is paused. Deactivate and activate marker again to show correctly when simulation paused.
- Change SMB Confusor config according to robot: waco_calibration.cfg or smb_calibration.cfg

## Instructions

Use service to set reference facets, otherwise whole model is used with normal ICP and deviation analysis not triggered:

```
rosservice call /set_ref "data:
- 14
- 1"
```

e.g.

```
cd megabot_ws/src/cad-percept/cpt_selective_icp/script/
sh publish_references.sh
```

Analysis of complete map (callback still needs to run and mapping needs to be turned on in cpt_selective_icp):

```
rosservice call /analyze_map
```