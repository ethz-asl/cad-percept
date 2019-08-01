# CPT Selective ICP

Package replacing ethzasl_icp_mapper by a selective ICP algorithm taking a reference map (mesh) and a list of references as input for better alignment.

## Instructions

Use service to set reference facets, otherwise whole model is used:
```
rosservice call /set_ref "data:
- 14
- 1"
```

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
roslaunch cad_interface cad_interface.launch
```

Terminal D:

```
roslaunch smb_state_estimator smb_state_estimator_standalone.launch
```

Terminal E:

```
roslaunch cpt_selective_icp supermegabot_selective_icp.launch
```
