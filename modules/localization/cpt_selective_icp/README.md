# CPT Selective ICP

Package replacing ethzasl_icp_mapper by a selective ICP algorithm taking a reference map (mesh) and a list of references as input for better alignment.

## About

This package is a complete alternative to ethzasl_icp_mapper. Instead of a point cloud reference, a mesh model is loaded and used for proper localization. For this reason, a point cloud is sampled at the beginning. In this default case, the package offers the same performance as ethzasl_icp_mapper.

Furthermore, this package offers selective ICP, i.e. only using chosen reference facets and their coplanar neighbors for localization. On a construction site with as-built deviations, this should offer proper localization based on known references. However, it is a trade-off between the chosen reference facets (number of facets, visibility, angle) and the advantage of only using correct references. The references need to be chosen smartly, because otherwise selective ICP can fail if the robot is moving. In the worst case, selective ICP gives worse results than full ICP because of missing visibility of references or ambiguous reference facets. 

A package containing both full ICP and selective ICP has the advantage that we can continuously and automatically switch between the two ICP methods. Both ICP methods are initialized separately. Full ICP is set-up completely at the beginning, while selective ICP is set-up whenever reference facets are changed. Full ICP and selective ICP can run in parallel. In this case selective ICP uses full ICP as a primer. Assuming selective ICP gives a better result, this is always sent to the position estimator. However, each methods can be turned on and off separately by a service and in this case the corresponding output is sent to the estimator.
There are situations where only selective ICP corrected scans are of interest (deviation analysis). Therefore, there is a separate scan topic only published in the case of selective ICP. 

If selective ICP fails with a convergence error, the package automatically uses the result from full ICP (if any). However, a known limitation is the case where selective ICP gives a completely wrong transformation without a convergence error. Selective ICP is more prone to this error than full ICP due to limited points for alignment. If the deviation is too large, this can break the position estimate completely.

Real-time capability is important, because if ICP takes too long, the estimator can break. In that case the estimator already moved too far away from the model using the other informations (IMU, odometry) so that ICP can not correct it anymore.

The node also offers a mapping thread based only on selective ICP references. The built reference map can further be used for selective ICP avoiding the low number of references when moving around.

## Instructions

Use service to set reference facets, otherwise whole model is used with full ICP:
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
roslaunch smb_state_estimator smb_state_estimator_standalone.launch
```

Terminal D:

```
roslaunch cpt_selective_icp supermegabot_selective_icp_with_rviz.launch
```

Terminal E: (deprecated)

```
roslaunch cad_interface cad_interface.launch
```

Now align /velodyne_points and then right-click on Marker and "Load CAD".
Now observe /corrected_scan and /ref_corrected_scan (only for selective ICP)

## Hints

- For better alignment of Marker, deactivate Mesh Model. Marker can not be accessed through mesh.

- There is a second interactive marker (red), which can be used to find closest facet ID to this marker. It is important to load the CAD first. Somehow the first projection on the facet is not shown in rviz.

