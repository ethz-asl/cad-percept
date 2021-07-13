# CPT Selective ICP Dyn: 

Package replacing ethzasl_icp_mapper by a selective ICP algorithm taking a reference map (mesh) and a list of references as input for better alignment. Odometry from a Realsense Tracking Camera is used as a prior for ICP.

## About

This package is a complete alternative to ethzasl_icp_mapper. Instead of a point cloud reference, a mesh model is loaded and used for proper localization. For this reason, a point cloud is sampled at the beginning. In this default case, the package offers the same performance as ethzasl_icp_mapper.

Furthermore, this package offers selective ICP, i.e. only using chosen reference facets and their coplanar neighbors for localization. On a construction site with as-built deviations, this should offer proper localization based on known references. However, it is a trade-off between the chosen reference facets (number of facets, visibility, angle) and the advantage of only using correct references. The references need to be chosen smartly, because otherwise selective ICP can fail if the robot is moving. In the worst case, selective ICP gives worse results than full ICP because of missing visibility of references or ambiguous reference facets. 

A package containing both full ICP and selective ICP has the advantage that we can continuously and automatically switch between the two ICP methods. Both ICP methods are initialized separately. Full ICP is set-up completely at the beginning, while selective ICP is set-up whenever reference facets are changed. Full ICP and selective ICP can run in parallel. In this case selective ICP uses full ICP as a primer. Assuming selective ICP gives a better result, this is always sent to the position estimator. However, each methods can be turned on and off separately by a service and in this case the corresponding output is sent to the estimator.
There are situations where only selective ICP corrected scans are of interest (deviation analysis). Therefore, there is a separate scan topic only published in the case of selective ICP. 

If selective ICP fails with a convergence error, the package automatically uses the result from full ICP (if any). However, a known limitation is the case where selective ICP gives a completely wrong transformation without a convergence error. Selective ICP is more prone to this error than full ICP due to limited points for alignment. If the deviation is too large, this can break the position estimate completely.

Real-time capability is important, because if ICP takes too long, the estimator can break. In that case the estimator already moved too far away from the model using the other informations (IMU, odometry) so that ICP can not correct it anymore.

The node also offers a mapping thread based only on selective ICP references. The built reference map can further be used for selective ICP avoiding the low number of references when moving around.

## Usage:
`realsense_broadcast_node` takes the odometry from a topic and broadcasts it to the TF tree. This is required to the final mapper node. `cpt_selective_icp_dyn_node` does ICP of a pointcloud against a mesh and uses an odometry as a pose prior. Therefore, `scanTopic` and `odomTopic` need to be specified. It is possible to skip LiDAR scans by taking only every `skipScans` scan for ICP. Instead of directly using the odometry as a pose prior for ICP, an Extended Kalman Filter is used, if `ekfEnable` is set to true. The nodes can be used as follows  (taken from `background_foreground_segmentation/launch/realsense_experiments/pickelhaube2_rt.launch`):

```xml
    <node name="realsense_broadcast"
          type="realsense_broadcast_node"
          pkg="cpt_selective_icp_dyn"
          output="screen" >
          <param name="odomTopic" value="/camera/odom/sample" />
          <param name="realsenseOdomTopic" value="/realsense_odom" />
          <param name="inputQueueSize" value="5" />
          <param name="tfMapFrame" value="map" />
          <param name="cameraPoseFrame" value="camera_pose_frame" />
          <param name="cameraOdomFrame" value="camera_odom_frame" />
          <param name="realsenseCov" value="0.1" />
    </node>

    <node name="mapper"
          type="cpt_selective_icp_dyn_node"
          pkg="cpt_selective_icp_dyn"
          output="screen" >
          <!--launch-prefix="tmux split-window" -->
    <rosparam command="load" file="$(find background_foreground_segmentation)/launch/realsense_experiments/mapper_parameters_dyn.yaml" />
                <param name="scanTopic" value="/rslidar_points" />
                <param name="odomTopic" value="/camera/odom/sample" />
                <param name="skipScans" value="5" /> 
                <param name="ekfEnable" value="false" />

        <param name="cadTopic" value="mesh_publisher/mesh_out" />
        <param name="icpConfig"
               value="$(find background_foreground_segmentation)/launch/paper_experiments/full_icp.yaml" />
        <param name="selectiveIcpConfig"
               value="$(find background_foreground_segmentation)/launch/paper_experiments/selective_icp.yaml" />
        <param name="inputFiltersConfig"
               value="$(find background_foreground_segmentation)/launch/paper_experiments/input_filters.yaml" />
        <param name="mapPostFiltersConfig"
               value="$(find background_foreground_segmentation)/launch/paper_experiments/map_post_filter.yaml" />
        <param name="mapPreFiltersConfig"
               value="$(find background_foreground_segmentation)/launch/paper_experiments/map_pre_filter.yaml" />
        <param name="path"
                 value="$(find cpt_selective_icp_dyn)" />
    </node>
```
`cloud_distance_node` takes a pointcloud and computes the distance to the mesh for each point. In `background_foreground_segmentation/launch/dense_labels_experiments/pickelhaube2_labelgen_dyn_dense_sparse_office.launch`, it is used to generate dense pseudo-labels from a depth pointcloud: 

```xml
       <node name="cloud_distance"
          type="cloud_distance_node"
          pkg="cpt_selective_icp_dyn"
          output="screen" >
          <param name="cadTopic" value="mesh_publisher/mesh_out" />
          <param name="cloudInTopic" value="/points2" />
          <param name="inputQueueSize" value="5" />
          <param name="tfMapFrame" value="map" />
          <param name="cloudOutTopic" value="/depth/distance_pc" />
          <param name="cloudFrame" value="pickelhaubedepth_camera_link" />
          <param name="mapSamplingDensity" value="100" />
       </node>
```

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

