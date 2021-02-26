# cpt_object_detection
This package contains a node refining object detections from a rough point cloud 
to the exact position of the provided object mesh.

## Table of Content
- [Basic usage](#basic-usage)
- [Code explained](#code-explained)
  - [Input](#input)
  - [Output](#output)
  - [Algorithm](#algorithm)
    - [PCA-ICP](#pca-icp)
    - [3D Features](#3d-features)
  - [Detection filters](#detection-filters)
- [Parameter overview](#parameters)

## Basic usage
A demo of the usage of this package can be launched with
```
roslaunch cpt_object_detection object_detection_mesh.launch
```
This will start the object detection node `cpt_object_detection_node`, together
with an RVIZ visualization and a rosbag containing the previous detection steps.
The bagfile can be downloaded [here](https://drive.google.com/file/d/1fhr-uqZKUYzn4yu6QJWsOqIJrIEoDE5Z/view?usp=sharing).

This package was designed to be used in combination with the detection 
point cloud provided by the `piloting_detector` package from the 
[`multisensor_tools` library](https://github.com/ethz-asl/multisensor_tools/tree/detector/piloting_detector).
To start the combined detection, launch
```
roslaunch cpt_object_detection detection_piloting.launch
```

## Code explained
### Input
The objective is to match a predefined mesh of an object with the point cloud of 
its detection in space.
The object mesh is loaded from an .off file, where the filepath is given in the
parameter ``off_model``.
The node subscribes to a point cloud via the ROS network. 
The topic name is set using the parameter ``detection_pointcloud_topic``.
For better alignment, a point cloud of the scene of the detection may additionally
be provided, using the topic ``scene_pointcloud_topic``.

### Output
As a result of the object detection matching, we publish the transform from the 
frame of the detection point cloud to the local frame of the object mesh 
``object_detection_mesh`` to the TF tree.
The object mesh is also published in its local frame and can be visualized 
using rviz.
The name of the object mesh frame can be set using the parameter 
`object_frame_id` in the launch file.

### Algorithm
The object mesh can be visualized upon startup of the node using the 
parameter ``visualize_object_on_startup``.

#### PCA-ICP
The function ``processDetectionUsingPcaAndIcp`` contains all processing steps for each 
point cloud received by the node.
The PCA-ICP pipeline aims to find the transformation by performing ICP on an 
initial guess obtained by a PCA on the detection point cloud and the object mesh. 

The initial guess is obtained in the function ```pca```.
After performing PCA on both the detection point cloud and the object mesh, we find 
the transform using the translation between the centroids and
the rotation between the coordinate systems defined by the eigen vectors, 
oriented to form a right-handed system. 
The resulting transform is additionally published to the TF tree as 
``object_detection_mesh_init``.

The more precise alignment is found using ICP in the function ``icp``.
We use the implementation of the library ``libpointmatcher``.
The object mesh is transformed to a set of data points used in libpointmatcher,
also containing the normals of the mesh. 
The ICP parameters are loaded from a yaml file, whose file path is given by
the argument ``icp_config_file``.
The results are then published as described previously.

#### 3D Features
A further approach is given by the use of 3D features of the detection and the object point cloud.
The function `processDetectionUsing3dFeatures` is called for every detection point cloud.
After an initial downsampling of the point cloud, we compute the features using the function
`compute3dFeatures` and the corresponding transform using `computeTransformUsing3dFeatures`.
The alignment is further refined using ICP as described in the previous section.
The following configurations of 3D features are supported: 

3D keypoints
- ISS
- Harris
- Uniform sampling

3D Descriptors
- FPFH
- SHOT

Matching
- Geometric Consistency
- Fast Global Registration
- Teaser

### Detection filters
To make the detections more stable with time, two filters are implemented:
* Inlier Ratio, using a minimum inlier ratio and determining the initial transformation for the final ICP alignment based on the higher inlier ratio, and
* Kalman Filter.

## Parameter overview
The following list summarizes the parameters that can be set in the launch file:
- `off_model`  
  The filepath where the object mesh is stored as a `.off` file.
  No default value.
  
- `reference_frame_id`  
  Reference frame in which the object is assumed to be static.  
  Default value: `arm_base_link`
  
- `object_frame_id`  
  The frame of the aligned object mesh published to the TF 
  tree, connected by a transformation from the frame of the point cloud.  
  Default value: `object_detection_mesh`
  
- `use_3d_features`  
  True for using 3D features to align the detection point cloud and the object mesh,
  false for using the PCA-ICP pipeline.  
  Default value: `true`
  
- `refine_using_icp`  
  The transformation found using 3D features is further refined using ICP.  
  Default value: `true`

- `icp_config_file`  
  The filepath of the config file for ICP used by
  `libpointmatcher`.  
  Default value: `config/icp_config.yaml`

- `use_inlier_ratio_filter`  
  Activate a filter on the object position based on the inlier ratio.
  For every new detected position, its inlier ratio is compared to the 
  current inlier ratio corresponding to the last published position. 
  The new position is only published, if its inlier ratio exceeds the ratio of
  the last published position.  
  Default value: `true`

- `use_kalman_filter`  
  Activate a Kalman filter on the object position.
  The corresponding parameters can be set in the `detection_config.yaml` file.  
  Default value: `true`

Additional parameters can be set in the `detection_config.yaml` file, such as:
- `detection_pointcloud_topic`  
  The name of the topic we subscribe to containing the
  preprocessed point clouds of the object detection.  
  Default value: `/detection_projector/object_pcl_3d`

- `scene_pointcloud_topic`  
  The name of the topic we subscribe to containing the
  preprocessed point clouds of the object detection.  
  Default value: `/detection_projector/scene_pcl`

- `downsampling`  
  Resolution in meters to which the detection point cloud is downsampled.  
  Default value: `0.001`

- `use_icp_on_pointcloud`  
  If set to `true`, ICP is based on the object mesh using `libpointmatcher`.
  Otherwise, a point cloud is sampled from the mesh model and used for the ICP
  alignment.  
  Default value: `true`

- `num_points_object_pointcloud`  
  Number of points sampled from the object mesh to compute the 3D features.  
  Default value: `1000`

- `keypoint_type`  
  Type of keypoints used for the 3D features.  
  Options: `ISS`, `Harris`, `uniform`, `all`; default value: `Harris`

- `descriptor_type`  
  Type of descriptors used for the 3D features.  
  Options: `FPFH`, `SHOT`, `3DSmoothNet`, `unit`; default value: `unit`

- `matching_method`  
  Method to match the 3D features and compute the transform between them.  
  Options: `geometric_consistency`, `FGR`, `Teaser`; default value: `Teaser`

- `correspondence_threshold`  
  Threshold value above which a correspondence between two 3D features are considered to be a good
  match. If set to zero, the correspondences are not filtered beforehand.  
  Default value: `0.1`
  