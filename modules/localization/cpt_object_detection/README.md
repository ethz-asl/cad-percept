# cpt object detection
This package contains a node refining object detections from a rough pointcloud 
to the exact position of the provided object mesh.

## Basic usage
A demo of the usage of this package can be launched with
```
roslaunch cpt_object_detection object_detection_ICP.launch
```
This will start the object detection node `cpt_object_detection_node`, together
with an RVIZ visualization and a rosbag containing the previous detection steps.
The bagfile can be downloaded [here](https://drive.google.com/file/d/1fhr-uqZKUYzn4yu6QJWsOqIJrIEoDE5Z/view?usp=sharing).

Alternatively, the detection pointcloud can be obtained using the 
`piloting_detector` package from the [`multisensor_tools` library](https://github.com/ethz-asl/multisensor_tools/tree/detector/piloting_detector):   
```
roslaunch piloting_detector master.launch
```

## Code explained
### Input
The objective is to match a predefined mesh of an object with the pointcloud of 
its detection in space.
The object mesh is loaded from an .off file, where the filepath is given in the
parameter ``off_model``.
The node subscribes to a pointcloud via the ROS network. 
The topic name is set using the parameter ``pointcloud_topic``.

### Output
As a result of the object detection matching, we publish the transform from the 
frame of the detection pointcloud to the local frame of the object mesh 
``object_detection_mesh`` to the TF tree.
The object mesh is also published in its local frame and can be visualized 
using rviz.
The name of the object mesh frame can be set using the parameter 
`object_frame_id` in the launch file.

### Algorithm
The object mesh can be visualized upon startup of the node using the 
parameter ``visualize_object_on_startup``.

The function ``processDetectionUsingPcaAndIcp`` contains all processing steps for each 
pointcloud received by the node.
The PCA-ICP pipeline aims to find the transformation by performing ICP on an 
initial guess obtained by a PCA on the detection pointcloud and the object mesh. 

The initial guess is obtained in the function ```pca```.
After performing PCA on both the detection pointcloud and the object mesh, we find 
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

### Parameters
The following list summarizes the parameters that can be set in the launch file:
- `off_model`  
  The filepath where the object mesh is stored as a `.off` file.
  No default value.
- `pointcloud_topic`  
  The name of the topic we subscribe to containing the 
  preprocessed pointclouds of the object detection.  
  Default value: `/camera/depth/color/points`
- `icp_config_file`  
  The filepath of the config file for ICP used by 
  `libpointmatcher`.  
  No default value. 
- `object_frame_id`  
  The frame of the aligned object mesh published to the TF 
  tree, connected by a transformation from the frame of the pointcloud.  
  Default value: `object_detection_mesh`
- `visualize_object_on_startup`  
  Visualizes the loaded object mesh in the frame 
  of the pointcloud.  
  Default value: `false`  
- `queue_size`  
  The queue size of the ROS subscriber to the pointcloud.  
  Default value: `1`