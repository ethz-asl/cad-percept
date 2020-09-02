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

### Algorithm
All initial processing of the object mesh is performed in the function 
``processObject``, called by the constructor of the node. 
The object mesh can be visualized upon startup of the node using the 
parameter ``visualize_object_on_startup``.

The function ``processPointcloudUsingPcaAndIcp`` contains all processing steps for each 
pointcloud received by the node.
The PCA-ICP pipeline aims to find the transformation by performing ICP on an 
initial guess obtained by a PCA on the detection pointcloud and the a pointcloud 
sampled from the object mesh. 
The number of sampled points is given as the parameter 
`num_points_object_pointcloud`.

The initial guess is obtained in the function ```findInitialGuessUsingPCA```.
After performing PCA on both the detection and the object pointcloud, we find 
the transform using the  translation between the centroids of the pointclouds and
the rotation between the coordinate systems defined by the eigen vectors, 
oriented to form a right-handed system. 
The resulting transform is additionally published to the TF tree as 
``object_detection_mesh_init``.

The more precise alignment is found using ICP in the function ``icp``.
We use the implementation of the library ``libpointmatcher``.
In order to speed up the process, both the detection and the object pointclouds
are uniformly sampled to contain a maximum number of points, defined in the 
parameter ``num_points_icp``.
The results are then published as described previously.