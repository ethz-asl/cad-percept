*This package corresponds to the SRD pipeline and state of intermediate presentation. RD pipeline is implemented in separate package.*

# Relative Deviations

To package "Relative Deviations" implements the necessary tools and pipeline to detect deviations between a mesh model and a building (as-built). Currently, this tool focuses on flat walls (planes in general), but will be extended by other geometric primitives. Deviations are computed relatively to given references for a task. This is realized by doing "Selective ICP". The computed transformation from ICP can be used to update the pose estimation from the robot.

## Scenarios

### Static Room Deviations (SRD)

"Static Room Deviations" describes a test case for demonstration and testing. A 3D mesh model of a simple room is created and converted into a deviated point cloud for testing purposes. The same pipeline as for online usage is used, however, the used point cloud is much denser and complete than what we get from a LiDAR.

"srd.launch" launches a demonstration of the pipeline. A mesh model and a corresponding deviated reading pointcloud is created. The pipeline is executed just once on this static data. Parameter "test" is set to "true".

```
roslaunch srd_relative_deviations srd.launch
```

![Result](resources/discrete_deviation_threshold_15mm_2.png)

### Relative Deviations (RD)

"Relative Deviations" describes the real, online scenario. The ROS node subscribes to the robot point cloud and pose estimation. It acts itself as a mapper by using a sliding window of several scans, matching it to the mesh model by ICP and returning the updated state estimation to the estimator. 

RD is not implemented in this package.

## Status

- [x] SRD tested and working
- [x] Adapted to new findCoplanarFacets

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
wstool merge cad-percept/relative_deviations/dependencies.rosinstall
wstool update
wstool merge eth_robotics_summer_school_2019/dependencies.rosinstall
wstool update
```

```
cd ~/megabot_ws/
catkin build relative_deviations
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