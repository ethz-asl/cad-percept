# Reconstruction

The package "cpt_reconstruction" reconstructs missing elements from sensory data and integrates those into an incomplete building model.

# Dependencies
C++
- CGAL 5.1
- PCL 1.8
  
Python 2.0
- Open3D 0.9
- rospy
- tkinter
- numpy
- (COMPAS, plyfile) for JSON convertion

# Launch

- Stationary Total Station
  
  Terminal 1:
    ```
    roscore
    ```
  Terminal 2:
    ```
    roslaunch cpt_reconstruction reconstruction.launch 
    ```
#
- Mobile Robot:

    Terminal 1:
    ```
    roscore
    ```

  Terminal 2:
    ```
    rosparam set use_sim_time true
    rosbag play --clock 0.2 /example_path/example_file.bag
    ```

  Terminal 3:
    ```
    roslaunch cpt_selective_icp cpt_selective_icp_standalone.launch
    ```

  Terminal 4:
    ```
    roslaunch cpt_reconstruction reconstruction.launch 
    ```
