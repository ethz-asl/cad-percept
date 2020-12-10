In the following, it will be assumed that a catkin workspace is available on the robot, that has the following packages installed (including the other packages on which the latter depend):
- This repo (`cad_percept`);
- The `mabi_mobile_robot` package from the `mabi_mobile` [repo](https://bitbucket.org/leggedrobotics/mabi_mobile/src/93e2ca550ebac4e64bb3330583959e7a88575aea/?at=feature%2Ftest_hal_routine), checked-out at branch `feature/test_hal_routine`.

To do so, follow the instructions below:
```bash
export HAL_WS=hal_test_ws
mkdir -p ${HAL_WS}/src
cd ${HAL_WS}
catkin init
catkin config --merge-devel
catkin config --extend /opt/ros/melodic/
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
# Clone and build cad-percept.
git clone git@github.com:ethz-asl/cad-percept.git
cd cad-percept
git checkout fmilano/cpt_pointlaser_EE_poses_visitor
cd ..
wstool init
wstool merge cad-percept/dependencies.rosinstall
wstool update -j8
catkin build cpt_pointlaser_hal
# Clone and build mabi_mobile.
cd ${HAL_WS}/src
git clone git@bitbucket.org:leggedrobotics/mabi_mobile.git
cd mabi_mobile
git checkout feature/test_hal_routine
cd ..
./mabi_mobile/mabi_mobile_deps/bin/clone_deps.sh
catkin build mabi_mobile_robot
```

It will also be assumed that a catkin workspace is available on the local computer, with the following packages installed:
- This repo (`cad_percept`);
- The `mabi_sim` package from [`mabi_common`](https://bitbucket.org/leggedrobotics/mabi_common), installed according the instructions of the [README](https://bitbucket.org/leggedrobotics/mabi_common/src/master/README.md).


Unless otherwise specified, *it is assumed that each of the terminals below have the catkin workspace sourced*.

The local computer is assumed to be connected to the robot through the usual SMB procedure:
```bash
connect-smb
```

To run the HAL routine:
- On the robot:
  - In Terminal 1, launch the HAL module and the interface to the arm controller:
    ```bash
    # If running in simulation, use argument `simulation:=true` and set the parameter `simulation_mode` to
    # `true` in `cpt_pointlaser_ctrl_ros/cfg/ee_poses_visitor_params_mabi.yaml`.
    roslaunch cpt_pointlaser_hal run_hal_routine.launch
    ```
  - In Terminal 2, launch the software stack for the robot:
    ```bash
    roslaunch mabi_mobile_robot hilti.launch
    ```
    This will launch:
    - The robot drivers;
    - The controllers;
    - The state estimator;
    - The URDF visualization;
    - The robot state publisher.

    In alternative, to run the software in simulation, cf. Terminal 2 below.
    In case this launch file is terminated (e.g., `Ctrl+C`) or some of its nodes crashes, you might have to fix some memory errors, by running `rosrun cosmo memory_remover`, which should display `Sucessfully removed shared memory pool 'COSMO_SHM'` in return.
  - In Terminal 3, enable the robot. NOTE: Make sure you know what you're doing.
    ```bash
    rosservice call /mabi_lowlevel_controller/enable_slave "id: 0 action: 1"
    ```
- On the local computer:
  - In Terminal 1, launch the interface to align the building model to the robot.
    ```bash
    roslaunch cpt_selective_icp cpt_selective_icp_hal_routine.launch
    ```
  - If the software should be run in simulation, in Terminal 2, launch the simulation GUI:
    ```bash
    roslaunch mabi_sim run_sim_hal_routine.launch
    ```
    Also, change `arm_controller_switch_service_name` to `/mabi_highlevel_controller/controller_manager/switch_controller` in `cpt_pointlaser_ctrl_ros/cfg/ee_poses_visitor_params_mabi.yaml`.
  - In Terminal 3, launch the interface that will guide the user through the routine:
    ```bash
    roslaunch cpt_pointlaser_hal hal_routine_guide.launch
    ```

To execute the routine:
1. Align mesh model to the robot.
2. Follow the instructions in Terminal 3 on the local computer.