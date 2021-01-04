In the following, it will be assumed that a catkin workspace is available on the robot, that has the following packages installed (including the other packages on which the latter depend):
- This repo (`cad_percept`);
- The `mabi_mobile_robot` package from the `mabi_mobile` [repo](https://bitbucket.org/leggedrobotics/mabi_mobile/src/08f6e1dad2588fb3a13c20b605b8be46180a90d1/?at=feature%2Ftest_hal_routine), checked-out at branch `feature/test_hal_routine`.

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
- The `mabi_sim` package from [`mabi_common`](https://bitbucket.org/leggedrobotics/mabi_common), installed according the instructions of the [README](https://bitbucket.org/leggedrobotics/mabi_common/src/851ba711753c80204e4eefd785d05d0d97f1ebf4/README.md?at=feature%2Ftest_hal_routine), branch `feature/test_hal_routine`.


Unless otherwise specified, *it is assumed that each of the terminals below have the catkin workspace sourced*.

The local computer is assumed to be connected to the robot through the usual SMB procedure:
```bash
connect-smb
```

To run the HAL routine:
1. Turn the robot on. Note: both the base and the computer need to be turned on, and all the e-stop need to be not enabled before turning the computer on.
2. On the robot:
    - Enable (i.e., push the red button) the hard remote e-stop (the orange one).
    - In Terminal 1, launch the software stack for the robot:
      ```bash
      # Every time you restart the drivers also run this:
      rosrun cosmo memory_remover
      # Run the drivers.
      roslaunch mabi_mobile_robot hilti.launch
      ```
      This will launch:
      - The robot drivers;
      - The controllers;
      - The URDF visualization;
      - The robot state publisher.

      NOTE: you should *not* hear any clicks in the robot arm at this point.
      
      In alternative, to run the software in simulation, cf. Terminal 2 in step 3. below.

      In case this launch file is terminated (e.g., `Ctrl+C`) or some of its nodes crashes, you might have to fix some memory errors, by running `rosrun cosmo memory_remover`, which should display `Sucessfully removed shared memory pool 'COSMO_SHM'` in return.
    - Enable the arm as follows. NOTE: Make sure you know what you're doing:
      - In Terminal 2, run:
        ```bash
        # First disable the arm (in case it not disabled).
        rosservice call /mabi_lowlevel_controller/enable_slave "id: 0 action: 0"
        ```
      - Disable (i.e., pull the red button out) the hard remote e-stop (the orange one).
      - In Terminal 2, run:
        ```bash
        # Enable all joints in the arm.
        rosservice call /mabi_lowlevel_controller/enable_slave "id: 0 action: 1"
        ```  
        NOTE: you should now hear "clicks" in the robot arm.
      - You can now also manually move the arm to an initial position, e.g., using the joint position controller.
        ```bash
        # "Master" controller, allows to select the joint-position controller.
        rosservice call /mabi_mobile_highlevel_controller/controller_manager/switch_controller MabiMobileCombinedController
        # Joint-position controller.
        rosservice call /mabi_highlevel_controller/controller_manager/switch_controller MabiJointPositionController
        # Go to storage/home position.
        rosrun mabi_joint_position_controller go_storage.py  # Use go_home.py instead to go to home.
        ```
        NOTE: you can skip this if you run the routine from the steps below, as it will automatically bring the robot to the initial position of the routine (which in general is not the home/storage position) and internally activate the controllers.
3. On the local computer:
    - In Terminal 1, launch the interface to align the building model to the robot.
      ```bash
      roslaunch cpt_selective_icp cpt_selective_icp_hal_routine.launch
      ```
    - Align the mesh model to the laser measurements from the robot, using the interface launched by the previous step.
    - If the software should be run in simulation, in Terminal 2, launch the simulation GUI:
      ```bash
      roslaunch mabi_sim run_sim_hal_routine.launch
      ```
      Also, change `arm_controller_switch_service_name` to `/mabi_highlevel_controller/controller_manager/switch_controller` in `cpt_pointlaser_ctrl_ros/cfg/ee_poses_visitor_params_mabi.yaml`.
4. On the robot, either in Terminal 2 or in a new Terminal, launch the actual HAL module:
    ```bash
    # If running in simulation, use argument `simulation:=true` and set the parameter `simulation_mode` to
    # `true` in `cpt_pointlaser_ctrl_ros/cfg/ee_poses_visitor_params_mabi.yaml`.
    roslaunch cpt_pointlaser_hal run_hal_routine.launch
    ```
5. Either on the robot or on the local computer, in a new terminal launch the interface that will guide the user through the routine, and follow the instructions to execute the routine:
    ```bash
    roslaunch cpt_pointlaser_hal hal_routine_guide.launch
    ```