# cad-percept
Bringing meshes to robotics.

![Cat](https://github.com/ethz-asl/cad-percept/blob/master/meshcat.png?raw=true)

# Installation

## Install dependencies
See the [maplab tutorial](https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu#installing-on-ubuntu-1404-1604-experimental-1804)
for instructions to install ros and ccache.

For cad-percept, we require the following dependencies:

```
sudo apt install clang-format libgmp-dev libboost-all-dev libmpfr-dev ros-melodic-geometric-shapes
```

## Set up a workspace

(optional) create a python environment separate from your system-wide pyhton
to avoid version conflicts.

```
virtualenv --pyhton=python2 --no-site-packages py2
source py2/bin/activate
pip install --upgrade pip
pip install wstool catkin-tools empy
```

Now set up a catkin workspace

```
mkdir -p <workspace_name>/src
cd <workspace_name>
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
git clone git@github.com:ethz-asl/cad-percept.git
wstool init
wstool merge cad-perpect/dependencies.rosinstall
wstool update
```

To activate your environment next time, first activate the python environment
with `source py2/bin/activate` and then activate the catkin environment with
`source <workscape_name>/devel/setup.bash`.

# Tests

Run GTests:

```
catkin run_tests <package_name>
```
