# cad-percept
Bringing meshes to robotics.

![Cat](https://github.com/ethz-asl/cad-percept/blob/master/meshcat.png?raw=true)

# Modularization
In a default build, only the _core_ modules are built. These are the moduls in the core folder and contain the necessary typdefs, conversion and logging tools, ros interfaces and RVIZ plugins.
All other modules are structured according to their overall theme (construction, meshing, localization, planning) and are in the _modules_ folder. These are by default marked with a CATKIN_IGNORE, meaning they are not built per default.

These modules might pull in arbitrary dependencies and are currently not guaranteed to be in a working state.

# Installation

Note that cad-percept was updated to CGAL Version 5. For the cgal_catkin dependency, please use the appropriate branch (feature/5.0.3) until it is merged.

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
wstool merge cad-percept/dependencies.rosinstall
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
