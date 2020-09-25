cmake_minimum_required(VERSION 3.0.2)
project(cpt_pointlaser_loc)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/cpt_pointlaser_loc/localizer/localizer.cpp
  src/cpt_pointlaser_loc/optimizer/geometric_utils.cpp
  src/cpt_pointlaser_loc/optimizer/optimizer.cpp
)

cs_install()
cs_export()