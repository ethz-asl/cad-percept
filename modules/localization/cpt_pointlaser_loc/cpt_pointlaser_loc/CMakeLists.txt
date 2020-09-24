cmake_minimum_required(VERSION 3.0.2)
project(cpt_pointlaser_loc)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cpt_pointlaser_loc.cpp
)

cs_install()
cs_export()