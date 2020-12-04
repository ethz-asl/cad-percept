cmake_minimum_required(VERSION 3.0.2)
project(cpt_pointlaser_hal)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/hal_routine_executer.cpp
)

cs_install()
cs_export()