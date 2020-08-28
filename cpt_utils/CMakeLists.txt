cmake_minimum_required(VERSION 2.8.3)
project(cpt_utils)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

set(CMAKE_CXX_STANDARD 14)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/conversions.cc
        src/cpt_utils.cc
        src/perf.cc
        src/pc_processing.cpp
        src/mesh_publisher.cc
        )


############
# BINARIES #
############
cs_add_executable(mesh_publisher_node
        src/mesh_publisher_node.cc
        )
target_link_libraries(mesh_publisher_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
