cmake_minimum_required(VERSION 2.8.3)
project(cpt_changes)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/changes_ros.cc
        )


############
# BINARIES #
############
cs_add_executable(changes_ros_node
        src/changes_ros_node.cc
        )
        
target_link_libraries(changes_ros_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
