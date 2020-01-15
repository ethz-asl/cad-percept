cmake_minimum_required(VERSION 2.8.3)
project(cpt_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/ros_config_provider.cc
        )

############
# BINARIES #
############
cs_add_executable(collision_manifold_test_node
        src/nodes/collision_manifold_test_node.cc
        )

target_link_libraries(collision_manifold_test_node ${PROJECT_NAME})

cs_add_executable(planning_test_node
        src/nodes/planning_test_node.cc
        )
target_link_libraries(planning_test_node ${PROJECT_NAME})

############
#  TESTS   #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
