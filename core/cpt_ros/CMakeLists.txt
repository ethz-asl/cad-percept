cmake_minimum_required(VERSION 2.8.3)
project(cpt_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++17 -Wall -Wextra -fopenmp)


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


cs_add_executable(rmp_test_node
        src/nodes/rmp_test_node.cc
        )
target_link_libraries(rmp_test_node ${PROJECT_NAME} -fopenmp)

cs_add_executable(rmp_coordinate_node
        src/nodes/coordinate_node.cc
        )
target_link_libraries(rmp_coordinate_node ${PROJECT_NAME} -fopenmp)

cs_add_executable(rmp_distance_node
        src/nodes/distance_test_node.cc
        )
target_link_libraries(rmp_distance_node ${PROJECT_NAME} -fopenmp)

cs_add_executable(rmp_zdistance_node
        src/nodes/zdistance_test_node.cc
        )
target_link_libraries(rmp_zdistance_node ${PROJECT_NAME} -fopenmp)


cs_add_executable(rmp_freespace_node
        src/nodes/freespace_test_node.cc
        )
target_link_libraries(rmp_freespace_node ${PROJECT_NAME} -fopenmp)

cs_add_executable(voliro_planner_node
        src/nodes/voliro_planner.cc
        )
target_link_libraries(voliro_planner_node ${PROJECT_NAME} -fopenmp)

cs_add_executable(param_test_node
        src/nodes/param_test_node.cc
        )
target_link_libraries(param_test_node ${PROJECT_NAME} -fopenmp)

############
#  TESTS   #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
