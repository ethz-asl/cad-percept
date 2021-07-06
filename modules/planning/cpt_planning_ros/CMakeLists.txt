cmake_minimum_required(VERSION 2.8.3)
project(cpt_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++17 -Wall -Wextra -fopenmp)


#############
# Executables #
#############

cs_add_executable(voliro_planner_node
        src/omav_planner.cc
        src/omav_planner_node.cc
        )


############
#  TESTS   #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
