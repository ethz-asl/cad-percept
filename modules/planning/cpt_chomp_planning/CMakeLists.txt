cmake_minimum_required(VERSION 2.8.3)
project(cpt_chomp_planning)
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})
add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_executable(test_node
        src/chomp_mesh_planner.cpp
        src/chomp.cpp
        src/test.cpp)

##########
# EXPORT #
##########
cs_install()
cs_export()
