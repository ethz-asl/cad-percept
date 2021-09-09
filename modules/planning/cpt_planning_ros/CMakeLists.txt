cmake_minimum_required(VERSION 2.8.3)
project(cpt_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++17 -Wall -Wextra -fopenmp)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#############
# Executables #
#############

cs_add_executable(omav_planner_node
        src/omav_planner.cc
        src/omav_planner_node.cc
        )

cs_add_executable(voliro_rope_planner_node
        src/voliro_rope_planner_node.cc
        src/voliro_rope_planner.cc
        )

############
#  TESTS   #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
