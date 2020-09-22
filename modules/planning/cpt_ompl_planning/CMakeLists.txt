cmake_minimum_required(VERSION 2.8.3)
project(cpt_ompl_planning)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})
find_package(ompl REQUIRED)
add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#        src/ompl_surface_planner.cpp
#        )

add_executable(ompl_surface_planner_node
        src/ompl_mesh_sampling_planner.cpp
        )

target_link_libraries(ompl_surface_planner_node ${OMPL_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
