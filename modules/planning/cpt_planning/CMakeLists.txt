cmake_minimum_required(VERSION 2.8.3)
project(cpt_planning)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})
set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Wextra -Werror=return-type)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/coordinates/triangle_coords.cc
        src/coordinates/uv_mapping.cc
        src/interface/mesh_manifold_interface.cpp
        src/interface/linear_manifold_interface.cpp
        src/implementation/rmp_mesh_planner.cpp
        src/implementation/rmp_linear_planner.cpp
        )

##########
# EXPORT #
##########
cs_install()
cs_export()
