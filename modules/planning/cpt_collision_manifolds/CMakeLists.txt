cmake_minimum_required(VERSION 2.8.3)
project(cpt_collision_manifolds)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/iso_surface_manifold.cc
        src/offset_surface/vertex_normal_strategy.cc
        src/offset_surface/meshdomain_strategy.cc
        )

##########
# EXPORT #
##########
cs_install()
cs_export()
