cmake_minimum_required(VERSION 2.8.3)
project(cpt_meshing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/preprocessing/pre_processing_filter.cc
        src/meshing_node.cc
        )

############
# BINARIES #
############
cs_add_executable(cpt_meshing_node src/cpt_meshing_node.cpp)
target_link_libraries(cpt_meshing_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
