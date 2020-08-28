cmake_minimum_required(VERSION 2.8.3)
project(cpt_geomesh_tools)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)

add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############


############
# BINARIES #
############
cs_add_executable(filter_center_node src/filter_center_node.cc)
target_link_libraries(filter_center_node CGAL::CGAL)




##########
# EXPORT #
##########
cs_install()
cs_export()
