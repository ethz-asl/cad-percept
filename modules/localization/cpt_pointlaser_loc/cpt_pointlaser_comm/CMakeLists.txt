cmake_minimum_required(VERSION 2.8.3)
project(cpt_pointlaser_comm)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############

cs_add_library(cpt_pointlaser_comm_lib
  src/distobox_serial.cpp
)

############
# BINARIES #
############

cs_add_executable(cpt_pointlaser_comm
  src/pointlaser_comm_node.cpp
)
target_link_libraries(cpt_pointlaser_comm cpt_pointlaser_comm_lib)

##########
# EXPORT #
##########
cs_install()
cs_export()
