cmake_minimum_required(VERSION 2.8.3)
project(cpt_pointlaser_comm_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############

cs_add_library(cpt_pointlaser_comm_lib
  src/distobox_worker.cpp
)

############
# BINARIES #
############

cs_add_executable(distobox_comm_node
  src/distobox_comm_node.cpp
)
target_link_libraries(distobox_comm_node
  cpt_pointlaser_comm_lib
  ${catkin_LIBRARIES}
)

##########
# EXPORT #
##########
cs_install()
cs_export()
