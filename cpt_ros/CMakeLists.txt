cmake_minimum_required(VERSION 2.8.3)
project(cpt_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/ros_config_provider.cc
        )

############
# BINARIES #
############

############
#  TESTS   #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
