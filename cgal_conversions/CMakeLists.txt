cmake_minimum_required(VERSION 2.8.3)
project(cgal_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#)


############
# BINARIES #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
