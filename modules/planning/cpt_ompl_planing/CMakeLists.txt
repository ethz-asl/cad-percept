cmake_minimum_required(VERSION 2.8.3)
project(cpt_ompl_planning)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#        )

##########
# EXPORT #
##########
cs_install()
cs_export()
