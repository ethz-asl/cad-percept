cmake_minimum_required(VERSION 2.8.3)
project(cpt_localization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/mesh_localizer.cpp
        src/mesh_localizer_ros.cpp
        )


############
# BINARIES #
############
cs_add_executable(mesh_localizer_ros_node
        src/mesh_localizer_ros_node.cpp
        )
target_link_libraries(mesh_localizer_ros_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
