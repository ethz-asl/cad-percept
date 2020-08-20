cmake_minimum_required(VERSION 2.8.3)
project(cpt_object_detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
    src/detection_matcher.cpp
)

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
    src/detection_matcher_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
