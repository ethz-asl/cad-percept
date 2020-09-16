cmake_minimum_required(VERSION 2.8.3)
project(cpt_object_detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)

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
target_link_libraries(${PROJECT_NAME}_node teaserpp::teaser_registration)

##########
# EXPORT #
##########
cs_install()
cs_export()
