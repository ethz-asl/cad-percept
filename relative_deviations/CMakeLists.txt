cmake_minimum_required(VERSION 2.8.3)
project(relative_deviations)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/relative_deviations_ros.cpp
        src/deviations.cpp
        )

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
        src/relative_deviations_ros_node.cpp
        )

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

############
#  TESTS   #
############
catkin_add_gtest(${PROJECT_NAME}_tests
    test/test_main.cpp
    test/test_deviations.cpp)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
